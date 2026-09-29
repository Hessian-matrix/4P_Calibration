//! RTSP / 容器解码源：解码线程 → **latest-wins 单槽**。
//!
//! 帧分发与背压：
//! - 槽深固定 1，`publish` 直接覆盖旧帧、绝不排队 → 慢消费者只拿到**最新**帧，解码端永不被反压；
//! - 消费者用 `generation` 判断"有没有新帧"，`wait_changed` 事件驱动等待（不忙轮询）；
//! - 解码错误写进 `error`，不静默停摆；`stop()` 通知线程退出并 join。
//!
//! 输出统一为 mono8 灰度，使用 swscale 转换为无行填充的 GRAY8。
//!
//! 动手解码前先做运行时依赖核对（`native_dependency_info`）：链接到的 FFmpeg 必须与
//! 编译期头文件同 ABI major 且不低于编译期版本，支持 6.x..9.x；不匹配就**直接失败**，
//! 绝不静默回退到别的后端。

use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, OnceLock};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use ffmpeg::format::Pixel;
use ffmpeg::media::Type;
use ffmpeg::software::scaling::{Context as Scaler, Flags};
use ffmpeg::util::frame::video::Video as VideoFrame;
use ffmpeg::{Dictionary, codec, ffi, format};
use ffmpeg_next as ffmpeg;

#[derive(Debug, thiserror::Error)]
pub enum SourceError {
    #[error("ffmpeg error: {0}")]
    Ffmpeg(#[from] ffmpeg::Error),
    #[error("no video stream in {0}")]
    NoVideoStream(String),
    #[error("decoder failed: {0}")]
    Decoder(String),
    /// 运行时链接到的 FFmpeg 与编译期头文件 ABI 不一致：动手解码前就失败，绝不带病运行。
    #[error("ffmpeg runtime dependency mismatch: {0}")]
    NativeDeps(String),
}

/// 一路灰度帧（mono8，行优先、无 padding）。
#[derive(Clone, Debug)]
pub struct MonoTile {
    pub gray: Vec<u8>,
    pub width: u32,
    pub height: u32,
    /// 该路累计发布序号（从 1 开始）。
    pub index: u64,
    /// **解码器**给出的帧时间戳（ns，按流的时基换算；取不到时为 None）。
    ///
    /// 域 = 该解码会话重基后的域（RTSP 客户端常见做法：连上后从 0 起算）。它只出现在
    /// **RTSP 路径**；与板端时间戳之间是一个常数偏移，由 `clock::ClockAligner` 标定。
    pub pts_ns: Option<i64>,
    /// **板端**时间戳（ns，板端 realtime 域）。只出现在**板端 raw 服务路径**（`camera_timestamp_ns`）。
    ///
    /// 与 `pts_ns` 属于不同时间域，不可直接混用。
    pub board_timestamp_ns: Option<i64>,
}

impl MonoTile {
    /// 帧内容的 FNV-1a 摘要。
    pub fn digest(&self) -> String {
        let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
        for byte in &self.gray {
            hash ^= *byte as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        format!("{hash:016x}")
    }
}

#[derive(Default)]
struct SlotInner {
    frame: Option<Arc<MonoTile>>,
    generation: u64,
    error: Option<String>,
    finished: bool,
}

/// latest-wins 单槽。
pub struct FrameSlot {
    inner: Mutex<SlotInner>,
    ready: Condvar,
}

impl FrameSlot {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(SlotInner::default()),
            ready: Condvar::new(),
        }
    }

    fn publish(&self, frame: MonoTile) {
        let mut inner = self.inner.lock().unwrap_or_else(|error| error.into_inner());
        inner.frame = Some(Arc::new(frame));
        inner.generation += 1;
        self.ready.notify_all();
    }

    fn fail(&self, message: String) {
        let mut inner = self.inner.lock().unwrap_or_else(|error| error.into_inner());
        inner.error = Some(message);
        inner.finished = true;
        self.ready.notify_all();
    }

    fn finish(&self) {
        let mut inner = self.inner.lock().unwrap_or_else(|error| error.into_inner());
        inner.finished = true;
        self.ready.notify_all();
    }

    /// 最新帧（非消费式）。
    pub fn latest(&self) -> Option<Arc<MonoTile>> {
        self.inner
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .frame
            .clone()
    }

    pub fn generation(&self) -> u64 {
        self.inner
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .generation
    }

    /// 等“有新帧”或超时；返回是否有新帧。
    pub fn wait_changed(&self, seen_generation: u64, timeout: Duration) -> bool {
        let inner = self.inner.lock().unwrap_or_else(|error| error.into_inner());
        let (inner, timeout_result) = self
            .ready
            .wait_timeout_while(inner, timeout, |state| {
                state.generation == seen_generation && !state.finished
            })
            .unwrap_or_else(|error| error.into_inner());
        let _ = timeout_result;
        inner.generation != seen_generation
    }

    pub fn error(&self) -> Option<String> {
        self.inner
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .error
            .clone()
    }

    pub fn finished(&self) -> bool {
        self.inner
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .finished
    }
}

impl Default for FrameSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// ffmpeg 输入参数。
///
/// 低延迟参数只用于 RTSP；文件输入保留默认探测，避免解复用信息不完整。
fn input_options(url: &str, timeout: Duration) -> Dictionary<'static> {
    let mut options = Dictionary::new();
    if url.starts_with("rtsp://") {
        options.set("rtsp_transport", "tcp");
        options.set("fflags", "nobuffer");
        options.set("analyzeduration", "0");
        options.set("probesize", "32");
        // RTSP demuxer 的 socket 超时选项名是 `timeout`（µs）；旧的 `stimeout` 已被移除。
        options.set("timeout", &timeout.as_micros().to_string());
    }
    options
}

/// 支持的 FFmpeg 发布族（6.x..9.x）对应的 libavcodec ABI major 区间。
///
/// 各库的 ABI major 与发布族同步：6.x→60、7.x→61、8.x→62、9.x→63。头文件 ABI 与
/// 运行时 ABI 只要不同 major 就不能混用（ABI 在 major 边界上会变）。
const AVCODEC_ABI_SUPPORTED: std::ops::RangeInclusive<u32> = 60..=63;

/// 一个本地 FFmpeg 库的编译期头文件版本与运行期版本（均为 `AV_VERSION_INT` 打包形式）。
struct NativeLib {
    name: &'static str,
    /// 编译期（构建时头文件宏）版本。
    header: u32,
    /// 运行期（实际链接到的实现）版本。
    runtime: u32,
}

impl NativeLib {
    fn describe(&self, output: &mut String) {
        write!(
            output,
            "{} header={} runtime={}",
            self.name,
            NativeVersion(self.header),
            NativeVersion(self.runtime)
        )
        .expect("writing to a String cannot fail");
    }
}

/// 以 `AV_VERSION_INT` 的口径把三段版本打包成 32 位整数。
///
/// `ffmpeg-sys-next` 的 bindgen 只产出 `*_VERSION_MAJOR/MINOR/MICRO` 三个整数宏
/// （嵌套展开的 `*_VERSION_INT` 宏不会被导出），因此这里按 FFmpeg 的定义手工打包。
const fn packed_version(major: i32, minor: i32, micro: i32) -> u32 {
    (((major as u32) & 0xff) << 16) | (((minor as u32) & 0xff) << 8) | ((micro as u32) & 0xff)
}

const fn version_major(packed: u32) -> u32 {
    packed >> 16
}

const fn version_minor(packed: u32) -> u32 {
    (packed >> 8) & 0xff
}

const fn version_micro(packed: u32) -> u32 {
    packed & 0xff
}

struct NativeVersion(u32);

impl std::fmt::Display for NativeVersion {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "{}.{}.{}",
            version_major(self.0),
            version_minor(self.0),
            version_micro(self.0)
        )
    }
}

/// 四个库的编译期头文件版本与运行期版本：头文件来自构建时的宏，运行期来自实现本身。
fn native_libs() -> [NativeLib; 4] {
    [
        NativeLib {
            name: "libavcodec",
            header: packed_version(
                ffi::LIBAVCODEC_VERSION_MAJOR,
                ffi::LIBAVCODEC_VERSION_MINOR,
                ffi::LIBAVCODEC_VERSION_MICRO,
            ),
            runtime: codec::version(),
        },
        NativeLib {
            name: "libavformat",
            header: packed_version(
                ffi::LIBAVFORMAT_VERSION_MAJOR,
                ffi::LIBAVFORMAT_VERSION_MINOR,
                ffi::LIBAVFORMAT_VERSION_MICRO,
            ),
            runtime: format::version(),
        },
        NativeLib {
            name: "libavutil",
            header: packed_version(
                ffi::LIBAVUTIL_VERSION_MAJOR,
                ffi::LIBAVUTIL_VERSION_MINOR,
                ffi::LIBAVUTIL_VERSION_MICRO,
            ),
            runtime: ffmpeg::util::version(),
        },
        NativeLib {
            name: "libswscale",
            header: packed_version(
                ffi::LIBSWSCALE_VERSION_MAJOR,
                ffi::LIBSWSCALE_VERSION_MINOR,
                ffi::LIBSWSCALE_VERSION_MICRO,
            ),
            runtime: ffmpeg::software::scaling::version(),
        },
    ]
}

/// 执行一次完整的运行时核对；失败文本里同样列出四个库的两侧版本，便于现场定位。
fn verify_native_dependencies() -> Result<String, String> {
    let libs = native_libs();
    let mut detail = String::new();
    for lib in &libs {
        if !detail.is_empty() {
            detail.push_str("; ");
        }
        lib.describe(&mut detail);
    }

    let mut problems = String::new();
    for lib in &libs {
        if lib.name == "libavcodec" && !AVCODEC_ABI_SUPPORTED.contains(&version_major(lib.runtime))
        {
            write!(
                problems,
                "unsupported libavcodec runtime ABI major {} (supported {:?}, i.e. FFmpeg 6.x..9.x); ",
                version_major(lib.runtime),
                AVCODEC_ABI_SUPPORTED
            )
            .expect("writing to a String cannot fail");
        }
        let header_major = version_major(lib.header);
        let runtime_major = version_major(lib.runtime);
        if runtime_major != header_major {
            write!(
                problems,
                "{} runtime ABI major {} does not match compiled header ABI major {}; ",
                lib.name, runtime_major, header_major
            )
            .expect("writing to a String cannot fail");
        } else if lib.runtime < lib.header {
            write!(
                problems,
                "{} runtime {} is older than compiled header {}; ",
                lib.name,
                NativeVersion(lib.runtime),
                NativeVersion(lib.header)
            )
            .expect("writing to a String cannot fail");
        }
    }

    if problems.is_empty() {
        Ok(format!("ffmpeg runtime ok: {detail}"))
    } else {
        problems.push_str("all libraries: ");
        problems.push_str(&detail);
        Err(problems)
    }
}

/// 校验链接到的 FFmpeg 运行时与编译期头文件 ABI 是否一致。
///
/// 成功文本包含四个库（libavcodec/libavformat/libavutil/libswscale）的编译期头文件
/// 版本与实际运行时版本；失败文本除具体原因外也列出这四个库的两侧版本。判定规则：
/// 每个库的运行时 ABI major 必须与编译期头文件 major 相同，且运行时打包版本不得低于
/// 编译期打包版本；libavcodec 的运行时 ABI major 还必须落在 6.x..9.x（60..=63）。
///
/// 成功结果进程内缓存，`FrameSource::start` 与 `decode_gray_frames` 每次调用只读取一次，
/// 不会逐帧重复；失败**不缓存**，一次失败的启动不会永久毒化后续调用。
pub fn native_dependency_info() -> Result<&'static str, String> {
    if let Some(cached) = NATIVE_DEPS.get() {
        return Ok(cached.as_str());
    }
    let info = verify_native_dependencies()?;
    Ok(NATIVE_DEPS.get_or_init(|| info).as_str())
}

/// 成功结果缓存；启动和同步解码的重复检查不分配字符串。
static NATIVE_DEPS: OnceLock<String> = OnceLock::new();

/// 解码源：`start` 后后台线程持续解码，帧进单槽。
pub struct FrameSource {
    url: String,
    slot: Arc<FrameSlot>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl FrameSource {
    /// `pace`：按流帧率节流（把**文件**当在线源演练时用；RTSP 天生有节奏，传 false 即可）。
    pub fn start(
        url: &str,
        expected_size: (u32, u32),
        timeout: Duration,
        pace: bool,
    ) -> Result<Self, SourceError> {
        // ABI 不匹配时在碰 ffmpeg 之前就失败，绝不静默降级到不可用的后端。
        native_dependency_info().map_err(SourceError::NativeDeps)?;
        ffmpeg::init()?;
        let slot = Arc::new(FrameSlot::new());
        let stop = Arc::new(AtomicBool::new(false));
        let worker_slot = Arc::clone(&slot);
        let worker_stop = Arc::clone(&stop);
        let worker_url = url.to_owned();
        let handle = thread::Builder::new()
            .name("rigcal-decode".to_owned())
            .spawn(move || {
                if let Err(error) = decode_loop(
                    &worker_url,
                    expected_size,
                    timeout,
                    pace,
                    &worker_slot,
                    &worker_stop,
                ) {
                    worker_slot.fail(error.to_string());
                } else {
                    worker_slot.finish();
                }
            })
            .map_err(|error| SourceError::Decoder(error.to_string()))?;
        Ok(Self {
            url: url.to_owned(),
            slot,
            stop,
            handle: Some(handle),
        })
    }

    pub fn slot(&self) -> Arc<FrameSlot> {
        Arc::clone(&self.slot)
    }

    pub fn url(&self) -> &str {
        &self.url
    }

    /// 等首帧（或超时）；返回是否拿到帧。
    pub fn wait_first_frame(&self, timeout: Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline {
            if self.slot.latest().is_some() {
                return true;
            }
            if self.slot.finished() {
                return false;
            }
            thread::sleep(Duration::from_millis(10));
        }
        false
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for FrameSource {
    fn drop(&mut self) {
        self.stop();
    }
}

/// 灰度化：把解码出的视频帧转到期望尺寸的 mono8；`stride` 可能大于宽度，必须按行裁剪。
fn to_mono8(
    scaler: &mut Scaler,
    frame: &VideoFrame,
    expected_size: (u32, u32),
    target: &mut VideoFrame,
) -> Result<Vec<u8>, SourceError> {
    let (width, height) = expected_size;
    scaler.run(frame, target)?;
    let stride = target.stride(0);
    let data = target.data(0);
    let mut gray = Vec::with_capacity((width * height) as usize);
    for row in 0..height as usize {
        let start = row * stride;
        gray.extend_from_slice(&data[start..start + width as usize]);
    }
    Ok(gray)
}

/// 解码回调的控制流：`Stop` 让解码器提前收工（消费者已不需要更多帧）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flow {
    Continue,
    Stop,
}

/// 同步顺序解码：每一帧按序交给 `sink`，返回交付帧数。
///
/// `FrameSource` 将它作为解码线程主体；探针可直接消费顺序帧。
pub fn decode_gray_frames(
    url: &str,
    expected_size: (u32, u32),
    timeout: Duration,
    pace: bool,
    mut sink: impl FnMut(MonoTile) -> Flow,
) -> Result<usize, SourceError> {
    // 同步路径与线程路径共用同一道闸：ABI 不匹配时在打开输入前失败（成功结果已缓存，不逐帧重算）。
    native_dependency_info().map_err(SourceError::NativeDeps)?;
    let options = input_options(url, timeout);
    let mut input = format::input_with_dictionary(&url, options)?;
    let stream = input
        .streams()
        .best(Type::Video)
        .ok_or_else(|| SourceError::NoVideoStream(url.to_owned()))?;
    let stream_index = stream.index();
    // `pace` 才读帧率：播放**文件**当在线源演练时必须按真实节奏出帧，否则 wall-clock 门禁
    // （检测节奏 0.5/12 Hz、冷却 0.5 s）在文件瞬间解完时形同虚设。
    let stream_fps = if pace {
        let rate = stream.avg_frame_rate();
        if rate.numerator() > 0 && rate.denominator() > 0 {
            Some(rate.numerator() as f64 / rate.denominator() as f64)
        } else {
            None
        }
    } else {
        None
    };
    // 帧时间戳的单位必须由 demuxed stream 明确传入解码器，否则 `frame.pts()` 的单位不确定。
    let stream_time_base = stream.time_base();
    let mut context = codec::context::Context::from_parameters(stream.parameters())?;
    if url.starts_with("rtsp://") {
        // `flags=low_delay` 放在 demuxer 选项里是无效的，必须设在码流上下文。
        context.set_flags(codec::flag::Flags::LOW_DELAY);
    }
    let mut decoder = context.decoder();
    decoder.set_packet_time_base(stream_time_base);
    let mut decoder = decoder.video()?;
    let ticks_to_ns = if stream_time_base.denominator() != 0 {
        stream_time_base.numerator() as f64 / stream_time_base.denominator() as f64 * 1e9
    } else {
        0.0
    };

    let mut state = DecodeState {
        scaler: None,
        source_frame: VideoFrame::empty(),
        target_frame: VideoFrame::empty(),
        published: 0,
        ticks_to_ns,
    };
    let mut packets_video = 0usize;
    let playback_started = std::time::Instant::now();

    for (stream, packet) in input.packets() {
        if stream.index() != stream_index {
            continue;
        }
        packets_video += 1;
        decoder.send_packet(&packet)?;
        if drain_decoder(&mut decoder, &mut state, expected_size, &mut sink)? {
            return Ok(state.published as usize);
        }
        if let Some(fps) = stream_fps {
            let target = playback_started + Duration::from_secs_f64(state.published as f64 / fps);
            if let Some(wait) = target.checked_duration_since(std::time::Instant::now()) {
                thread::sleep(wait);
            }
        }
    }
    // 末尾冲刷：解码器的重排缓冲里可能还压着帧（B 帧/低延迟模式），不发 EOF 就会少几帧。
    // 文件路径上这很直观（20 帧只出 18 帧）；RTSP 上表现为停止后丢尾帧。
    decoder.send_eof()?;
    drain_decoder(&mut decoder, &mut state, expected_size, &mut sink)?;

    if state.published == 0 {
        return Err(SourceError::Decoder(format!(
            "no frames decoded from {url} ({packets_video} video packet(s) submitted)"
        )));
    }
    Ok(state.published as usize)
}

/// 解码器的输出缓冲、scaler 与计数（`decode_gray_frames` 的状态）。
struct DecodeState {
    scaler: Option<Scaler>,
    source_frame: VideoFrame,
    target_frame: VideoFrame,
    published: u64,
    /// 流时基的一格等于多少 ns。
    ticks_to_ns: f64,
}

/// 把解码器当前可取的帧全部取出交给 `sink`；返回 `true` 表示 `sink` 要求提前收工。
fn drain_decoder(
    decoder: &mut codec::decoder::Video,
    state: &mut DecodeState,
    expected_size: (u32, u32),
    sink: &mut impl FnMut(MonoTile) -> Flow,
) -> Result<bool, SourceError> {
    loop {
        if decoder.receive_frame(&mut state.source_frame).is_err() {
            return Ok(false);
        }
        if state.scaler.is_none() {
            state.scaler = Some(Scaler::get(
                state.source_frame.format(),
                state.source_frame.width(),
                state.source_frame.height(),
                Pixel::GRAY8,
                expected_size.0,
                expected_size.1,
                Flags::BILINEAR,
            )?);
        }
        let Some(active) = state.scaler.as_mut() else {
            return Err(SourceError::Decoder(
                "scaler was not initialised".to_owned(),
            ));
        };
        let gray = to_mono8(
            active,
            &state.source_frame,
            expected_size,
            &mut state.target_frame,
        )?;
        let pts_ns = state
            .source_frame
            .pts()
            .map(|pts| (pts as f64 * state.ticks_to_ns).round() as i64);
        state.published += 1;
        if sink(MonoTile {
            gray,
            width: expected_size.0,
            height: expected_size.1,
            index: state.published,
            pts_ns,
            board_timestamp_ns: None,
        }) == Flow::Stop
        {
            return Ok(true);
        }
    }
}

/// 解码线程主体：解出的每帧进 latest-wins 槽，直到 `stop` 置位。
fn decode_loop(
    url: &str,
    expected_size: (u32, u32),
    timeout: Duration,
    pace: bool,
    slot: &FrameSlot,
    stop: &AtomicBool,
) -> Result<(), SourceError> {
    decode_gray_frames(url, expected_size, timeout, pace, |tile| {
        slot.publish(tile);
        if stop.load(Ordering::Acquire) {
            Flow::Stop
        } else {
            Flow::Continue
        }
    })?;
    Ok(())
}
