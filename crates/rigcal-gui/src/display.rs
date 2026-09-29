//! 预览显示链路：唯一后台 worker 从四路 `FrameSlot` 取**最新**帧 → 缩到预览宽 →
//! 叠加**最近一次**检测结果 → RGBA → 由 GUI 线程写进持久的 `frames` 模型。
//!
//! 显示与采集、分析解耦：
//! - **只认最新帧**：槽位是 latest-wins 单槽，本 worker 以 `MonoTile::index` 去重（[`FrameCursor`]）。
//!   检测、时钟采样和数值求解不阻塞此路径；
//! - **叠加只是反馈**：检测与画面不同源、可以差若干帧。Main 每次拿到检测结果就换掉共享的
//!   `Arc<Detection>`（无板时清空），本模块把**当前**手里那份按预览比例画到**当前**这帧上，
//!   既不等分析、也不要求叠加与帧对齐；
//! - **像素不在锁里搬**：worker 准备好 `SharedPixelBuffer`（引用计数，克隆廉价），`Image` 只在
//!   GUI 线程创建（`Image` 非 `Send`，见 Slint `graphics::image` 文档）；共享锁内只做
//!   “换一个槽位 / 取走一个缓冲”，没有深复制。
//!
//! 通知合并：worker 只置一个 `pending` 标志，最多让**一个** `invoke_from_event_loop` 在途；
//! GUI 侧处理时**先撤标志、再取槽**，所以被合并掉的中间帧不会丢唤醒（见 [`Shared::take`]）；
//! 事件循环没起来或已结束时不空等回调——投递失败即撤标志，下一帧重新通知。
//!
//! 用法（Main 侧）：
//! ```ignore
//! let handle = display::start(&ui, inputs, Arc::clone(&stop))?; // inputs: 每路 slot + overlay
//! ui.run()?;
//! handle.finish()?; // 停 worker + join + 打印一行计数（运行中可 `handle.stats()` 取快照）
//! ```

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use opencv::core::{Mat, Size, Vec4b};
use opencv::imgproc;
use opencv::prelude::*;

use slint::{ComponentHandle, Image, Model, ModelRc, Rgba8Pixel, SharedPixelBuffer, VecModel};

use rigcal_camera::detect::Detection;
use rigcal_camera::preview;
use rigcal_io::rtsp::{FrameSlot, MonoTile};

use crate::Dashboard;

/// 预览格目标宽度。
const PREVIEW_WIDTH: u32 = 480;

/// 空闲等待上限：等不到新帧也要能及时看见 `stop`（毫秒级，不是逐帧日志）。
const IDLE_WAIT: Duration = Duration::from_millis(4);

/// 一路预览的输入：该路解码槽 + 该路“最近一次检测”的共享槽（`None` = 当前无叠加）。
///
/// `overlay` 由 Main 的分析线程写、本模块每帧读一次；两边都只换 `Arc`，不搬像素。
pub struct PreviewInput {
    pub slot: Arc<FrameSlot>,
    pub overlay: Arc<Mutex<Option<Arc<Detection>>>>,
}

/// 预览链路的低成本计数（运行中全是原子读，`finish` 时打印一行）。
///
/// - `prepared`：worker 完成“缩放 + 叠加 + 转 RGBA”的帧数（≈ 各路的源新帧数）；
/// - `submitted`：真正写进 Slint `frames` 模型的帧数（≤ `prepared`，差额是被 latest-wins
///   覆盖掉的中间结果）。这是**渲染通知**次数，不等于屏幕物理呈现次数；
/// - `frame_gaps`：源侧被跳过的帧数（相邻两次准备之间 `index` 差 - 1 累计），直接反映
///   “源新帧率 vs 预览准备速率”；
/// - `prepare_errors`：准备失败的帧数（静默计数，不逐帧打日志）；
/// - `notifications`：实际投递给 GUI 事件循环的**合并后**通知次数。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PreviewStats {
    pub prepared: u64,
    pub submitted: u64,
    pub frame_gaps: u64,
    pub prepare_errors: u64,
    pub notifications: u64,
}

/// 预览 worker 的句柄：Main 只需在关闭时 [`PreviewHandle::finish`]。
pub struct PreviewHandle {
    /// 只属于本 worker 的停止位：`finish` 置位不会顺手停掉 Main 的其它 worker。
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
    shared: Arc<Shared>,
}

impl PreviewHandle {
    /// 计数快照（运行中可读）。
    pub fn stats(&self) -> PreviewStats {
        self.shared.counters.snapshot()
    }

    /// 停 worker、join，并打印一行计数汇总（这是本模块唯一的日志 IO）。
    pub fn finish(mut self) -> Result<(), String> {
        let joined = self.join();
        let stats = self.shared.counters.snapshot();
        eprintln!(
            "preview: prepared={} submitted={} frame_gaps={} prepare_errors={} notifications={}",
            stats.prepared,
            stats.submitted,
            stats.frame_gaps,
            stats.prepare_errors,
            stats.notifications
        );
        joined
    }

    fn join(&mut self) -> Result<(), String> {
        self.stop.store(true, Ordering::SeqCst);
        match self.worker.take() {
            Some(handle) => handle
                .join()
                .map_err(|_| "预览 worker 线程 panic".to_string()),
            None => Ok(()),
        }
    }
}

impl Drop for PreviewHandle {
    fn drop(&mut self) {
        // 漏调 `finish` 也不能留下野线程（`join` 幂等：worker 已被 take 时是空操作）。
        let _ = self.join();
    }
}

/// 计数字段（worker 写、GUI 线程写 `submitted`、任意线程读快照）。
#[derive(Default)]
struct Counters {
    prepared: AtomicU64,
    submitted: AtomicU64,
    frame_gaps: AtomicU64,
    prepare_errors: AtomicU64,
    notifications: AtomicU64,
}

impl Counters {
    fn snapshot(&self) -> PreviewStats {
        let relaxed = Ordering::Relaxed;
        PreviewStats {
            prepared: self.prepared.load(relaxed),
            submitted: self.submitted.load(relaxed),
            frame_gaps: self.frame_gaps.load(relaxed),
            prepare_errors: self.prepare_errors.load(relaxed),
            notifications: self.notifications.load(relaxed),
        }
    }
}

/// worker 与 GUI 线程之间的共享态：latest-wins 待显示槽 + 合并通知标志 + 计数。
struct Shared {
    /// 每路“已准备好、等 GUI 取走”的最新一帧（取走后为 `None`）。
    slots: Mutex<Vec<Option<SharedPixelBuffer<Rgba8Pixel>>>>,
    /// 是否已有一个 UI 通知在途（用于把多次生产合并成一次唤醒）。
    notify_pending: AtomicBool,
    counters: Counters,
}

impl Shared {
    fn new(inputs: usize) -> Self {
        Self {
            slots: Mutex::new((0..inputs).map(|_| None).collect()),
            notify_pending: AtomicBool::new(false),
            counters: Counters::default(),
        }
    }

    /// worker 侧：换掉该路待显示帧。返回 `true` = 调用方需要投递一次 UI 通知。
    fn publish(&self, index: usize, frame: SharedPixelBuffer<Rgba8Pixel>) -> bool {
        if let Ok(mut slots) = self.slots.lock()
            && let Some(slot) = slots.get_mut(index)
        {
            *slot = Some(frame);
        }
        !self.notify_pending.swap(true, Ordering::SeqCst)
    }

    /// GUI 侧：取走所有待显示帧。
    ///
    /// **顺序是约定的一部分**：先撤 `pending` 再取槽。若反过来（取完再撤），生产者可能在两者
    /// 之间看到 `pending == true` 而跳过通知，那一帧就永远不上屏了。
    fn take(&self) -> Vec<Option<SharedPixelBuffer<Rgba8Pixel>>> {
        self.notify_pending.store(false, Ordering::SeqCst);
        match self.slots.lock() {
            Ok(mut slots) => slots.iter_mut().map(Option::take).collect(),
            Err(_) => Vec::new(),
        }
    }

    /// 通知没能投出去（事件循环未起/已结束）：撤标志，让下一帧重新通知。
    fn abandon_notification(&self) {
        self.notify_pending.store(false, Ordering::SeqCst);
    }
}

/// 每路“已准备到哪一帧”的游标：以 `MonoTile::index` 去重。
#[derive(Default)]
struct FrameCursor {
    last: u64,
}

impl FrameCursor {
    /// `None` = 不是新帧；`Some(skipped)` = 新帧，以及其间被 latest-wins 丢弃的源帧数。
    fn accept(&mut self, index: u64) -> Option<u64> {
        if index <= self.last {
            return None;
        }
        let skipped = index - self.last - 1;
        self.last = index;
        Some(skipped)
    }
}

/// 跨帧复用的 OpenCV 缓冲（缩放结果 + 叠加画布），避免每帧重新分配。
#[derive(Default)]
struct Scratch {
    gray: Mat,
    bgr: Mat,
}

/// 启动唯一预览 worker，并在 `ui` 上建立**持久**的 `frames` 模型。
///
/// 模型只在这里建一次；之后 worker 侧按行更新（`set_row_data`），GUI 不重建模型。
/// 返回的句柄负责停止与 join；`stop` 是 Main 自己的停止位，worker 同时监听它。
pub fn start(
    ui: &Dashboard,
    inputs: Vec<PreviewInput>,
    stop: Arc<AtomicBool>,
) -> std::io::Result<PreviewHandle> {
    let count = inputs.len();
    ui.set_frames(ModelRc::new(VecModel::from(vec![Image::default(); count])));
    let shared = Arc::new(Shared::new(count));
    let own_stop = Arc::new(AtomicBool::new(false));
    let ui_weak = ui.as_weak();
    let worker_shared = Arc::clone(&shared);
    let worker_stop = Arc::clone(&stop);
    let worker_own_stop = Arc::clone(&own_stop);
    let spawned = thread::Builder::new()
        .name("preview".to_string())
        .spawn(move || worker(inputs, worker_shared, ui_weak, worker_stop, worker_own_stop))?;
    Ok(PreviewHandle {
        stop: own_stop,
        worker: Some(spawned),
        shared,
    })
}

/// worker 主体：公平轮询四路槽位，只处理新帧；空闲时轮转等待（不忙轮询）。
fn worker(
    inputs: Vec<PreviewInput>,
    shared: Arc<Shared>,
    ui: slint::Weak<Dashboard>,
    stop: Arc<AtomicBool>,
    own_stop: Arc<AtomicBool>,
) {
    let mut cursors: Vec<FrameCursor> = inputs.iter().map(|_| FrameCursor::default()).collect();
    let mut scratch = Scratch::default();
    let mut next_wait = 0usize;
    while !(stop.load(Ordering::Relaxed) || own_stop.load(Ordering::Relaxed)) {
        let mut progressed = false;
        let mut notify = false;
        for (index, input) in inputs.iter().enumerate() {
            let Some(tile) = input.slot.latest() else {
                continue;
            };
            let Some(skipped) = cursors[index].accept(tile.index) else {
                continue;
            };
            progressed = true;
            if skipped > 0 {
                shared
                    .counters
                    .frame_gaps
                    .fetch_add(skipped, Ordering::Relaxed);
            }
            // 锁只用来换走 `Arc`，叠加的绘制在锁外做。
            let overlay = input
                .overlay
                .lock()
                .ok()
                .and_then(|held| held.as_ref().map(Arc::clone));
            match prepare(&tile, overlay.as_deref(), &mut scratch) {
                Ok(buffer) => {
                    shared.counters.prepared.fetch_add(1, Ordering::Relaxed);
                    notify |= shared.publish(index, buffer);
                }
                Err(_) => {
                    shared
                        .counters
                        .prepare_errors
                        .fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if notify {
            notify_gui(&shared, &ui);
        }
        if progressed {
            continue;
        }
        if inputs.is_empty() || inputs.iter().all(|input| input.slot.finished()) {
            break; // 全部源都已结束：不会再有新帧。
        }
        let index = next_wait % inputs.len();
        next_wait = index + 1;
        let generation = inputs[index].slot.generation();
        inputs[index].slot.wait_changed(generation, IDLE_WAIT);
    }
}

/// 投递一次（最多一个在途的）UI 通知：GUI 线程取走最新帧并写进 `frames` 模型。
fn notify_gui(shared: &Arc<Shared>, ui: &slint::Weak<Dashboard>) {
    let state = Arc::clone(shared);
    let ui_weak = ui.clone();
    state.counters.notifications.fetch_add(1, Ordering::Relaxed);
    let queued = slint::invoke_from_event_loop(move || {
        let Some(ui) = ui_weak.upgrade() else {
            state.abandon_notification();
            return;
        };
        // 模型只可能是 `start` 建的那个 `VecModel`；不是（有人换了模型）就原样留着槽位，
        // 撤掉标志等下一次通知——绝不在这里换成闭包自带的那帧。
        let frames = ui.get_frames();
        let Some(model) = frames.as_any().downcast_ref::<VecModel<Image>>() else {
            state.abandon_notification();
            return;
        };
        for (index, frame) in state.take().into_iter().enumerate() {
            if let Some(frame) = frame {
                model.set_row_data(index, Image::from_rgba8(frame));
                state.counters.submitted.fetch_add(1, Ordering::Relaxed);
            }
        }
    });
    if queued.is_err() {
        shared.abandon_notification();
    }
}

/// 一帧的预览准备：全分辨率灰度（借用头，零拷贝）→ 缩到预览宽 →（有叠加时）画检测 → RGBA。
///
/// 全程只生成**预览尺寸**的图像，不生成全分辨率 BGR。
fn prepare(
    tile: &MonoTile,
    overlay: Option<&Detection>,
    scratch: &mut Scratch,
) -> Result<SharedPixelBuffer<Rgba8Pixel>, opencv::Error> {
    // 解码缓冲本来就是连续 mono8：直接建 Mat 头，不复制整帧（`tile` 在本函数内存活）。
    let decoded =
        Mat::new_rows_cols_with_bytes::<u8>(tile.height as i32, tile.width as i32, &tile.gray)?;
    let (width, height) = preview_size(tile.width, tile.height);
    imgproc::resize(
        &decoded,
        &mut scratch.gray,
        Size::new(width as i32, height as i32),
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;

    let mut buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    match overlay {
        Some(detection) => {
            // `cvt_color_def` = `cvt_color(src, dst, code, dst_cn=0, hint=ALGO_HINT_DEFAULT)`：
            // 默认参数版本在 4.x（无 `AlgorithmHint`）与 5.x 同名同义。
            imgproc::cvt_color_def(&scratch.gray, &mut scratch.bgr, imgproc::COLOR_GRAY2BGR)?;
            // 角点是全分辨率坐标：只在真的缩过时折算，不缩就不复制检测结果。
            let scaled =
                (width != tile.width).then(|| scale_detection(detection, width, tile.width));
            preview::draw_detection(&mut scratch.bgr, scaled.as_ref().unwrap_or(detection), "")?;
            rgba_from(&scratch.bgr, imgproc::COLOR_BGR2RGBA, &mut buffer)?;
        }
        None => rgba_from(&scratch.gray, imgproc::COLOR_GRAY2RGBA, &mut buffer)?,
    }
    Ok(buffer)
}

/// 把 `source` 转换到 RGBA，**直接写进** `buffer`（不经中间 Mat、不做第二次拷贝）。
fn rgba_from(
    source: &Mat,
    code: i32,
    buffer: &mut SharedPixelBuffer<Rgba8Pixel>,
) -> Result<(), opencv::Error> {
    // 类型必须是 `CV_8UC4`（`Vec4b` = `VecN<u8, 4>`）：只有尺寸**和类型**都与转换结果一致，
    // OpenCV 的 `create` 才会原地复用这块外部缓冲；否则它会另分配一块，画面全黑。
    let (rows, cols) = (source.rows(), source.cols());
    let mut target =
        Mat::new_rows_cols_with_bytes_mut::<Vec4b>(rows, cols, buffer.make_mut_bytes())?;
    imgproc::cvt_color_def(source, &mut target, code)
}

/// 预览尺寸：**只缩不放大**（源本来就不宽于预览宽时保持原样，不重采样）。
fn preview_size(width: u32, height: u32) -> (u32, u32) {
    if width <= PREVIEW_WIDTH {
        return (width.max(1), height.max(1));
    }
    let height = ((height as f64 * PREVIEW_WIDTH as f64 / width as f64).round() as u32).max(1);
    (PREVIEW_WIDTH, height)
}

/// 检测角点（全分辨率像素）→ 预览像素：不折算就会画到画面外。
fn scale_detection(detection: &Detection, preview_width: u32, source_width: u32) -> Detection {
    let scale = preview_width as f64 / source_width.max(1) as f64;
    let mut scaled = detection.clone();
    for point in &mut scaled.image_points {
        point[0] *= scale;
        point[1] *= scale;
    }
    scaled
}

#[cfg(test)]
mod tests {
    use super::*;

    fn buffer(value: u8) -> SharedPixelBuffer<Rgba8Pixel> {
        let mut buffer = SharedPixelBuffer::<Rgba8Pixel>::new(2, 2);
        buffer.make_mut_slice().fill(Rgba8Pixel {
            r: value,
            g: value,
            b: value,
            a: 255,
        });
        buffer
    }

    /// latest-wins：同一路连续生产只保留最新那帧，旧帧不排队。
    #[test]
    fn publish_keeps_only_the_latest_frame() {
        let shared = Shared::new(2);
        assert!(shared.publish(0, buffer(1)));
        assert!(!shared.publish(0, buffer(2)));
        let taken = shared.take();
        assert_eq!(taken[0].as_ref().unwrap().as_slice()[0].r, 2);
        assert!(taken[1].is_none(), "没有更新的那路不该被凭空填上帧");
    }

    /// 合并通知不能变成“永久静默”：取走之后必须重新允许通知，否则后到的帧永远上不了屏。
    #[test]
    fn take_reenables_notification() {
        let shared = Shared::new(1);
        assert!(shared.publish(0, buffer(1)));
        let _ = shared.take();
        assert!(shared.publish(0, buffer(2)), "GUI 取走后的新帧必须再次通知");
    }

    /// 投递失败（事件循环未起/已结束）后同样要恢复通知能力。
    #[test]
    fn abandoned_notification_is_retried() {
        let shared = Shared::new(1);
        assert!(shared.publish(0, buffer(1)));
        shared.abandon_notification();
        assert!(shared.publish(0, buffer(2)));
    }

    /// 去重按 `index`：重复序列不是新帧；跳号要计入 `frame_gaps`。
    #[test]
    fn cursor_dedupes_and_counts_gaps() {
        let mut cursor = FrameCursor::default();
        assert_eq!(cursor.accept(1), Some(0));
        assert_eq!(cursor.accept(1), None);
        assert_eq!(cursor.accept(4), Some(2));
        assert_eq!(cursor.accept(5), Some(0));
    }

    /// 叠加坐标必须按预览比例折算（x、y 都要）。
    #[test]
    fn overlay_is_scaled_to_preview() {
        let detection = Detection {
            status: "detected",
            tag_ids: vec![0],
            image_points: vec![[100.0, 200.0], [1280.0, 640.0]],
            object_points: Vec::new(),
            rejected: 0,
            border_bits: None,
        };
        let scaled = scale_detection(&detection, 480, 1280);
        assert_eq!(scaled.image_points, vec![[37.5, 75.0], [480.0, 240.0]]);
    }

    /// 只缩不放大：宽于预览宽才缩，窄的按原样。
    #[test]
    fn preview_size_never_upscales() {
        assert_eq!(preview_size(1280, 1088), (480, 408));
        assert_eq!(preview_size(320, 240), (320, 240));
        assert_eq!(preview_size(481, 100), (480, 100));
    }

    /// 合成的一路灰度帧：横向渐变，右端亮块（任何“像素没真的写进缓冲”都会被抓住）。
    fn tile(width: u32, height: u32) -> MonoTile {
        let mut gray = vec![0u8; (width * height) as usize];
        for y in 0..height {
            for x in 0..width {
                gray[(y * width + x) as usize] = (x * 255 / width.max(1)) as u8;
            }
        }
        MonoTile {
            gray,
            width,
            height,
            index: 1,
            pts_ns: None,
            board_timestamp_ns: None,
        }
    }

    /// 缩放 + 灰度→RGBA 必须真的落进 `SharedPixelBuffer`：OpenCV 只在尺寸**和类型**都匹配时
    /// 复用外部缓冲；类型不符会另行分配，无法写入目标画面。
    #[test]
    fn prepare_fills_rgba_buffer() {
        let mut scratch = Scratch::default();
        let buffer = prepare(&tile(1280, 1088), None, &mut scratch).expect("prepare");
        assert_eq!((buffer.width(), buffer.height()), (480, 408));
        let pixels = buffer.as_slice();
        assert!(
            pixels.iter().any(|pixel| pixel.r > 200),
            "右端亮块必须出现在预览里"
        );
        assert!(
            pixels
                .iter()
                .all(|pixel| pixel.a == 255 && pixel.r == pixel.g && pixel.b == pixel.r),
            "灰度源转 RGBA 必须是 r=g=b、a=255"
        );
    }

    /// 叠加：全分辨率角点必须按预览比例折算。四角 (100,100)-(1180,980) 在 480/1280 = 0.375 下
    /// 落在 (37.5,37.5)-(442.5,367.5)；不折算的话顶边会跑到 y=100，顶边行里就没有绿色。
    #[test]
    fn prepare_draws_overlay_at_preview_scale() {
        let detection = Detection {
            status: "detected",
            tag_ids: vec![0],
            image_points: vec![
                [100.0, 100.0],
                [1180.0, 100.0],
                [1180.0, 980.0],
                [100.0, 980.0],
            ],
            object_points: Vec::new(),
            rejected: 0,
            border_bits: None,
        };
        let mut scratch = Scratch::default();
        let buffer = prepare(&tile(1280, 1088), Some(&detection), &mut scratch).expect("prepare");
        let width = buffer.width() as usize;
        let mut top_edge = Vec::new();
        for y in 34..=41 {
            for x in 0..width {
                let pixel = buffer.as_slice()[y * width + x];
                if pixel.g > 200 && pixel.r < 150 {
                    top_edge.push(x);
                }
            }
        }
        let left = top_edge
            .iter()
            .min()
            .copied()
            .expect("缩放后的顶边应落在 y≈37 一行");
        let right = top_edge.iter().max().copied().unwrap();
        assert!(
            (33..=48).contains(&left),
            "顶边左端 x={left} 不在缩放后的位置"
        );
        assert!(
            (435..=450).contains(&right),
            "顶边右端 x={right} 不在缩放后的位置"
        );
    }
}
