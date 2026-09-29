//! 演练用板端 raw 服务 mock（`--features drills`）：按 RWF1 协议为 4 路返回**同一帧组**。
//!
//! 配合引导视频提供无设备采集演练：
//! - **同一组的四路共享 `group_id` / `group_timestamp_ns`**，`camera_timestamp_ns` 可按通道注入偏斜；
//! - 无 `--fps` 时 `LATEST <cam>` 每次前进一组（快速协议演练）；
//! - `--fps N` 时按单调时钟产帧，每个 PGM 姿态保持一秒，支持引导视频的时钟采样与 GET 回查；
//! - `GET <cam> <ts>` 只在最近若干组里回查，查不到回 `NO_MATCH`，不重新取锚；
//! - 载荷是紧凑 NV12（Y 平面 + UV），Y 来自 `--frames` 目录里的 P5 PGM（循环播放）。
//!
//! ```text
//! cargo run -p rigcal-io --features drills --bin mock-raw-server -- \
//!     --frames /tmp/drill_frames --port 4211 --fps 60
//! ```

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use rigcal_io::raw_tcp::{RAW_FRAME_HEADER_SIZE, RAW_FRAME_MAGIC, RAW_FRAME_VERSION};

const STATUS_OK: i32 = 0;
const STATUS_NO_MATCH: i32 = 1;
const STATUS_BAD_REQUEST: i32 = 2;

/// 一个帧组在 mock 里的表示：四路共享 `group_id`，逐路可有自己的 `camera_timestamp_ns`。
#[derive(Clone, Debug)]
struct Group {
    group_id: u64,
    group_timestamp_ns: u64,
    frame_index: usize,
}

struct Mock {
    frames: Vec<Vec<u8>>,
    width: u32,
    height: u32,
    cameras: u32,
    /// 每路相对组时间戳的偏斜（用于演练"组内不同刻"的判定）。
    skew_ns: Vec<u64>,
    current: u64,
    ring: VecDeque<Group>,
    ring_capacity: usize,
    fps: Option<u32>,
    started: Option<Instant>,
}

impl Mock {
    fn new(frames: Vec<Vec<u8>>, width: u32, height: u32, cameras: u32, skew_ns: Vec<u64>) -> Self {
        Self {
            frames,
            width,
            height,
            cameras,
            skew_ns,
            current: 0,
            ring: VecDeque::new(),
            ring_capacity: 64,
            fps: None,
            started: None,
        }
    }

    fn group_at(&self, group_id: u64) -> Group {
        Group {
            group_id,
            group_timestamp_ns: 1_700_000_000_000_000_000
                + group_id * 1_000_000_000 / u64::from(self.fps.unwrap_or(30)),
            frame_index: ((group_id / u64::from(self.fps.unwrap_or(1))) as usize)
                % self.frames.len(),
        }
    }

    fn remember(&mut self, group: Group) {
        self.ring.push_back(group);
        while self.ring.len() > self.ring_capacity {
            self.ring.pop_front();
        }
    }

    fn advance_clock(&mut self) {
        let Some(fps) = self.fps else {
            return;
        };
        // Start playback on first use, not at an unrelated server-launch phase. The first
        // request lands in the middle of frame zero; later frames still follow this fixed
        // monotonic clock, never the request count. This is a replay fixture, not a clock-accuracy test.
        let started = self.started.get_or_insert_with(|| {
            Instant::now() - Duration::from_nanos(500_000_000 / u64::from(fps))
        });
        self.current = (started.elapsed().as_nanos() * u128::from(fps) / 1_000_000_000) as u64;
        let first = self
            .ring
            .back()
            .map_or(0, |group| group.group_id + 1)
            .max(self.current.saturating_sub(self.ring_capacity as u64 - 1));
        for group_id in first..=self.current {
            self.remember(self.group_at(group_id));
        }
    }

    /// 在 ring 内按半帧容差寻找最近邻；超出容差返回 None。
    fn find(&self, timestamp_ns: u64) -> Option<&Group> {
        let tolerance_ns = 1_000_000_000 / u64::from(self.fps.unwrap_or(30)) / 2 + 1;
        self.ring
            .iter()
            .filter_map(|group| {
                let delta = group.group_timestamp_ns.abs_diff(timestamp_ns);
                (delta <= tolerance_ns).then_some((delta, group))
            })
            .min_by_key(|(delta, _)| *delta)
            .map(|(_, group)| group)
    }

    fn payload(&self, group: &Group) -> Vec<u8> {
        let y = &self.frames[group.frame_index];
        let mut payload = Vec::with_capacity(y.len() + (self.width * self.height / 2) as usize);
        payload.extend_from_slice(y);
        payload.extend(std::iter::repeat_n(
            128u8,
            (self.width * self.height / 2) as usize,
        ));
        payload
    }

    fn header(
        &self,
        status: i32,
        camera: i32,
        group: Option<&Group>,
        payload_size: u64,
    ) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(RAW_FRAME_HEADER_SIZE);
        let push_u32 =
            |bytes: &mut Vec<u8>, value: u32| bytes.extend_from_slice(&value.to_le_bytes());
        let push_i32 =
            |bytes: &mut Vec<u8>, value: i32| bytes.extend_from_slice(&value.to_le_bytes());
        let push_u64 =
            |bytes: &mut Vec<u8>, value: u64| bytes.extend_from_slice(&value.to_le_bytes());
        push_u32(&mut bytes, RAW_FRAME_MAGIC);
        push_u32(&mut bytes, RAW_FRAME_VERSION);
        push_i32(&mut bytes, status);
        push_i32(&mut bytes, camera);
        match group {
            Some(group) => {
                let camera_timestamp_ns = group.group_timestamp_ns
                    + self.skew_ns.get(camera as usize).copied().unwrap_or(0);
                push_u64(&mut bytes, group.group_timestamp_ns);
                push_u64(&mut bytes, camera_timestamp_ns);
                push_u64(&mut bytes, group.group_id);
                push_u64(&mut bytes, group.group_id);
            }
            None => {
                push_u64(&mut bytes, 0);
                push_u64(&mut bytes, 0);
                push_u64(&mut bytes, 0);
                push_u64(&mut bytes, 0);
            }
        }
        push_u32(&mut bytes, self.width);
        push_u32(&mut bytes, self.height);
        push_u32(&mut bytes, self.width);
        push_u32(&mut bytes, self.height / 2);
        push_u64(&mut bytes, payload_size);
        for _ in 0..3 {
            push_u64(&mut bytes, 0);
        }
        bytes
    }

    fn respond(&mut self, line: &str) -> (Vec<u8>, Vec<u8>) {
        self.advance_clock();
        let parts: Vec<&str> = line.split_whitespace().collect();
        let camera = parts
            .get(1)
            .and_then(|value| value.parse::<u32>().ok())
            .filter(|value| *value < self.cameras);
        let Some(camera) = camera else {
            return (self.header(STATUS_BAD_REQUEST, -1, None, 0), Vec::new());
        };
        match parts.first().copied() {
            Some("LATEST") => {
                let group = self.group_at(self.current);
                if self.fps.is_none() {
                    self.current += 1;
                    self.remember(group.clone());
                }
                let payload = self.payload(&group);
                let header =
                    self.header(STATUS_OK, camera as i32, Some(&group), payload.len() as u64);
                (header, payload)
            }
            Some("GET") => {
                let timestamp = parts.get(2).and_then(|value| value.parse::<u64>().ok());
                match timestamp.and_then(|timestamp| self.find(timestamp).cloned()) {
                    Some(group) => {
                        let payload = self.payload(&group);
                        let header = self.header(
                            STATUS_OK,
                            camera as i32,
                            Some(&group),
                            payload.len() as u64,
                        );
                        (header, payload)
                    }
                    None => (
                        self.header(STATUS_NO_MATCH, camera as i32, None, 0),
                        Vec::new(),
                    ),
                }
            }
            _ => (
                self.header(STATUS_BAD_REQUEST, camera as i32, None, 0),
                Vec::new(),
            ),
        }
    }
}

/// 读 P5 PGM（灰度）。渲染器写的就是它，解析只需 3 行头。
fn read_pgm(path: &Path) -> std::io::Result<(Vec<u8>, u32, u32)> {
    let bytes = std::fs::read(path)?;
    let mut fields = Vec::new();
    let mut offset = 0usize;
    while fields.len() < 4 && offset < bytes.len() {
        // 跳过空白与注释
        while offset < bytes.len() && bytes[offset].is_ascii_whitespace() {
            offset += 1;
        }
        if offset < bytes.len() && bytes[offset] == b'#' {
            while offset < bytes.len() && bytes[offset] != b'\n' {
                offset += 1;
            }
            continue;
        }
        let start = offset;
        while offset < bytes.len() && !bytes[offset].is_ascii_whitespace() {
            offset += 1;
        }
        fields.push(String::from_utf8_lossy(&bytes[start..offset]).to_string());
    }
    offset += 1; // 头部的单个换行
    let width: u32 = fields
        .get(1)
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let height: u32 = fields
        .get(2)
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let pixels = bytes[offset..].to_vec();
    Ok((pixels, width, height))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut frames_dir = PathBuf::from("/tmp/board_frames");
    let mut port = 4211u16;
    let mut cameras = 4u32;
    let mut skew_ms: u64 = 0;
    let mut fps = None;
    let mut index = 0;
    while index < argv.len() {
        let value = |shift: usize| argv.get(index + shift).cloned().unwrap_or_default();
        match argv[index].as_str() {
            "--frames" => {
                frames_dir = PathBuf::from(value(1));
                index += 2;
            }
            "--port" => {
                port = value(1).parse()?;
                index += 2;
            }
            "--cameras" => {
                cameras = value(1).parse()?;
                index += 2;
            }
            "--skew-ms" => {
                skew_ms = value(1).parse()?;
                index += 2;
            }
            "--fps" => {
                let value = value(1).parse::<u32>()?;
                if !(1..=240).contains(&value) {
                    return Err("--fps must be 1..=240".into());
                }
                fps = Some(value);
                index += 2;
            }
            "-h" | "--help" => {
                println!(
                    "mock-raw-server --frames <pgm dir> [--port 4211] [--cameras 4] [--skew-ms 0] [--fps N]\n\
                     默认 LATEST 前进一组；--fps 按时钟产帧、每个 PGM 姿态保持一秒（GUI 演练）"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&frames_dir)?
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .filter(|path| path.extension().and_then(|value| value.to_str()) == Some("pgm"))
        .collect();
    paths.sort();
    if paths.is_empty() {
        return Err(format!(
            "{} 里没有 .pgm 帧（用 ffmpeg 将可检测的板图转成 P5 灰度帧）",
            frames_dir.display()
        )
        .into());
    }
    let mut frames = Vec::with_capacity(paths.len());
    let mut size = (0u32, 0u32);
    for path in &paths {
        let (pixels, width, height) = read_pgm(path)?;
        size = (width, height);
        frames.push(pixels);
    }
    let skew_ns: Vec<u64> = (0..cameras)
        .map(|camera| if camera == 0 { 0 } else { skew_ms * 1_000_000 })
        .collect();
    let mut mock = Mock::new(frames, size.0, size.1, cameras, skew_ns);
    mock.fps = fps;
    let mock = std::sync::Arc::new(std::sync::Mutex::new(mock));
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    println!(
        "mock-raw-server: {} 帧 {}x{}，{} 路，监听 127.0.0.1:{port}{}",
        paths.len(),
        size.0,
        size.1,
        cameras,
        fps.map(|value| format!("（时钟推进 {value} Hz）"))
            .unwrap_or_else(|| "（LATEST 请求推进）".to_owned())
    );
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        let mock = std::sync::Arc::clone(&mock);
        std::thread::spawn(move || {
            let _ = serve(stream, mock);
        });
    }
    Ok(())
}

fn serve(stream: TcpStream, mock: std::sync::Arc<std::sync::Mutex<Mock>>) -> std::io::Result<()> {
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut line = String::new();
    if reader.read_line(&mut line)? == 0 {
        return Ok(());
    }
    let (header, payload) = {
        let mut mock = mock.lock().expect("mock");
        mock.respond(line.trim_end())
    };
    let mut stream = stream;
    stream.write_all(&header)?;
    if !payload.is_empty() {
        stream.write_all(&payload)?;
    }
    stream.flush()
}
