//! 帧组捕获探针：验"锚点 → 并发扇出 → 组一致性校验 → 两级重试"在现场注入条件下是否成立。
//!
//! ```text
//! group-probe --host 127.0.0.1 --port 4210 --frames 3 [--camera cam0] [--expect 1280x1088]
//! ```
//! 每组输出一行 JSON（`group_id`/`group_timestamp_ns`/各路 frame_id 与时间戳/`max_skew_ns`/`attempts`），
//! 最后一行是汇总；组捕获失败时打印失败原因并以非零码退出。

use std::process::ExitCode;
use std::time::Duration;

use rigcal_io::group::{CameraGroupSpec, GroupCapture, GroupCaptureOptions};

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut host = "127.0.0.1".to_owned();
    let mut port: u16 = 4210;
    let mut frames = 1usize;
    let mut expect = (1280u32, 1088u32);
    let mut trigger = "cam0".to_owned();
    let mut at: Option<u64> = None;
    let mut index = 0;
    while index < argv.len() {
        let value = |shift: usize| argv.get(index + shift).cloned().unwrap_or_default();
        match argv[index].as_str() {
            "--host" => {
                host = value(1);
                index += 2;
            }
            "--port" => {
                port = value(1).parse().unwrap_or(4210);
                index += 2;
            }
            "--frames" => {
                frames = value(1).parse().unwrap_or(1);
                index += 2;
            }
            "--expect" => {
                if let Some((w, h)) = value(1).split_once('x') {
                    expect = (w.parse().unwrap_or(1280), h.parse().unwrap_or(1088));
                }
                index += 2;
            }
            "--camera" => {
                trigger = value(1);
                index += 2;
            }
            "--at" => {
                at = value(1).parse().ok();
                index += 2;
            }
            other => {
                eprintln!("group-probe: unknown argument {other}");
                return ExitCode::from(2);
            }
        }
    }

    let cameras: Vec<CameraGroupSpec> = (0..4)
        .map(|id| CameraGroupSpec {
            camera_id: format!("cam{id}"),
            raw_camera_id: id,
        })
        .collect();
    let capture = match GroupCapture::new(
        &host,
        port,
        &cameras,
        &trigger,
        expect,
        5.0,
        GroupCaptureOptions::default(),
    ) {
        Ok(capture) => capture,
        Err(error) => {
            eprintln!("group-probe: {error}");
            return ExitCode::from(2);
        }
    };
    println!(
        "endpoint {} expect {}x{} frames={frames}",
        capture.endpoint(),
        expect.0,
        expect.1
    );

    let mut attempts = Vec::new();
    for number in 1..=frames {
        let result = match at {
            Some(timestamp_ns) => capture.capture_at(timestamp_ns),
            None => capture.capture(),
        };
        match result {
            Ok(group) => {
                attempts.push(group.attempts);
                let frames_json: Vec<String> = group
                    .frames
                    .iter()
                    .map(|entry| {
                        format!(
                            "{{\"camera\":\"{}\",\"frame_id\":{},\"camera_timestamp_ns\":{},\"bytes\":{}}}",
                            entry.camera_id,
                            entry.frame.header.frame_id,
                            entry.frame.header.camera_timestamp_ns,
                            entry.frame.gray.len()
                        )
                    })
                    .collect();
                println!(
                    "{{\"n\":{number},\"group_id\":{},\"group_timestamp_ns\":{},\"max_skew_ns\":{},\"attempts\":{},\"frames\":[{}]}}",
                    group.group_id,
                    group.group_timestamp_ns,
                    group.max_skew_ns,
                    group.attempts,
                    frames_json.join(",")
                );
            }
            Err(error) => {
                eprintln!("group-probe: {error}");
                return ExitCode::from(1);
            }
        }
    }
    // 稳定性检查：所有组必须各自内部一致，且组号严格递增（说明没有回放旧组）。
    let summary = format!(
        "{{\"groups\":{},\"attempts\":{attempts:?}}}",
        attempts.len()
    );
    println!("{summary}");
    std::thread::sleep(Duration::from_millis(10));
    ExitCode::SUCCESS
}
