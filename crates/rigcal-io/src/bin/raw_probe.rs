//! 板端 raw 服务连通性与响应诊断探针。
//!
//! ```text
//! raw-probe --host 10.21.12.162 --port 30432 --camera 0 --frames 3 [--expect 1280x1088]
//! raw-probe --host 10.21.12.162 --port 30432 --camera 0 --get <timestamp_ns>
//! ```
//!
//! 每个响应输出一行 JSON，包含时间戳、尺寸和图像摘要。

use std::process::ExitCode;

use rigcal_io::{RawFrameStatus, RawTcpFrameSource};

fn parse_u32(text: &str, flag: &str) -> Result<u32, String> {
    text.parse().map_err(|error| format!("{flag}: {error}"))
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut host = None;
    let mut port = None;
    let mut camera = 0i32;
    let mut frames = 1usize;
    let mut get = None;
    let mut expect = (1280u32, 1088u32);
    let mut index = 0;
    while index < argv.len() {
        let value = |index: usize| -> Result<String, String> {
            argv.get(index + 1)
                .cloned()
                .ok_or_else(|| format!("{} needs a value", argv[index]))
        };
        let parsed = match argv[index].as_str() {
            "--host" => value(index).map(|v| host = Some(v)),
            "--port" => value(index).and_then(|v| {
                v.parse::<u16>()
                    .map(|p| port = Some(p))
                    .map_err(|e| e.to_string())
            }),
            "--camera" => value(index).and_then(|v| {
                v.parse::<i32>()
                    .map(|c| camera = c)
                    .map_err(|e| e.to_string())
            }),
            "--frames" => value(index).and_then(|v| {
                v.parse::<usize>()
                    .map(|f| frames = f)
                    .map_err(|e| e.to_string())
            }),
            "--get" => value(index).and_then(|v| {
                v.parse::<u64>()
                    .map(|t| get = Some(t))
                    .map_err(|e| e.to_string())
            }),
            "--expect" => value(index).and_then(|v| {
                let (w, h) = v
                    .split_once('x')
                    .ok_or_else(|| "--expect wants WxH".to_string())?;
                expect = (
                    parse_u32(w, "--expect width")?,
                    parse_u32(h, "--expect height")?,
                );
                Ok(())
            }),
            "-h" | "--help" => {
                println!(
                    "raw-probe --host H --port P [--camera N] [--frames N] [--get TS] [--expect WxH]"
                );
                return ExitCode::SUCCESS;
            }
            other => Err(format!("unknown argument {other}")),
        };
        if let Err(message) = parsed {
            eprintln!("{message}");
            return ExitCode::from(2);
        }
        index += if argv[index].starts_with("--") { 2 } else { 1 };
    }
    let (Some(host), Some(port)) = (host, port) else {
        eprintln!("--host and --port are required");
        return ExitCode::from(2);
    };
    let mut source = match RawTcpFrameSource::new(&host, port, camera, expect, 5.0) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("rigcal raw-probe: {error}");
            return ExitCode::from(2);
        }
    };
    println!(
        "endpoint {} expect {}x{}",
        source.source_label(),
        expect.0,
        expect.1
    );
    let mut delivered = 0usize;
    let mut attempts = 0usize;
    while delivered < frames && attempts < frames * 20 + 20 {
        attempts += 1;
        match source.fetch(get) {
            Ok(Some(frame)) => {
                delivered += 1;
                println!(
                    "{{\"n\":{delivered},\"status\":\"OK\",\"frame_id\":{},\"group_id\":{},\"group_timestamp_ns\":{},\
                     \"camera_timestamp_ns\":{},\"size\":[{},{}],\"payload_digest\":\"{}\"}}",
                    frame.header.frame_id,
                    frame.header.group_id,
                    frame.header.group_timestamp_ns,
                    frame.header.camera_timestamp_ns,
                    frame.header.width,
                    frame.header.height,
                    frame.gray_digest()
                );
            }
            Ok(None) => {
                println!("{{\"n\":{delivered},\"status\":\"NO_MATCH\"}}");
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            Err(error) => {
                println!("{{\"n\":{delivered},\"status\":\"ERROR\",\"error\":\"{error}\"}}");
                return ExitCode::from(1);
            }
        }
        if get.is_some() {
            break; // 单次按时间戳寻址
        }
    }
    let _ = RawFrameStatus::Ok;
    if delivered == 0 {
        eprintln!("rigcal raw-probe: no frame delivered in {attempts} attempt(s)");
        return ExitCode::from(1);
    }
    ExitCode::SUCCESS
}
