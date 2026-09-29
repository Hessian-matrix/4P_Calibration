//! 解码探针：检查 RTSP/容器的尺寸、序号与帧内容摘要。
//!
//! ```text
//! decode-probe --url <rtsp://…|file.mp4> --frames 5 [--expect 1280x1088]
//! ```
//! 输出每帧的尺寸/序号/摘要；`--frames` 到达即退出。

use std::process::ExitCode;
use std::time::Duration;

use rigcal_io::rtsp::{Flow, FrameSource, decode_gray_frames};

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut url = None;
    let mut frames = 3usize;
    let mut expect = (1280u32, 1088u32);
    let mut dump_all = false;
    let mut index = 0;
    while index < argv.len() {
        match argv[index].as_str() {
            "--dump-all" => {
                dump_all = true;
                index += 1;
            }
            "--url" if index + 1 < argv.len() => {
                url = Some(argv[index + 1].clone());
                index += 2;
            }
            "--frames" if index + 1 < argv.len() => {
                frames = argv[index + 1].parse().unwrap_or(3);
                index += 2;
            }
            "--expect" if index + 1 < argv.len() => {
                if let Some((w, h)) = argv[index + 1].split_once('x') {
                    expect = (w.parse().unwrap_or(1280), h.parse().unwrap_or(1088));
                }
                index += 2;
            }
            other => {
                eprintln!("decode-probe: unknown argument {other}");
                return ExitCode::from(2);
            }
        }
    }
    let Some(url) = url else {
        eprintln!("decode-probe --url <rtsp://…|file> [--frames N] [--expect WxH]");
        return ExitCode::from(2);
    };

    if dump_all {
        // 逐帧顺序对拍模式：与 ffmpeg CLI `-pix_fmt gray` 的输出逐帧比对用。
        let decoded = decode_gray_frames(&url, expect, Duration::from_secs(8), false, |tile| {
            println!(
                "{{\"n\":{},\"size\":[{},{}],\"bytes\":{},\"digest\":\"{}\"}}",
                tile.index,
                tile.width,
                tile.height,
                tile.gray.len(),
                tile.digest()
            );
            if tile.index as usize >= frames {
                Flow::Stop
            } else {
                Flow::Continue
            }
        });
        return match decoded {
            Ok(count) => {
                eprintln!("decode-probe: {count} frame(s) decoded in order");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("decode-probe: {error}");
                ExitCode::from(1)
            }
        };
    }

    let mut source = match FrameSource::start(&url, expect, Duration::from_secs(8), false) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("decode-probe: {error}");
            return ExitCode::from(1);
        }
    };
    let slot = source.slot();
    if !source.wait_first_frame(Duration::from_secs(10)) {
        let reason = slot
            .error()
            .unwrap_or_else(|| "timeout waiting for the first frame".to_owned());
        eprintln!("decode-probe: {reason}");
        source.stop();
        return ExitCode::from(1);
    }
    println!("endpoint {} expect {}x{}", source.url(), expect.0, expect.1);

    let mut seen = 0usize;
    let mut generation = 0u64;
    while seen < frames {
        if let Some(frame) = slot.latest() {
            if frame.index as usize > seen {
                seen = frame.index as usize;
                let digest = frame.digest();
                println!(
                    "{{\"n\":{seen},\"size\":[{},{}],\"bytes\":{},\"digest\":\"{digest}\"}}",
                    frame.width,
                    frame.height,
                    frame.gray.len()
                );
            }
        }
        let changed = slot.wait_changed(generation, Duration::from_secs(5));
        generation = slot.generation();
        if !changed && slot.finished() {
            break;
        }
    }
    source.stop();
    if seen == 0 {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}
