//! 时钟标定探针（`--features drills`）：求「RTSP 帧时间戳 ↔ 板端 raw 时间戳」的**常数偏移**、
//! **帧内相位散布**与**漂移率**。
//!
//! 只读采样解码序号/PTS 与 raw 帧号/realtime 时间戳，复用 `estimate_clock`
//! 估计偏移、实测周期和两种散布。两窗偏移差在同一周期内折算为漂移。
//! `--verify` 通过 GET 检查命中残差；时间戳不证明整数帧对应。
//!
//! ```text
//! cargo run -p rigcal-io --features drills --bin clock-calibrate -- \
//!     --url rtsp://10.21.12.162:554/PRR --raw 10.21.12.162:30432 --camera 0 --seconds 90
//! ```

use std::time::{Duration, Instant};

use rigcal_io::clock::{
    ClockError, ClockEstimate, ClockSample, PHASE_UNCERTAINTY_LIMIT_NS, estimate_clock,
};
use rigcal_io::raw_tcp::RawTcpFrameSource;
use rigcal_io::rtsp::FrameSource;

/// 探针自己的采样下限（与 GUI 一样 8 个；估计能力下限是 `MIN_CLOCK_SAMPLES`）。
const MIN_PROBE_SAMPLES: usize = 8;

struct Sample {
    /// 两域原始读数（估计只用它，探针不改口径）。
    clock: ClockSample,
    /// 取帧往返耗时（ms）——链路耗时，与帧内残差不是同一个量。
    fetch_ms: f64,
    /// 采样时刻（主机单调钟，仅用于报告进度与分窗）。
    at: Instant,
}

fn percentile(sorted: &[i64], percent: f64) -> i64 {
    if sorted.is_empty() {
        return 0;
    }
    if sorted.len() == 1 {
        return sorted[0];
    }
    let position = (percent / 100.0) * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    if lower == upper {
        return sorted[lower];
    }
    let fraction = position - lower as f64;
    (sorted[lower] as f64 * (1.0 - fraction) + sorted[upper] as f64 * fraction).round() as i64
}

/// Compare epoch-scale offsets only after subtraction, on one shared frame grid.
/// Comparing phase_ns() from independently estimated periods amplifies nanosecond
/// period differences by the epoch's frame count and invents millisecond drift.
fn phase_drift_ns(first: ClockEstimate, last: ClockEstimate, period_ns: i64) -> i64 {
    let period = i128::from(period_ns);
    let delta = (i128::from(last.offset_ns) - i128::from(first.offset_ns)).rem_euclid(period);
    if delta > period / 2 {
        (delta - period) as i64
    } else {
        delta as i64
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut url = String::new();
    let mut raw_endpoint = String::new();
    let mut camera: i32 = 0;
    let mut seconds = 90.0_f64;
    let mut interval_ms = 100.0_f64;
    let mut verify_n = 0usize;
    let mut index = 0;
    while index < argv.len() {
        let value = |shift: usize| argv.get(index + shift).cloned().unwrap_or_default();
        match argv[index].as_str() {
            "--url" => {
                url = value(1);
                index += 2;
            }
            "--raw" => {
                raw_endpoint = value(1);
                index += 2;
            }
            "--camera" => {
                camera = value(1).parse()?;
                index += 2;
            }
            "--seconds" => {
                seconds = value(1).parse()?;
                index += 2;
            }
            "--interval-ms" => {
                interval_ms = value(1).parse()?;
                index += 2;
            }
            "--verify" => {
                verify_n = value(1).parse()?;
                index += 2;
            }
            "-h" | "--help" => {
                println!(
                    "clock-calibrate --url <rtsp://…> --raw <host:port> [--camera 0] \
                     [--seconds 90] [--interval-ms 100] [--verify N]\n\
                     只读：标定 RTSP 帧时间戳与板端 raw 时间戳之间的常数偏移、帧内相位散布与漂移率"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    if url.is_empty() || raw_endpoint.is_empty() {
        return Err("需要 --url 与 --raw".into());
    }
    let (raw_host, raw_port) = raw_endpoint.split_once(':').ok_or("--raw 需要 host:port")?;
    let raw_port: u16 = raw_port.parse()?;

    let mut source = FrameSource::start(&url, (1280, 1088), Duration::from_secs(10), false)?;
    let slot = source.slot();
    if !source.wait_first_frame(Duration::from_secs(15)) {
        return Err(format!(
            "无帧：{}",
            slot.error().unwrap_or_else(|| "等待首帧超时".to_owned())
        )
        .into());
    }

    let mut samples: Vec<Sample> = Vec::new();
    let started = Instant::now();
    let mut generation = 0u64;
    let mut last_sample = Instant::now() - Duration::from_secs(1);
    let interval = Duration::from_secs_f64((interval_ms / 1000.0).max(0.02));
    println!(
        "clock-calibrate: {} → raw://{}:{} cam{}，采样 {:.0}s（间隔 {:.0}ms）",
        url, raw_host, raw_port, camera, seconds, interval_ms
    );

    while started.elapsed().as_secs_f64() < seconds {
        if !slot.wait_changed(generation, Duration::from_millis(100)) {
            if slot.finished() {
                break;
            }
            continue;
        }
        generation = slot.generation();
        if last_sample.elapsed() < interval {
            continue;
        }
        let Some(tile) = slot.latest() else {
            continue;
        };
        let Some(pts_ns) = tile.pts_ns else {
            continue; // 该流没给时间戳：本探针无意义
        };
        last_sample = Instant::now();
        let mut raw = RawTcpFrameSource::new(raw_host, raw_port, camera, (1280, 1088), 3.0)?;
        let fetch_started = Instant::now();
        let frame = raw.fetch(None)?;
        let fetch_ms = fetch_started.elapsed().as_secs_f64() * 1000.0;
        let Some(frame) = frame else {
            continue;
        };
        let Ok(board_ns) = i64::try_from(frame.header.camera_timestamp_ns) else {
            return Err(format!(
                "板端时间戳 {} 超出 i64 纳秒域",
                frame.header.camera_timestamp_ns
            )
            .into());
        };
        // 两个域各自记全：整数帧配对跳变只能靠"序号/帧号 + 时间戳"一起才折得掉。
        samples.push(Sample {
            clock: ClockSample {
                decoder_seq: tile.index,
                decoder_ns: pts_ns,
                board_frame_id: frame.header.frame_id,
                board_ns,
            },
            fetch_ms,
            at: last_sample,
        });
    }

    if samples.len() < MIN_PROBE_SAMPLES {
        return Err(format!("有效样本只有 {} 个，无法标定", samples.len()).into());
    }

    let clock_samples: Vec<ClockSample> = samples.iter().map(|sample| sample.clock).collect();
    let estimate = estimate_clock(&clock_samples)?;
    // 与 GUI 同一个门槛、同一个函数：超限就是"标定不可用"，不是降级使用。
    let gate = estimate.check_uncertainty(PHASE_UNCERTAINTY_LIMIT_NS);

    // 漂移使用首末两窗的偏移锚点差，在整窗周期内折算。
    let window_span = (seconds * 0.25).max(5.0); // 前 25% / 后 25% 作为两个窗口
    let first: Vec<&Sample> = samples
        .iter()
        .filter(|sample| sample.at.duration_since(started).as_secs_f64() <= window_span)
        .collect();
    let last: Vec<&Sample> = samples
        .iter()
        .filter(|sample| {
            started.elapsed().as_secs_f64() - sample.at.duration_since(started).as_secs_f64()
                <= window_span
        })
        .collect();
    let window_estimate = |subset: &[&Sample]| -> Result<ClockEstimate, ClockError> {
        let clocks: Vec<ClockSample> = subset.iter().map(|sample| sample.clock).collect();
        estimate_clock(&clocks)
    };
    let drift = match (window_estimate(&first), window_estimate(&last)) {
        (Ok(first_estimate), Ok(last_estimate)) => {
            // 窗口"中点"取排序后的中位数——**不要用求和**：
            // raw 时间戳量级 ~9.5e17 ns，十来个样本相加就溢出 i64（本探针第一版就栽在这里）。
            let mid_of = |subset: &[&Sample]| -> i64 {
                let mut values: Vec<i64> =
                    subset.iter().map(|sample| sample.clock.board_ns).collect();
                values.sort_unstable();
                values[values.len() / 2]
            };
            let elapsed_ns = (mid_of(&last) - mid_of(&first)).max(1);
            // 两窗各自的周期只用于估偏移；漂移必须在整窗的同一个帧网格上比较。
            let drift_ns = phase_drift_ns(first_estimate, last_estimate, estimate.period_ns);
            let window_s = elapsed_ns as f64 / 1e9;
            Some((
                drift_ns,
                window_s,
                drift_ns as f64 / elapsed_ns as f64 * 1e6,
            ))
        }
        (first_result, last_result) => {
            let note = |result: &Result<ClockEstimate, ClockError>| match result {
                Ok(_) => "可用".to_owned(),
                Err(error) => error.to_string(),
            };
            println!();
            println!(
                "漂移              : 未测（首窗/末窗不足以估相位：{} / {}）",
                note(&first_result),
                note(&last_result)
            );
            None
        }
    };

    let fetch_median_ms = {
        let mut fetch: Vec<i64> = samples
            .iter()
            .map(|sample| (sample.fetch_ms * 1000.0) as i64)
            .collect();
        fetch.sort_unstable();
        percentile(&fetch, 50.0) as f64 / 1000.0
    };

    println!();
    println!("样本数            : {}", samples.len());
    println!(
        "帧周期            : {} ns（≈ {:.4} fps；板端长基线，两域速率已校验相容）",
        estimate.period_ns,
        1e9 / estimate.period_ns as f64
    );
    println!(
        "偏移 offset       : {} ns（≡ 相位 {} ns (mod 周期)；≤ 每一次实测差，整数帧身份不可分辨）",
        estimate.offset_ns,
        estimate.phase_ns()
    );
    println!(
        "帧内相位散布      : {} ns（{:.3} ms；门槛 {:.3} ms）——去掉整数帧支路后，偏移有多准",
        estimate.uncertainty_ns,
        estimate.uncertainty_ns as f64 / 1e6,
        PHASE_UNCERTAINTY_LIMIT_NS as f64 / 1e6
    );
    println!(
        "原始差散布        : {} ns（{:.3} ms；含整数帧配对台阶，仅诊断，不参与门槛）",
        estimate.raw_spread_ns,
        estimate.raw_spread_ns as f64 / 1e6
    );
    println!(
        "取帧往返          : 中位 {fetch_median_ms:.1} ms（链路耗时，与命中残差不是同一个量）"
    );
    if let Some((drift_ns, window_s, ppm)) = drift {
        println!(
            "漂移              : {drift_ns} ns / {window_s:.1} s = {ppm:.3} ppm（两窗偏移锚点差，折到同一周期）"
        );
    }

    let verdict = match (&gate, drift) {
        (Err(_), _) => "帧内相位散布超过 5 ms 门槛：标定不可用（门槛不放宽，也不拿旧口径充数）",
        (Ok(_), Some((_, _, ppm))) if ppm.abs() < 1.0 => {
            "当前窗漂移 < 1 ppm：仍须运行期 GET 残差自检，不能据短窗保证长期稳定"
        }
        (Ok(_), Some((_, _, ppm))) if ppm.abs() < 10.0 => {
            "当前窗漂移 1–10 ppm：每次取组校验命中残差，失效即重标定"
        }
        (Ok(_), Some(_)) => "当前窗漂移 > 10 ppm：可能脱离半帧容差，必须每次取组校验并重标定",
        (Ok(_), None) => "帧内相位散布在门槛内：可用于触发；漂移未测",
    };
    println!("结论              : {verdict}");

    let (drift_json, window_json, ppm_json) = match drift {
        Some((drift_ns, window_s, ppm)) => (
            drift_ns.to_string(),
            format!("{window_s:.1}"),
            format!("{ppm:.3}"),
        ),
        None => ("null".to_owned(), "null".to_owned(), "null".to_owned()),
    };
    println!(
        "JSON: {{\"samples\":{},\"period_ns\":{},\"offset_ns\":{},\"phase_ns\":{},\
         \"phase_uncertainty_ns\":{},\"raw_spread_ns\":{},\"fetch_median_ms\":{:.1},\
         \"drift_ns\":{},\"window_s\":{},\"ppm\":{}}}",
        samples.len(),
        estimate.period_ns,
        estimate.offset_ns,
        estimate.phase_ns(),
        estimate.uncertainty_ns,
        estimate.raw_spread_ns,
        fetch_median_ms,
        drift_json,
        window_json,
        ppm_json
    );

    if let Err(error) = gate {
        source.stop();
        return Err(error.into());
    }

    // ---- 闭环验证：用 offset 去指定时刻取帧，看板端回的是不是"那个帧网格上的组" ----
    // `hit_error` 里**不含任何链路延迟**：它是"板端给该帧的时间戳"减去"我们请求的时刻"。
    if verify_n > 0 {
        println!();
        println!("=== 闭环验证（target = pts + offset，GET 指定时刻）===");
        let mut generation = slot.generation();
        let mut offset = estimate.offset_ns;
        for round in 1..=verify_n {
            if !slot.wait_changed(generation, Duration::from_secs(3)) {
                return Err(format!("验证 #{round}：等不到新帧").into());
            }
            generation = slot.generation();
            let tile = slot.latest().ok_or("验证失败：缺少解码帧")?;
            let pts_ns = tile.pts_ns.ok_or("验证失败：解码帧没有 PTS")?;
            let Some(target) = pts_ns.checked_add(offset) else {
                return Err(
                    format!("验证 #{round}：pts={pts_ns} + offset={offset} 溢出 i64").into(),
                );
            };
            let Ok(target_request) = u64::try_from(target) else {
                return Err(format!("验证 #{round}：target={target} 为负（域不对）").into());
            };
            let mut raw = RawTcpFrameSource::new(raw_host, raw_port, camera, (1280, 1088), 3.0)?;
            let Some(frame) = raw.fetch(Some(target_request))? else {
                return Err(format!(
                    "验证 #{round}：target={target} → NO_MATCH（超出 ring 或域不对）"
                )
                .into());
            };
            let Ok(board_ns) = i64::try_from(frame.header.camera_timestamp_ns) else {
                return Err(format!("验证 #{round}：板端时间戳超出 i64 纳秒域").into());
            };
            let hit = board_ns - target;
            println!(
                "  #{round}: target={target} → 命中 camera_ts={} frame_id={}  hit_error={hit} ns（{:.3} ms）",
                frame.header.camera_timestamp_ns,
                frame.header.frame_id,
                hit as f64 / 1e6
            );
            // 闭环修正：把命中误差折进 offset，把目标推到板端实际返回的帧网格上；
            // 这不声称消掉"整数帧身份"（那本来就不可从时间戳分辨）。
            if hit.unsigned_abs() > (estimate.period_ns / 2 + 100_000) as u64 {
                return Err(format!("验证 #{round}：命中误差 {hit} ns 超过半帧容差").into());
            }
            if hit != 0 {
                offset = offset.checked_add(hit).ok_or("闭环偏移溢出 i64")?;
                println!("        （已闭环修正：offset ← {offset}）");
            }
        }
    }
    source.stop();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drift_uses_one_grid_despite_epoch_and_period_rounding() {
        let period = 33_333_333;
        let first = ClockEstimate {
            offset_ns: 1_700_000_000_000_000_000,
            uncertainty_ns: 100,
            raw_spread_ns: period,
            period_ns: period - 17,
        };
        let last = ClockEstimate {
            offset_ns: first.offset_ns + 3 * period + 900,
            period_ns: period + 22,
            ..first
        };
        assert_eq!(phase_drift_ns(first, last, period), 900);
        assert_eq!(phase_drift_ns(last, first, period), -900);
    }
}
