//! 解码 PTS 到板端帧网格的时钟对齐。
//!
//! raw 时间戳属于板端 realtime 域，解码 PTS 属于连接会话域。两域同速时，
//! `板端目标时刻 = pts + offset` 可用于 GET 最近邻取组。
//!
//! `raw_ns − pts_ns` 同时包含亚帧抖动和整数帧配对跳变。估计器先用两域各自的
//! 序号/时间戳步长估周期并检查同速，再对差值取模估帧内相位。帧内散布决定可用性，
//! 原始散布仅用于诊断；周期来自采样，不假定 30/60 fps。
//!
//! 偏移锚在不大于最小观测差的相位一致支路。时间戳无法区分 `offset` 与 `offset + k×T`，
//! 因而不证明引导图像与 raw 来自同一次曝光；四路组一致性由设备组身份合同保证。
//!
//! 纳秒差值先用整数计算，再在单周期内转浮点求相位，避免纪元量级的精度损失。

use std::time::{Duration, Instant};

use crate::raw_tcp::{RawFrameError, RawTcpFrameSource};
use crate::rtsp::FrameSlot;

/// 探针与 GUI 共用的**帧内相位散布**门槛（ns）：超过即标定失败——门槛不放宽。
pub const PHASE_UNCERTAINTY_LIMIT_NS: i64 = 5_000_000;

/// 估周期/相位至少需要的样本数（= 3 个相邻对）。
pub const MIN_CLOCK_SAMPLES: usize = 4;

/// 每轮标定的最少有效样本数（采样策略，不是估计能力下限）。
const MIN_CALIBRATION_SAMPLES: usize = 8;

/// 单域周期候选相对该域中位数的 MAD 上限（百分比）：节奏不匀 ⇒ 没有可靠周期。
const PERIOD_MAD_LIMIT_PERCENT: i64 = 10;

/// 两域周期/网格周期的相对差上限（百分比）：超过即**不同速**，常数偏移模型不成立。
///
/// 粗差守卫用于拒绝不同帧率或单位；较小的速率偏差由相位散布门槛检查。
const RATE_MISMATCH_LIMIT_PERCENT: i64 = 10;

#[derive(Debug, thiserror::Error)]
pub enum ClockError {
    #[error("raw: {0}")]
    Raw(#[from] RawFrameError),
    #[error("引导源在 {0:?} 内没有带时间戳的新帧（pts 不可用）")]
    NoTimestampedFrame(Duration),
    #[error("有效样本只有 {0} 个，不足以标定")]
    TooFewSamples(usize),
    #[error("标定不确定度 {0} ns 过大（>{1} ns）：链路抖动异常")]
    TooNoisy(i64, i64),
    #[error("样本 #{index} 的{field}没有前进（两域序号/时间戳必须严格单调）")]
    NonMonotonic { index: usize, field: &'static str },
    #[error("帧周期不可靠（{0} 个相邻对）：两域步长不匀或相位散布覆盖半个周期以上")]
    UnreliablePeriod(usize),
    #[error("两域速率不相容：解码域 {0} ns/帧，板端域 {1} ns/帧")]
    RateMismatch(i64, i64),
    #[error("时间戳量级超出 i64 纳秒表示：{0}")]
    TimestampOutOfRange(i128),
    #[error("时钟标定已取消")]
    Cancelled,
}

/// 一帧时间戳的**域**——两个域不能混用（混用是这类 bug 的常见来源）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimestampDomain {
    /// 解码器域（RTSP 路径）。
    Decoder,
    /// 板端 realtime 域（raw 服务路径）。
    Board,
}

/// 一次采样的两域原始读数（**不做任何换算**：域不同，混用是 bug 之源）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockSample {
    /// 解码域：该路累计发布序号（单调）。
    pub decoder_seq: u64,
    /// 解码域：帧时间戳（ns）。
    pub decoder_ns: i64,
    /// 板端域：该相机的帧号（单调）。
    pub board_frame_id: u64,
    /// 板端域：帧时间戳（ns）。
    pub board_ns: i64,
}

/// 标定结果：绝对偏移，以及**两种必须分开报的散布**。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockEstimate {
    /// 板端网格上的常数偏移：`offset ≤ 每一次实测差`，且 `offset ≡ 相位 (mod period)`。
    ///
    /// 它与真实常数最多差整数个帧周期（时间戳不可识别），但**不会晚于**任何一次实测。
    pub offset_ns: i64,
    /// **帧内相位散布**（p95 − 最紧）：去掉整数帧支路后残留的抖动——决定偏移有多准。
    pub uncertainty_ns: i64,
    /// **原始差散布**（`raw − pts` 的 p95 − min，含整数帧台阶）：诊断用，不参与门槛。
    pub raw_spread_ns: i64,
    /// 两域共同估出的帧周期（板端长基线）。
    pub period_ns: i64,
}

impl ClockEstimate {
    /// 帧网格相位：`offset ≡ phase (mod period)`。
    pub fn phase_ns(&self) -> i64 {
        self.offset_ns.rem_euclid(self.period_ns)
    }

    /// 用门槛校验帧内相位散布；超限即报错（**不**返回降级结果、不放宽门槛）。
    pub fn check_uncertainty(&self, limit_ns: i64) -> Result<(), ClockError> {
        if self.uncertainty_ns > limit_ns {
            return Err(ClockError::TooNoisy(self.uncertainty_ns, limit_ns));
        }
        Ok(())
    }
}

/// 由采样估偏移、帧周期与两种散布（纯函数，便于单测）。
///
/// 步骤：
/// 1. 相邻样本的**序号/时间戳步长**必须严格前进，否则 `NonMonotonic`；
/// 2. 每域各自用步长求周期候选，中位数即该域周期；MAD 过大 ⇒ `UnreliablePeriod`；
/// 3. 两域周期必须相容（同速异域的前提），否则 `RateMismatch`；
/// 4. 帧网格周期取**板端长基线**（帧号↔时间戳这一对定义了 `GET` 的匹配网格，长基线把单帧抖动平掉）；
/// 5. 每个 `S` 折到 `[0, T)`，相位散布就是 `uncertainty_ns`；
/// 6. `offset` 锚在最小原始差的支路、取最紧的一端。
pub fn estimate_clock(samples: &[ClockSample]) -> Result<ClockEstimate, ClockError> {
    if samples.len() < MIN_CLOCK_SAMPLES {
        return Err(ClockError::TooFewSamples(samples.len()));
    }

    // 大整数先减锚点在后面做；这里只做 i128 减法，避免纪元级溢出/量化。
    let differences: Vec<i128> = samples
        .iter()
        .map(|sample| i128::from(sample.board_ns) - i128::from(sample.decoder_ns))
        .collect();

    let mut decoder_periods: Vec<i64> = Vec::with_capacity(samples.len() - 1);
    let mut board_periods: Vec<i64> = Vec::with_capacity(samples.len() - 1);
    for (pair_index, pair) in samples.windows(2).enumerate() {
        let index = pair_index + 1;
        let decoder_steps = step(
            i128::from(pair[1].decoder_seq),
            i128::from(pair[0].decoder_seq),
            index,
            "解码序号",
        )?;
        let grid_frames = step(
            i128::from(pair[1].board_frame_id),
            i128::from(pair[0].board_frame_id),
            index,
            "板端帧号",
        )?;
        let decoder_span = step(
            i128::from(pair[1].decoder_ns),
            i128::from(pair[0].decoder_ns),
            index,
            "解码时间戳",
        )?;
        let board_span = step(
            i128::from(pair[1].board_ns),
            i128::from(pair[0].board_ns),
            index,
            "板端时间戳",
        )?;
        // 周期 < 1 ns 的域没有意义（时间戳比序号走得还慢）。
        if decoder_span < decoder_steps || board_span < grid_frames {
            return Err(ClockError::UnreliablePeriod(samples.len() - 1));
        }
        decoder_periods.push(decoder_span / decoder_steps);
        board_periods.push(board_span / grid_frames);
    }

    let pairs = decoder_periods.len();
    let decoder_period = median(&mut decoder_periods);
    let board_period = median(&mut board_periods);
    if !cadence_is_uniform(&mut decoder_periods, decoder_period)
        || !cadence_is_uniform(&mut board_periods, board_period)
    {
        return Err(ClockError::UnreliablePeriod(pairs));
    }
    if rate_mismatch(decoder_period, board_period) {
        return Err(ClockError::RateMismatch(decoder_period, board_period));
    }

    // 整窗长基线降低周期估计中的量化与单帧抖动误差。
    let first = samples.first().expect("样本数已保证非空");
    let last = samples.last().expect("样本数已保证非空");
    let board_frames = i128::from(last.board_frame_id) - i128::from(first.board_frame_id);
    let board_span = i128::from(last.board_ns) - i128::from(first.board_ns);
    if board_frames <= 0 || board_span <= 0 {
        return Err(ClockError::UnreliablePeriod(pairs));
    }
    let period_ns = narrow(board_span / board_frames)?;
    if rate_mismatch(period_ns, board_period) {
        return Err(ClockError::UnreliablePeriod(pairs));
    }

    let period = i128::from(period_ns);
    let phases: Vec<i64> = differences
        .iter()
        .map(|difference| (difference.rem_euclid(period)) as i64)
        .collect();

    // 相位是小量（< T），浮点只在这一步出现；圆均值用来定"哪一端最紧"，
    // 锚点本身仍取某个样本的整数相位，不受浮点误差影响。
    let mean_phase = circular_mean(&phases, period_ns);
    // 最紧的一端 = 相对圆均值的**带符号**偏差最小者（相位簇的下沿）。
    // 必须带符号：直接用 `(p − mean) mod T` 取极值会被相位表示在 0 处的折返骗到簇中间，
    // 于是把整段相位差算成 ~T 的假散布。
    let phase_anchor = *phases
        .iter()
        .min_by_key(|phase| signed_delta(**phase - mean_phase, period_ns))
        .expect("样本数已保证非空");

    // 帧内散布为相位簇相对下沿的 p95，排除整数帧台阶。
    let mut spread: Vec<i64> = phases
        .iter()
        .map(|phase| (phase - phase_anchor).rem_euclid(period_ns))
        .collect();
    let uncertainty_ns = percentile95(&mut spread);
    // 相位簇覆盖半个周期以上时，"最紧的一端"不再可识别：拒绝，不猜。
    if i128::from(uncertainty_ns) * 2 >= period {
        return Err(ClockError::UnreliablePeriod(pairs));
    }

    // 保守锚定：≤ 最小原始差的最大"相位一致"候选（时延只会把 S 撑大）。
    let minimum = *differences.iter().min().expect("样本数已保证非空");
    let offset =
        i128::from(phase_anchor) + (minimum - i128::from(phase_anchor)).div_euclid(period) * period;
    let offset_ns = narrow(offset)?;

    // 原始差散布：先减最小差（锚点）再排序取 p95，全程整数。
    let mut shifted: Vec<i64> = Vec::with_capacity(differences.len());
    for difference in &differences {
        shifted.push(narrow(difference - minimum)?);
    }
    let raw_spread_ns = percentile95(&mut shifted);

    Ok(ClockEstimate {
        offset_ns,
        uncertainty_ns,
        raw_spread_ns,
        period_ns,
    })
}

/// 每连接一次的时钟对齐。
#[derive(Clone, Debug)]
pub struct ClockAligner {
    offset_ns: i64,
    uncertainty_ns: i64,
    raw_spread_ns: i64,
    frame_period_ns: i64,
    samples: usize,
    corrections: usize,
    calibrated_at: Instant,
}

impl ClockAligner {
    /// 采样标定：反复做「等一个新解码帧 → 立刻取一次 raw LATEST」。
    ///
    /// `samples` 建议 50–200（每次约 `interval`，总耗时 = samples × interval）。
    /// `stop` 可中止采样；单次 raw 等待仍受客户端超时约束。
    ///
    /// 每个样本同时记下两域的序号/时间戳，交给 `estimate_clock`——**解码帧与 LATEST 帧
    /// 差整数个帧周期不再算作抖动**，它被折进帧网格；只有帧内相位散布能顶破门槛。
    pub fn calibrate(
        slot: &FrameSlot,
        raw: &mut RawTcpFrameSource,
        samples: usize,
        interval: Duration,
        max_uncertainty_ns: i64,
        stop: &std::sync::atomic::AtomicBool,
    ) -> Result<Self, ClockError> {
        let mut observations: Vec<ClockSample> = Vec::with_capacity(samples);
        let mut generation = slot.generation();
        let mut last = Instant::now() - interval;
        let deadline = Instant::now() + interval * (samples as u32 * 6 + 20);
        while observations.len() < samples && Instant::now() < deadline {
            if stop.load(std::sync::atomic::Ordering::Acquire) {
                return Err(ClockError::Cancelled);
            }
            if !slot.wait_changed(generation, Duration::from_millis(200)) {
                if slot.finished() {
                    break;
                }
                continue;
            }
            generation = slot.generation();
            if last.elapsed() < interval {
                continue;
            }
            let Some(tile) = slot.latest() else {
                continue;
            };
            let Some(pts_ns) = tile.pts_ns else {
                continue; // 该流没有解码时间戳：对齐能力不可用
            };
            last = Instant::now();
            let Some(frame) = raw.fetch(None)? else {
                continue;
            };
            let camera_timestamp_ns = frame.header.camera_timestamp_ns;
            let board_ns = i64::try_from(camera_timestamp_ns)
                .map_err(|_| ClockError::TimestampOutOfRange(i128::from(camera_timestamp_ns)))?;
            observations.push(ClockSample {
                decoder_seq: tile.index,
                decoder_ns: pts_ns,
                board_frame_id: frame.header.frame_id,
                board_ns,
            });
        }
        if stop.load(std::sync::atomic::Ordering::Acquire) {
            return Err(ClockError::Cancelled);
        }
        if observations.len() < MIN_CALIBRATION_SAMPLES {
            return Err(ClockError::TooFewSamples(observations.len()));
        }
        let estimate = estimate_clock(&observations)?;
        estimate.check_uncertainty(max_uncertainty_ns)?;
        Ok(Self {
            offset_ns: estimate.offset_ns,
            uncertainty_ns: estimate.uncertainty_ns,
            raw_spread_ns: estimate.raw_spread_ns,
            frame_period_ns: estimate.period_ns,
            samples: observations.len(),
            corrections: 0,
            calibrated_at: Instant::now(),
        })
    }

    /// 解码帧时间戳 → 板端时间戳（这就是要发给板端 `GET` 的值）。
    pub fn to_board_ns(&self, pts_ns: i64) -> i64 {
        pts_ns + self.offset_ns
    }

    /// 反向换算（诊断/日志用）。
    pub fn to_decoder_ns(&self, board_ns: i64) -> i64 {
        board_ns - self.offset_ns
    }

    /// 运行期自检量：`raw 锚点时间戳 − (触发帧 pts 换算值)`。
    ///
    /// GET 最近邻匹配允许正负残差，调用方按本路实际帧周期的半帧容差检查。
    /// 时间戳无法识别整数帧偏差；超带拒绝本次采集并重新标定。
    pub fn residual_ns(&self, board_ns: i64, pts_ns: i64) -> i64 {
        board_ns - self.to_board_ns(pts_ns)
    }

    /// 闭环修正：把一次 `GET` 的命中误差折进偏移。
    ///
    /// `hit_error = 命中组的 group_timestamp_ns − 我们请求的 target`；
    /// 该量不含链路延迟。将目标推向已命中的帧网格（加残差），而不是反向推离。
    pub fn fold_hit_error(&mut self, hit_error_ns: i64) {
        self.offset_ns += hit_error_ns;
        self.corrections += 1;
    }

    pub fn offset_ns(&self) -> i64 {
        self.offset_ns
    }

    /// **帧内相位散布**（ns）：去掉整数帧支路后偏移的不确定度。门槛就看它。
    pub fn uncertainty_ns(&self) -> i64 {
        self.uncertainty_ns
    }

    /// **原始差散布**（ns，含整数帧台阶）：诊断量，与 `uncertainty_ns` 分开报。
    pub fn raw_spread_ns(&self) -> i64 {
        self.raw_spread_ns
    }

    /// 本次标定实测的帧周期（ns）：板端长基线，不写死 30/60 fps。
    pub fn frame_period_ns(&self) -> i64 {
        self.frame_period_ns
    }

    pub fn samples(&self) -> usize {
        self.samples
    }

    pub fn corrections(&self) -> usize {
        self.corrections
    }

    pub fn age(&self) -> Duration {
        self.calibrated_at.elapsed()
    }
}

/// 相邻样本必须严格前进；返回步长。非正/超 `i64` 一律当"不单调"（fail closed）。
fn step(
    current: i128,
    previous: i128,
    index: usize,
    field: &'static str,
) -> Result<i64, ClockError> {
    let delta = current - previous;
    if delta <= 0 || delta > i128::from(i64::MAX) {
        return Err(ClockError::NonMonotonic { index, field });
    }
    Ok(delta as i64)
}

fn narrow(value: i128) -> Result<i64, ClockError> {
    i64::try_from(value).map_err(|_| ClockError::TimestampOutOfRange(value))
}

/// 排序后取中位（偶数取上中位；全整数，无浮点）。
fn median(values: &mut [i64]) -> i64 {
    values.sort_unstable();
    values[values.len() / 2]
}

/// 最近秩 p95（升序排序后）。
fn percentile95(values: &mut [i64]) -> i64 {
    values.sort_unstable();
    let rank = (values.len() * 95).div_ceil(100);
    values[rank.saturating_sub(1).min(values.len() - 1)]
}

/// 候选相对中位数的 MAD 是否在限内：一步快、一步慢（变帧率/整窗网格不匀）即没有可靠节奏。
///
/// 少数**丢帧**造成的 2×T 离群值不动 MAD（多数候选仍在网格上），这是有意的：
/// 丢帧不改变帧网格，只是让"序号步长"少算了一帧。
fn cadence_is_uniform(candidates: &mut [i64], center: i64) -> bool {
    for candidate in candidates.iter_mut() {
        *candidate = (*candidate - center).abs();
    }
    let mad = median(candidates);
    i128::from(mad) * 100 <= i128::from(center) * i128::from(PERIOD_MAD_LIMIT_PERCENT)
}

/// 两个周期是否相差过大（`i64` 减法先升到 `i128`，周期为正常量级）。
fn rate_mismatch(left: i64, right: i64) -> bool {
    let difference = (i128::from(left) - i128::from(right)).abs();
    let smaller = left.abs().min(right.abs());
    difference * 100 > i128::from(smaller) * i128::from(RATE_MISMATCH_LIMIT_PERCENT)
}

/// 带符号的相位差，折到 `(−period/2, period/2]`。
fn signed_delta(delta: i64, period: i64) -> i64 {
    let wrapped = delta.rem_euclid(period);
    if wrapped > period / 2 {
        wrapped - period
    } else {
        wrapped
    }
}

/// 相位的圆均值（相位 ∈ `[0, period)`，浮点在这里安全）。
fn circular_mean(phases: &[i64], period: i64) -> i64 {
    let turn = std::f64::consts::TAU / period as f64;
    let (mut sin, mut cos) = (0.0_f64, 0.0_f64);
    for phase in phases {
        let angle = *phase as f64 * turn;
        sin += angle.sin();
        cos += angle.cos();
    }
    let mean = (sin.atan2(cos).rem_euclid(std::f64::consts::TAU) / turn).round() as i64;
    mean.rem_euclid(period)
}

#[cfg(test)]
mod tests {
    use super::{
        ClockAligner, ClockError, ClockSample, PHASE_UNCERTAINTY_LIMIT_NS, estimate_clock,
    };

    /// 合成用的帧周期（30 fps）与板端 realtime 纪元量级。
    const PERIOD: i64 = 33_333_333;
    const EPOCH_NS: i64 = 1_700_000_000_000_000_000;

    /// 造一段合成采样：两域**同速异域**（周期 `period`），板端帧比解码帧**领先 `ahead` 整帧**
    /// （现场把 `raw − pts` 撑成整帧台阶的那回事），解码时间戳再叠加 `jitter` 的亚帧抖动。
    ///
    /// `decoder_steps[i]`/`grid_steps[i]` 是第 i 个相邻对的"解码序号步长"/"帧网格步长"
    /// （下标 0 忽略）：两者不等即模拟**帧序号跳过**（丢帧）。真实常数偏移 = `board_base − decoder_base`。
    fn synthetic(
        decoder_base_ns: i64,
        board_base_ns: i64,
        period: i64,
        decoder_steps: &[i64],
        grid_steps: &[i64],
        ahead: &[i64],
        jitter: &[i64],
    ) -> Vec<ClockSample> {
        let count = ahead.len();
        assert_eq!(jitter.len(), count);
        assert_eq!(decoder_steps.len(), count);
        assert_eq!(grid_steps.len(), count);
        let mut samples = Vec::with_capacity(count);
        let (mut decoder_seq, mut grid_frames) = (1u64, 0i64);
        for index in 0..count {
            if index > 0 {
                decoder_seq += decoder_steps[index] as u64;
                grid_frames += grid_steps[index];
            }
            samples.push(ClockSample {
                decoder_seq,
                decoder_ns: decoder_base_ns + grid_frames * period + jitter[index],
                board_frame_id: 1 + (grid_frames + ahead[index]) as u64,
                board_ns: board_base_ns + (grid_frames + ahead[index]) * period,
            });
        }
        samples
    }

    fn difference(sample: &ClockSample) -> i128 {
        i128::from(sample.board_ns) - i128::from(sample.decoder_ns)
    }

    #[test]
    fn whole_frame_pairing_jumps_do_not_inflate_the_phase_spread() {
        // 现场形态：解码帧从 0 起算、板端在 realtime 纪元，LATEST 比解码帧领先 0–2 整帧。
        let ahead = [0, 1, 0, 2, 1, 0, 1];
        let jitter = [19_000, 5_000, 9_000, 2_000, 11_000, 4_000, 7_000];
        let steps = [3, 3, 4, 3, 3, 3, 3];
        let samples = synthetic(0, EPOCH_NS, PERIOD, &steps, &steps, &ahead, &jitter);
        let estimate = estimate_clock(&samples).expect("estimate");

        assert_eq!(
            estimate.period_ns, PERIOD,
            "周期直接来自采样，不写死 30/60 fps"
        );
        // 整帧配对跳变不应放大亚帧散布。
        assert!(
            estimate.uncertainty_ns <= 19_000,
            "uncertainty={}",
            estimate.uncertainty_ns
        );
        estimate
            .check_uncertainty(PHASE_UNCERTAINTY_LIMIT_NS)
            .expect("整帧支路变化但帧内相位稳定必须通过");
        // 两种散布必须分开：原始差里确实有整帧台阶，相位里没有。
        assert!(
            estimate.raw_spread_ns >= PERIOD,
            "raw_spread={}",
            estimate.raw_spread_ns
        );
        // 保守锚定：offset 不超过任何一次实测差，且离最小差不足一个帧周期。
        let minimum = samples.iter().map(difference).min().expect("非空");
        assert!(i128::from(estimate.offset_ns) <= minimum);
        assert!(minimum - i128::from(estimate.offset_ns) < i128::from(PERIOD));
        // 相位与真实常数一致（差在抖动范围内）；**不**声称绝对帧身份。
        let true_phase = i128::from(EPOCH_NS).rem_euclid(i128::from(PERIOD));
        let circular =
            (i128::from(estimate.phase_ns()) - true_phase).rem_euclid(i128::from(PERIOD));
        assert!(
            circular <= 19_000 || circular >= i128::from(PERIOD - 19_000),
            "circular={circular}"
        );
    }

    #[test]
    fn sixty_frames_per_second_is_measured_not_assumed() {
        let period = 16_666_667;
        let ahead = [0, 1, 2, 0, 1, 2, 0];
        let jitter = [0, 4_000, 8_000, 2_000, 6_000, 1_000, 3_000];
        let steps = [6, 6, 6, 6, 6, 6, 6];
        let samples = synthetic(0, EPOCH_NS, period, &steps, &steps, &ahead, &jitter);
        let estimate = estimate_clock(&samples).expect("estimate");
        assert_eq!(estimate.period_ns, period);
        assert!(estimate.uncertainty_ns <= 8_000);
        estimate
            .check_uncertainty(PHASE_UNCERTAINTY_LIMIT_NS)
            .expect("亚帧散布必须通过");
    }

    #[test]
    fn genuine_sub_frame_instability_still_fails_the_five_ms_gate() {
        // 解码时间戳真的在帧网格上抖 8 ms —— 这是"偏移有多准"的问题，必须 fail closed。
        let ahead = [0, 1, 0, 2, 1, 0, 1];
        let jitter = [
            8_000_000, 2_000_000, 6_000_000, 1_000_000, 7_000_000, 3_000_000, 500_000,
        ];
        let steps = [3, 3, 3, 3, 3, 3, 3];
        let samples = synthetic(0, EPOCH_NS, PERIOD, &steps, &steps, &ahead, &jitter);
        let estimate = estimate_clock(&samples).expect("estimate");
        assert!(
            estimate.uncertainty_ns > PHASE_UNCERTAINTY_LIMIT_NS,
            "uncertainty={}",
            estimate.uncertainty_ns
        );
        assert!(matches!(
            estimate.check_uncertainty(PHASE_UNCERTAINTY_LIMIT_NS),
            Err(ClockError::TooNoisy(_, _))
        ));
    }

    #[test]
    fn sequence_skips_and_epoch_scale_keep_the_grid() {
        // 丢帧（序号步长 < 网格步长）会让单域候选出现 1.5T/2T 离群值；
        // 中位数仍在网格上。两域都在纪元量级，取模/减锚点必须全整数，否则会丢几百纳秒。
        let decoder_steps = [3, 3, 1, 4, 3, 3, 2];
        let grid_steps = [3, 3, 2, 4, 3, 3, 3];
        let ahead = [0, 1, 2, 0, 1, 0, 2];
        let jitter = [19_000, 5_000, 9_000, 2_000, 11_000, 4_000, 7_000];
        let samples = synthetic(
            EPOCH_NS - 5_000_000_000,
            EPOCH_NS,
            PERIOD,
            &decoder_steps,
            &grid_steps,
            &ahead,
            &jitter,
        );
        let estimate = estimate_clock(&samples).expect("estimate");
        assert_eq!(estimate.period_ns, PERIOD, "丢帧不改网格周期");
        assert!(
            estimate.uncertainty_ns <= 19_000,
            "uncertainty={}",
            estimate.uncertainty_ns
        );
        // 最紧样本就是最小差样本 ⇒ 偏移正好落在最小实测差上（任何 epoch 级浮点量化都会打破它）。
        let minimum = samples.iter().map(difference).min().expect("非空");
        assert_eq!(i128::from(estimate.offset_ns), minimum);
    }

    #[test]
    fn non_monotonic_samples_are_rejected() {
        let ahead = [0, 1, 0, 1, 0, 1];
        let jitter = [0, 4_000, 2_000, 6_000, 1_000, 3_000];
        let steps = [3, 3, 3, 3, 3, 3];

        let mut stalled_frame = synthetic(0, EPOCH_NS, PERIOD, &steps, &steps, &ahead, &jitter);
        stalled_frame[3].board_frame_id = stalled_frame[2].board_frame_id;
        assert!(matches!(
            estimate_clock(&stalled_frame),
            Err(ClockError::NonMonotonic { .. })
        ));

        let mut rewound_pts = synthetic(0, EPOCH_NS, PERIOD, &steps, &steps, &ahead, &jitter);
        rewound_pts[4].decoder_ns = rewound_pts[3].decoder_ns - 1;
        assert!(matches!(
            estimate_clock(&rewound_pts),
            Err(ClockError::NonMonotonic { .. })
        ));
    }

    #[test]
    fn incompatible_domain_rates_are_rejected() {
        // 解码域 30 fps、板端域 60 fps：常数偏移模型不成立，必须报错而不是硬套一个周期。
        let samples: Vec<ClockSample> = (0..6i64)
            .map(|index| ClockSample {
                decoder_seq: 1 + index as u64,
                decoder_ns: index * PERIOD,
                board_frame_id: 1 + (index * 2) as u64,
                board_ns: EPOCH_NS + index * PERIOD,
            })
            .collect();
        assert!(matches!(
            estimate_clock(&samples),
            Err(ClockError::RateMismatch(_, _))
        ));
    }

    #[test]
    fn unreliable_cadence_is_rejected() {
        // 变帧率：每步都推进 3 个帧号，但时间戳步长在 20 ms / 46 ms 之间交替。
        // 这样的"帧网格"不存在可靠周期，不许折叠成成功。
        let mut samples = Vec::new();
        let (mut seq, mut frames) = (1u64, 1u64);
        let mut decoder_ns = 0i64;
        let mut board_ns = EPOCH_NS;
        for index in 0..8i64 {
            samples.push(ClockSample {
                decoder_seq: seq,
                decoder_ns,
                board_frame_id: frames,
                board_ns,
            });
            seq += 3;
            frames += 3;
            let step = if index % 2 == 0 {
                20_000_000
            } else {
                46_000_000
            };
            decoder_ns += step;
            board_ns += step;
        }
        let error = estimate_clock(&samples).expect_err("变帧率必须报错");
        assert!(matches!(error, ClockError::UnreliablePeriod(_)), "{error}");
    }

    #[test]
    fn too_few_samples_are_rejected() {
        let ahead = [0, 0, 0];
        let jitter = [0, 0, 0];
        let steps = [3, 3, 3];
        let samples = synthetic(0, EPOCH_NS, PERIOD, &steps, &steps, &ahead, &jitter);
        assert!(matches!(
            estimate_clock(&samples),
            Err(ClockError::TooFewSamples(3))
        ));
    }

    #[test]
    fn nearest_frame_feedback_removes_positive_and_negative_residuals() {
        for error in [-1_000_000, 1_000_000] {
            let mut aligner = ClockAligner {
                offset_ns: 1_000_000_000,
                uncertainty_ns: 21_000,
                raw_spread_ns: 33_400_000,
                frame_period_ns: 33_333_333,
                samples: 100,
                corrections: 0,
                calibrated_at: std::time::Instant::now(),
            };
            let pts = 5_000;
            let returned_frame = aligner.to_board_ns(pts) + error;
            aligner.fold_hit_error(aligner.residual_ns(returned_frame, pts));
            assert_eq!(aligner.residual_ns(returned_frame, pts), 0);
            aligner.fold_hit_error(aligner.residual_ns(returned_frame, pts));
            assert_eq!(aligner.to_board_ns(pts), returned_frame);
        }
    }
}
