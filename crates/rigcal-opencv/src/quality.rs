//! 采集质量门禁（在线筛帧）：清晰度 / 对比度 / 削顶比例。
//!
//! - 清晰度：[`tenengrad`] —— **全分辨率**灰度上 Sobel(ksize=3) 梯度平方的全图均值
//!   `mean(Gx² + Gy²)`，单帧计算，不做 LK/多帧跟踪。单位是「灰度级²/像素²」：
//!   平场/暗场恒为 0（有限、无 NaN），8 bit 读出噪声（σ≈2）地板约 10²，普通清晰纹理在 10³ 量级。
//!   阈值是**初值**，需按现场光学/增益调优，不是标定过的判据。
//! - 对比度：灰度 `p95 − p5`（numpy 默认线性插值语义，本模块用直方图求**次序统计量**，不用近似）；
//! - 削顶：`mean(gray ≤ 2)` 与 `mean(gray ≥ 253)`，两者之一超过阈值且另一侧小于 5% 才判 `SATURATED`
//!   ——合成棋盘会两端饱和但仍有有效边缘，单侧大面积削顶才是真实曝光异常。

use opencv::core::Mat;
use opencv::prelude::*;
use opencv::{core, imgproc};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QualityThresholds {
    pub min_focus_score: f64,
    pub min_contrast: f64,
    pub max_saturated_fraction: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FrameQuality {
    pub accepted: bool,
    pub reasons: Vec<&'static str>,
    pub focus_score: f64,
    pub contrast: f64,
    pub saturated_fraction: f64,
    pub low_clip_fraction: f64,
    pub high_clip_fraction: f64,
}

/// 判定一帧（mono8）是否可用于标定；拒绝理由直接给出，便于现场定位。
pub fn classify_frame_quality(
    gray: &Mat,
    thresholds: &QualityThresholds,
) -> Result<FrameQuality, opencv::Error> {
    if gray.channels() != 1 {
        return Err(opencv::Error::new(
            core::StsError,
            "quality input must be mono8",
        ));
    }
    let focus_score = tenengrad(gray)?;
    let (contrast, low_clip, high_clip) = gray_statistics(gray)?;
    let saturated = low_clip + high_clip;
    let mut reasons = Vec::new();
    if focus_score < thresholds.min_focus_score {
        reasons.push("LOW_FOCUS");
    }
    if contrast < thresholds.min_contrast {
        reasons.push("LOW_CONTRAST");
    }
    if low_clip.max(high_clip) > thresholds.max_saturated_fraction && low_clip.min(high_clip) < 0.05
    {
        reasons.push("SATURATED");
    }
    Ok(FrameQuality {
        accepted: reasons.is_empty(),
        reasons,
        focus_score,
        contrast,
        saturated_fraction: saturated,
        low_clip_fraction: low_clip,
        high_clip_fraction: high_clip,
    })
}

/// 全分辨率 Tenengrad：`Sobel_x`、`Sobel_y`（ksize=3，`CV_32F`）后取 `Gx² + Gy²` 的全图均值。
///
/// 常量画面（平场/暗场）恒为 0.0，空图为 0.0：有限、可解释、无 NaN 分支。
fn tenengrad(gray: &Mat) -> Result<f64, opencv::Error> {
    let mut gx = Mat::default();
    let mut gy = Mat::default();
    imgproc::sobel(
        gray,
        &mut gx,
        core::CV_32F,
        1,
        0,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    imgproc::sobel(
        gray,
        &mut gy,
        core::CV_32F,
        0,
        1,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let gx = gx.data_typed::<f32>()?;
    let gy = gy.data_typed::<f32>()?;
    if gx.is_empty() {
        return Ok(0.0);
    }
    let mut total = 0.0f64;
    for (x, y) in gx.iter().zip(gy.iter()) {
        total += (*x as f64) * (*x as f64) + (*y as f64) * (*y as f64);
    }
    Ok(total / gx.len() as f64)
}

/// 直方图统计：`p95 − p5`（次序统计量线性插值）与两侧削顶比例。
fn gray_statistics(gray: &Mat) -> Result<(f64, f64, f64), opencv::Error> {
    let pixels = gray.data_typed::<u8>()?;
    let mut histogram = [0usize; 256];
    for value in pixels {
        histogram[*value as usize] += 1;
    }
    let total = pixels.len();
    if total == 0 {
        return Ok((0.0, 0.0, 0.0));
    }
    let low_clip = histogram[..=2].iter().sum::<usize>() as f64 / total as f64;
    let high_clip = histogram[253..].iter().sum::<usize>() as f64 / total as f64;
    let p5 = order_statistic(&histogram, total, 5.0);
    let p95 = order_statistic(&histogram, total, 95.0);
    Ok((p95 - p5, low_clip, high_clip))
}

/// `numpy.percentile(values, percent)`（默认线性插值）在整数灰度上的精确实现：
/// 先按直方图定位次序统计量，再在相邻两个次序统计量之间线性插值。
fn order_statistic(histogram: &[usize; 256], total: usize, percent: f64) -> f64 {
    let position = (percent / 100.0) * (total - 1) as f64;
    let lower_rank = position.floor() as usize;
    let upper_rank = position.ceil() as usize;
    let lower = value_at_rank(histogram, lower_rank);
    if lower_rank == upper_rank {
        return lower;
    }
    let upper = value_at_rank(histogram, upper_rank);
    lower + (upper - lower) * (position - lower_rank as f64)
}

/// 第 `rank` 个（0 基）次序统计量的灰度值。
fn value_at_rank(histogram: &[usize; 256], rank: usize) -> f64 {
    let mut seen = 0usize;
    for (value, count) in histogram.iter().enumerate() {
        seen += *count;
        if seen > rank {
            return value as f64;
        }
    }
    255.0
}

#[cfg(test)]
mod tests {
    use super::order_statistic;

    #[test]
    fn order_statistics_match_numpy_interpolation() {
        // 灰度分布：100 个 10、100 个 20 → numpy.percentile(v, 50) == 15.0
        let mut histogram = [0usize; 256];
        histogram[10] = 100;
        histogram[20] = 100;
        assert!((order_statistic(&histogram, 200, 50.0) - 15.0).abs() < 1e-12);
        // p5 → 位置 0.05*199 = 9.95 → 落在 10 与 10 之间（前 100 个都是 10）
        assert!((order_statistic(&histogram, 200, 5.0) - 10.0).abs() < 1e-12);
        // p95 → 位置 0.95*199 = 189.05 → 落在 20 与 20 之间
        assert!((order_statistic(&histogram, 200, 95.0) - 20.0).abs() < 1e-12);
        // 单一值：任何分位都等于它
        let mut single = [0usize; 256];
        single[77] = 5;
        assert!((order_statistic(&single, 5, 95.0) - 77.0).abs() < 1e-12);
    }
}
