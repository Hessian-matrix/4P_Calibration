//! 预览与进度（无 GUI 依赖）。
//!
//! 终端预览将检测叠加画面缩放为 ANSI 真彩色半块字符；`--preview-out` 保存首帧 PNG。
//! 收敛进度使用 `value/limit` 进度条，越小越好；不依赖 OpenCV highgui。

use opencv::core::{Mat, Point, Scalar};
use opencv::prelude::*;
use opencv::{imgcodecs, imgproc};

use crate::detect::Detection;

pub fn draw_detection(
    canvas: &mut Mat,
    detection: &Detection,
    header: &str,
) -> Result<(), opencv::Error> {
    let green = Scalar::new(80.0, 230.0, 80.0, 0.0);
    let amber = Scalar::new(0.0, 215.0, 255.0, 0.0);
    let white = Scalar::new(235.0, 235.0, 235.0, 0.0);
    let color = if detection.detected() { green } else { amber };
    if detection.detected() {
        for quad in detection.image_points.chunks(4) {
            for index in 0..4 {
                let a = quad[index];
                let b = quad[(index + 1) % 4];
                imgproc::line(
                    canvas,
                    Point::new(a[0] as i32, a[1] as i32),
                    Point::new(b[0] as i32, b[1] as i32),
                    color,
                    2,
                    imgproc::LINE_AA,
                    0,
                )?;
            }
        }
    }
    imgproc::put_text_def(
        canvas,
        header,
        Point::new(12, 34),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.8,
        white,
    )?;
    let status = format!("{}  tags={}", detection.status, detection.tag_ids.len());
    imgproc::put_text_def(
        canvas,
        &status,
        Point::new(12, 66),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
    )?;
    Ok(())
}

pub fn save_snapshot(canvas: &Mat, path: &str) -> Result<(), opencv::Error> {
    imgcodecs::imwrite(path, canvas, &opencv::core::Vector::new()).map(|_| ())
}

/// BGR 画面 → ANSI 真彩色半块字符（每格代表上下两个像素）。
pub fn render_terminal(canvas: &Mat, columns: usize) -> Result<String, opencv::Error> {
    let (width, height) = (canvas.cols(), canvas.rows());
    if width <= 0 || height <= 0 || columns == 0 {
        return Ok(String::new());
    }
    let rows_px =
        (((columns as f64 * (height as f64 / width as f64)) / 2.0).round() as i32).max(1) * 2;
    let mut small = Mat::default();
    imgproc::resize(
        canvas,
        &mut small,
        opencv::core::Size::new(columns as i32, rows_px),
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;
    if !small.is_continuous() {
        small = small.clone();
    }
    let bytes = small.data_bytes()?;
    let stride = columns * 3;
    let mut out = String::with_capacity(columns * rows_px as usize * 24);
    let mut row = 0;
    while row + 1 < rows_px as usize {
        for column in 0..columns {
            let top = (row * stride) + column * 3;
            let bottom = ((row + 1) * stride) + column * 3;
            let (tr, tg, tb) = (bytes[top + 2], bytes[top + 1], bytes[top]);
            let (br, bg, bb) = (bytes[bottom + 2], bytes[bottom + 1], bytes[bottom]);
            out.push_str(&format!(
                "\x1b[38;2;{tr};{tg};{tb}m\x1b[48;2;{br};{bg};{bb}m▀"
            ));
        }
        out.push_str("\x1b[0m\n");
        row += 2;
    }
    Ok(out)
}

/// 一条进度条：`value/limit` 越小越好。
pub fn bar(label: &str, value: f64, limit: f64, width: usize) -> String {
    let ratio = if limit > 0.0 && value.is_finite() {
        (value / limit).clamp(0.0, 1.0)
    } else {
        1.0
    };
    let filled = ((1.0 - ratio) * width as f64).round() as usize;
    let shown = if value.is_finite() {
        format!("{value:.4}")
    } else {
        "n/a".to_owned()
    };
    format!(
        "{label:<14}[{}{}] {shown} / {limit:.4}",
        "█".repeat(filled),
        "░".repeat(width - filled)
    )
}

/// 会话状态 → 一屏进度条（每次入库后重绘）。
pub fn render_session_bars(
    state: &rigcal_core::session::SessionState,
    thresholds: rigcal_core::session::SessionThresholds,
) -> Vec<String> {
    vec![
        bar("rms_px", state.rms_px, thresholds.max_rms_px, 22),
        bar(
            "focal_sigma",
            state.focal_relative_stddev.0,
            thresholds.max_focal_relative_stddev,
            22,
        ),
        bar(
            "principal_px",
            state.principal_stddev_px.0,
            thresholds.max_principal_stddev_px,
            22,
        ),
        bar(
            "holdout_rms",
            state.holdout_rms_px,
            thresholds.max_holdout_rms_px,
            22,
        ),
        format!(
            "views={:<3} used={:<3} excl={:<3} holdout={:<3} rank={} solves={} failures={} streak={}/{} {}",
            state.views,
            state.used_views,
            state.excluded_views,
            state.holdout_views,
            state.rank,
            state.solves,
            state.failures,
            state.streak,
            thresholds.window,
            if state.converged { "CONVERGED" } else { "" }
        ),
        format!("  {}", state.detail),
    ]
}
