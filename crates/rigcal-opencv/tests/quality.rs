//! 质量门禁的行为边界（清晰度 / 对比度 / 削顶）。
//!
//! 不逐值 pin 分数：清晰度用 Tenengrad，这里只断言**可观察的接受/拒绝行为**与有限性——
//! 清晰纹理可成候选，平场/暗场/模糊/低对比/单侧削顶被拒。

use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use rigcal_opencv::quality::{FrameQuality, QualityThresholds, classify_frame_quality};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 1088;

fn thresholds() -> QualityThresholds {
    QualityThresholds {
        min_focus_score: 500.0,
        min_contrast: 20.0,
        max_saturated_fraction: 0.35,
    }
}

fn mat_of(values: impl Fn(i32, i32) -> u8) -> Mat {
    let mut image =
        Mat::new_rows_cols_with_default(HEIGHT, WIDTH, opencv::core::CV_8UC1, Scalar::all(0.0))
            .expect("image");
    {
        let data = image.data_typed_mut::<u8>().expect("data");
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                data[(y * WIDTH + x) as usize] = values(x, y);
            }
        }
    }
    image
}

fn classify(image: &Mat) -> FrameQuality {
    let quality = classify_frame_quality(image, &thresholds()).expect("quality");
    assert!(
        quality.focus_score.is_finite(),
        "focus score must be finite: {quality:?}"
    );
    quality
}

#[test]
fn sharp_texture_is_accepted() {
    // 2 px 棋盘：高频边缘充足；两端各半并不算「单侧削顶」。
    let quality = classify(&mat_of(
        |x, y| {
            if ((x / 2) + (y / 2)) % 2 == 0 { 0 } else { 255 }
        },
    ));
    assert!(quality.accepted, "sharp texture must pass: {quality:?}");
    assert!(quality.reasons.is_empty(), "{quality:?}");
    assert!(quality.focus_score > thresholds().min_focus_score);
}

#[test]
fn flat_and_dark_fields_are_rejected_finite() {
    for level in [128u8, 3u8] {
        let quality = classify(&mat_of(|_, _| level));
        assert!(
            !quality.accepted,
            "flat field must be rejected: {quality:?}"
        );
        assert!(quality.reasons.contains(&"LOW_FOCUS"), "{quality:?}");
        assert!(quality.reasons.contains(&"LOW_CONTRAST"), "{quality:?}");
        assert_eq!(quality.focus_score, 0.0);
    }
}

#[test]
fn smooth_blur_has_no_high_frequency() {
    // 缓变斜坡（清晰度的「模糊」代理）：对比度充足，但没有高频边缘 → 只判低清晰度。
    let quality = classify(&mat_of(|x, _| (x * 255 / (WIDTH - 1)) as u8));
    assert!(!quality.accepted, "{quality:?}");
    assert!(quality.reasons.contains(&"LOW_FOCUS"), "{quality:?}");
    assert!(!quality.reasons.contains(&"LOW_CONTRAST"), "{quality:?}");
}

#[test]
fn low_contrast_texture_is_rejected() {
    // 幅度 10 的棋盘：有边缘但动态范围不足 → LOW_CONTRAST。
    let quality = classify(&mat_of(|x, y| {
        if ((x / 2) + (y / 2)) % 2 == 0 {
            123
        } else {
            133
        }
    }));
    assert!(!quality.accepted, "{quality:?}");
    assert!(quality.reasons.contains(&"LOW_CONTRAST"), "{quality:?}");
}

#[test]
fn one_sided_clipping_is_rejected() {
    // 上半 255、下半中灰纹理：大面积高侧削顶且低侧几乎为零 → SATURATED。
    let quality = classify(&mat_of(|x, y| {
        if y < HEIGHT / 2 {
            255
        } else if ((x / 2) + (y / 2)) % 2 == 0 {
            40
        } else {
            120
        }
    }));
    assert!(!quality.accepted, "{quality:?}");
    assert!(quality.reasons.contains(&"SATURATED"), "{quality:?}");
}
