//! AprilTag 检测与角点精修。
//!
//! 0.75× 检出后在全分辨率灰度图上做 7×7 亚像素精修，再按 tag id 映射板坐标角点。
//! 黑边比特数未锁定时按板型尝试 `2 → 1`；首次成功后由调用方锁定。
//!
//! 失败一律 fail closed：tag id 越界、角点非有限、角点数与物点不齐都返回错误而不是静默丢点。

use opencv::core::{Mat, Point2f, Size, TermCriteria, Vector};
use opencv::objdetect::{
    self, ArucoDetector, DetectorParameters, DetectorParametersTrait, PredefinedDictionaryType,
    RefineParameters,
};
use opencv::prelude::*;
use opencv::{imgcodecs, imgproc};
use rigcal_core::AprilGridConfig;

/// 证据帧的检出尺度。
pub const DETECT_SCALE: f64 = 0.75;
/// 全分辨率亚像素精修窗口。
pub const REFINE_WINDOW: i32 = 7;

#[derive(Clone, Debug)]
pub struct Detection {
    pub status: &'static str,
    pub tag_ids: Vec<usize>,
    /// 全分辨率像素坐标，顺序与 `object_points` 一一对应。
    pub image_points: Vec<[f64; 2]>,
    pub object_points: Vec<[f64; 3]>,
    /// 检测器拒绝的候选数（诊断用）。
    pub rejected: usize,
    /// 检出所用的黑边比特数；调用方锁定后，后续帧只检测一次。
    pub border_bits: Option<i32>,
}

impl Detection {
    fn failed(status: &'static str, rejected: usize) -> Self {
        Self {
            status,
            tag_ids: Vec::new(),
            image_points: Vec::new(),
            object_points: Vec::new(),
            rejected,
            border_bits: None,
        }
    }

    pub fn detected(&self) -> bool {
        !self.tag_ids.is_empty()
    }
}

fn dictionary_for(board: &AprilGridConfig) -> Option<PredefinedDictionaryType> {
    Some(match board.dictionary.as_str() {
        "DICT_APRILTAG_36h11" => PredefinedDictionaryType::DICT_APRILTAG_36h11,
        "DICT_APRILTAG_36h10" => PredefinedDictionaryType::DICT_APRILTAG_36h10,
        "DICT_APRILTAG_25h9" => PredefinedDictionaryType::DICT_APRILTAG_25h9,
        "DICT_APRILTAG_16h5" => PredefinedDictionaryType::DICT_APRILTAG_16h5,
        "DICT_ARUCO_MIP_36h12" => PredefinedDictionaryType::DICT_ARUCO_MIP_36h12,
        _ => return None,
    })
}

/// 黑边比特数候选：AprilTag 板先试 2 位（实拍板约定），失败回退 1 位。
fn border_bits_candidates(board: &AprilGridConfig, locked: Option<i32>) -> Vec<i32> {
    match locked {
        Some(bits) => vec![bits],
        None if board.dictionary.starts_with("DICT_APRILTAG_") => vec![2, 1],
        None => vec![1],
    }
}

fn detector(board: &AprilGridConfig, border_bits: i32) -> Result<ArucoDetector, opencv::Error> {
    let name = dictionary_for(board).ok_or_else(|| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("unsupported dictionary: {}", board.dictionary),
        )
    })?;
    let dictionary = objdetect::get_predefined_dictionary(name)?;
    let mut parameters = DetectorParameters::default()?;
    parameters.set_marker_border_bits(border_bits);
    parameters.set_corner_refinement_method(objdetect::CORNER_REFINE_APRILTAG);
    let refine = RefineParameters::new_def()?;
    ArucoDetector::new(&dictionary, &parameters, refine)
}

/// 缩放到指定宽度（保持宽高比，`INTER_AREA`）：预览显示用（终端/仪表盘缩图）。
pub fn downscale_to_width(gray: &Mat, width: i32) -> Result<Mat, opencv::Error> {
    let scale = width as f64 / gray.cols() as f64;
    let height = ((gray.rows() as f64 * scale).round() as i32).max(1);
    let mut out = Mat::default();
    imgproc::resize(
        gray,
        &mut out,
        opencv::core::Size::new(width, height),
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;
    Ok(out)
}

fn downscale(gray: &Mat) -> Result<Mat, opencv::Error> {
    let mut small = Mat::default();
    imgproc::resize(
        gray,
        &mut small,
        Size::default(),
        DETECT_SCALE,
        DETECT_SCALE,
        imgproc::INTER_AREA,
    )?;
    Ok(small)
}

/// 在全分辨率灰度图上把降采样检出得到的角点做亚像素精修（窗口放不下时保留原值，不崩）。
fn refine_corners(
    gray: &Mat,
    corners: &Mat,
    inverse_scale: f64,
) -> Result<Vec<[f64; 2]>, opencv::Error> {
    let raw = corners.data_typed::<Point2f>()?;
    let scaled: Vec<Point2f> = raw
        .iter()
        .map(|point| {
            Point2f::new(
                point.x * inverse_scale as f32,
                point.y * inverse_scale as f32,
            )
        })
        .collect();
    // 先建空 Mat 再写入：`Mat::from_slice` 返回借用包装（BoxedRef），不能作为 in/out 传给精修
    let mut refined = Mat::new_rows_cols_with_default(
        4,
        1,
        opencv::core::CV_32FC2,
        opencv::core::Scalar::default(),
    )?;
    {
        let values = refined.data_typed_mut::<Point2f>()?;
        for (slot, point) in scaled.iter().enumerate() {
            values[slot] = *point;
        }
    }
    let radius = REFINE_WINDOW / 2;
    let (width, height) = (gray.cols(), gray.rows());
    let refinable = scaled.iter().all(|point| {
        let (x, y) = (point.x.round() as i32, point.y.round() as i32);
        x >= radius && x <= width - 1 - radius && y >= radius && y <= height - 1 - radius
    });
    if refinable {
        let criteria = TermCriteria::new(
            opencv::core::TermCriteria_COUNT | opencv::core::TermCriteria_EPS,
            20,
            0.01,
        )?;
        imgproc::corner_sub_pix(
            gray,
            &mut refined,
            Size::new(REFINE_WINDOW, REFINE_WINDOW),
            Size::new(-1, -1),
            criteria,
        )?;
    }
    let values = refined.data_typed::<Point2f>()?;
    Ok(values
        .iter()
        .map(|point| [point.x as f64, point.y as f64])
        .collect())
}

/// 单帧检测：返回观测与状态；`locked_border_bits` 命中后由调用方锁定，后续帧只跑一遍。
pub fn detect_board(
    gray: &Mat,
    board: &AprilGridConfig,
    locked_border_bits: Option<i32>,
) -> Result<Detection, opencv::Error> {
    if gray.channels() != 1 {
        return Err(opencv::Error::new(
            opencv::core::StsError,
            "detection expects a single-channel (grayscale) frame",
        ));
    }
    let small = downscale(gray)?;
    let mut last_failure = Detection::failed("NO_DETECTION", 0);
    for border_bits in border_bits_candidates(board, locked_border_bits) {
        let detector = detector(board, border_bits)?;
        let mut corners = Vector::<Mat>::new();
        let mut ids = Mat::default();
        let mut rejected = Vector::<Mat>::new();
        detector.detect_markers(&small, &mut corners, &mut ids, &mut rejected)?;
        if ids.empty() {
            if last_failure.status == "NO_DETECTION" {
                last_failure = Detection::failed("NO_DETECTION", rejected.len());
            }
            continue;
        }
        let identifiers = ids.data_typed::<i32>()?;
        if identifiers.len() != corners.len() {
            return Err(opencv::Error::new(
                opencv::core::StsError,
                "detector returned mismatched corner/id counts",
            ));
        }
        let mut seen = Vec::with_capacity(identifiers.len());
        let mut marker_corners = Vec::with_capacity(identifiers.len());
        for (index, identifier) in identifiers.iter().enumerate() {
            let tag_id = *identifier as usize;
            if tag_id < board.first_tag_id || tag_id >= board.first_tag_id + board.tag_count() {
                return Ok(Detection::failed("UNKNOWN_TAG_ID", rejected.len()));
            }
            if seen.contains(&tag_id) {
                return Ok(Detection::failed("DUPLICATE_TAG_ID", rejected.len()));
            }
            seen.push(tag_id);
            marker_corners.push(corners.get(index)?);
        }

        let inverse_scale = 1.0 / DETECT_SCALE;
        let mut image_points = Vec::with_capacity(seen.len() * 4);
        let mut object_points = Vec::with_capacity(seen.len() * 4);
        for (tag_id, corners_of_tag) in seen.iter().zip(marker_corners.iter()) {
            let refined = refine_corners(gray, corners_of_tag, inverse_scale)?;
            if refined
                .iter()
                .any(|point| !point[0].is_finite() || !point[1].is_finite())
            {
                return Ok(Detection::failed("NONFINITE_CORNERS", rejected.len()));
            }
            image_points.extend_from_slice(&refined);
            let object = board
                .object_corners_for_tag(*tag_id)
                .map_err(|error| opencv::Error::new(opencv::core::StsError, error.to_string()))?;
            object_points.extend_from_slice(&object);
        }
        return Ok(Detection {
            status: "DETECTED",
            tag_ids: seen,
            image_points,
            object_points,
            rejected: rejected.len(),
            border_bits: Some(border_bits),
        });
    }
    Ok(last_failure)
}

/// 读入灰度帧（PNG/JPG 都走 imgcodecs）。
pub fn load_gray(path: &str) -> Result<Mat, opencv::Error> {
    imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE)
}

/// 灰度 → BGR 画布（叠加层与窗口都要求三通道）。
pub fn to_canvas(gray: &Mat) -> Result<Mat, opencv::Error> {
    let mut canvas = Mat::default();
    // `cvt_color_def` = `cvt_color(src, dst, code, dst_cn=0, hint=ALGO_HINT_DEFAULT)`：
    // 用默认参数版本，4.x（无 `AlgorithmHint`）与 5.x 是同一个绑定名。
    imgproc::cvt_color_def(gray, &mut canvas, imgproc::COLOR_GRAY2BGR)?;
    Ok(canvas)
}
