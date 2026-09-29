//! AprilGrid 板几何：tag id ↔ 板坐标角点。
//!
//! 板坐标系原点在正视图左下角，x 向右、y 向上、z=0；tag 索引 `row = id / cols`、
//! `col = id % cols`；角点顺序由 `tag_corner_order` 给出（必须匹配实际打印方向，默认
//! `[bottom_left, bottom_right, top_right, top_left]`）。

use serde::{Deserialize, Serialize};

/// 单个 tag 的四个角点在 tag 局部坐标里的位置（单位化，乘 tag 边长即得板坐标）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Corner {
    BottomLeft,
    BottomRight,
    TopRight,
    TopLeft,
}

impl Corner {
    fn unit(self) -> [f64; 2] {
        match self {
            Corner::BottomLeft => [0.0, 0.0],
            Corner::BottomRight => [1.0, 0.0],
            Corner::TopRight => [1.0, 1.0],
            Corner::TopLeft => [0.0, 1.0],
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "bottom_left" => Some(Corner::BottomLeft),
            "bottom_right" => Some(Corner::BottomRight),
            "top_right" => Some(Corner::TopRight),
            "top_left" => Some(Corner::TopLeft),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Corner::BottomLeft => "bottom_left",
            Corner::BottomRight => "bottom_right",
            Corner::TopRight => "top_right",
            Corner::TopLeft => "top_left",
        }
    }
}

pub const DEFAULT_CORNER_ORDER: [Corner; 4] = [
    Corner::BottomLeft,
    Corner::BottomRight,
    Corner::TopRight,
    Corner::TopLeft,
];

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AprilGridConfig {
    pub rows: usize,
    pub cols: usize,
    pub tag_size_m: f64,
    pub tag_spacing_ratio: f64,
    pub first_tag_id: usize,
    pub dictionary: String,
    pub target_id: String,
    pub tag_corner_order: [Corner; 4],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BoardError {
    EmptyGrid,
    NonPositiveTagSize,
    NegativeSpacing,
    TagOutsideGrid(usize),
    BadCornerOrder,
}

impl std::fmt::Display for BoardError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BoardError::EmptyGrid => write!(f, "rows and cols must be positive"),
            BoardError::NonPositiveTagSize => {
                write!(f, "tag_size_m must be a measured positive value")
            }
            BoardError::NegativeSpacing => {
                write!(f, "tag_spacing_ratio must be measured and non-negative")
            }
            BoardError::TagOutsideGrid(id) => {
                write!(f, "tag_id {id} is outside the configured grid")
            }
            BoardError::BadCornerOrder => write!(
                f,
                "tag_corner_order must contain bottom_left, bottom_right, top_right, top_left exactly once"
            ),
        }
    }
}

impl std::error::Error for BoardError {}

impl AprilGridConfig {
    pub fn new(
        rows: usize,
        cols: usize,
        tag_size_m: f64,
        tag_spacing_ratio: f64,
    ) -> Result<Self, BoardError> {
        let config = Self {
            rows,
            cols,
            tag_size_m,
            tag_spacing_ratio,
            first_tag_id: 0,
            dictionary: "DICT_APRILTAG_36h11".to_owned(),
            target_id: "target".to_owned(),
            tag_corner_order: DEFAULT_CORNER_ORDER,
        };
        config.validate()?;
        Ok(config)
    }

    pub fn validate(&self) -> Result<(), BoardError> {
        if self.rows == 0 || self.cols == 0 {
            return Err(BoardError::EmptyGrid);
        }
        if !self.tag_size_m.is_finite() || self.tag_size_m <= 0.0 {
            return Err(BoardError::NonPositiveTagSize);
        }
        if !self.tag_spacing_ratio.is_finite() || self.tag_spacing_ratio < 0.0 {
            return Err(BoardError::NegativeSpacing);
        }
        let mut seen = [false; 4];
        for corner in self.tag_corner_order {
            let slot = match corner {
                Corner::BottomLeft => 0,
                Corner::BottomRight => 1,
                Corner::TopRight => 2,
                Corner::TopLeft => 3,
            };
            if seen[slot] {
                return Err(BoardError::BadCornerOrder);
            }
            seen[slot] = true;
        }
        Ok(())
    }

    pub fn tag_count(&self) -> usize {
        self.rows * self.cols
    }

    pub fn tag_pitch_m(&self) -> f64 {
        self.tag_size_m * (1.0 + self.tag_spacing_ratio)
    }

    /// 单个 tag 的四个板坐标角点，顺序 = `tag_corner_order`。
    pub fn object_corners_for_tag(&self, tag_id: usize) -> Result<[[f64; 3]; 4], BoardError> {
        let local_id = tag_id
            .checked_sub(self.first_tag_id)
            .ok_or(BoardError::TagOutsideGrid(tag_id))?;
        if local_id >= self.tag_count() {
            return Err(BoardError::TagOutsideGrid(tag_id));
        }
        let (row, col) = (local_id / self.cols, local_id % self.cols);
        let pitch = self.tag_pitch_m();
        let (x0, y0) = (col as f64 * pitch, row as f64 * pitch);
        let size = self.tag_size_m;
        let mut corners = [[0.0; 3]; 4];
        for (index, corner) in self.tag_corner_order.iter().enumerate() {
            let [ux, uy] = corner.unit();
            corners[index] = [x0 + ux * size, y0 + uy * size, 0.0];
        }
        Ok(corners)
    }

    /// 一组 tag 的角点按 `tag_ids` 顺序拼接。
    pub fn object_points(&self, tag_ids: &[usize]) -> Result<Vec<[f64; 3]>, BoardError> {
        let mut points = Vec::with_capacity(tag_ids.len() * 4);
        for tag_id in tag_ids {
            points.extend_from_slice(&self.object_corners_for_tag(*tag_id)?);
        }
        Ok(points)
    }
}
