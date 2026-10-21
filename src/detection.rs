use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;

use anyhow::{Context, Result, ensure};
use ccl_calib::camera::CalibrationImageSize;
use ccl_image::{apriltag, detection};
use serde::Serialize;

use crate::config::{RefinementMethodConfig, RunSettings, TagFamilyConfig, TargetConfig};

#[derive(Clone, Debug)]
pub struct DetectedFrame {
    pub camera_index: usize,
    pub image_size: CalibrationImageSize,
    pub target_indices: Vec<usize>,
    pub pixels: Vec<[f64; 2]>,
}

/// One image observation set. `timestamp_ns` is the view's timeline anchor: exact-time union
/// views contain only frames published at that instant, while approximately synchronized
/// views carry the anchor camera's own source time and may hold frames within the tolerance.
#[derive(Clone, Debug)]
pub struct DetectedView {
    pub id: String,
    pub timestamp_ns: Option<i64>,
    pub frames: Vec<Arc<DetectedFrame>>,
}

#[derive(Clone, Debug)]
pub struct DetectedDataset {
    /// Exact-time union: one view per distinct source timestamp, holding only the cameras
    /// that published at that timestamp. Continuous-time consumes this so asynchronous
    /// cameras keep their own timestamps instead of being retimed to one anchor.
    pub views: Vec<DetectedView>,
    /// Approximately synchronized shared views anchored on the configured anchor camera,
    /// timed by that camera's own header stamps. The visual rig seed (and the preintegration
    /// path) consume this; it is never rebuilt by a second detection pass.
    pub synchronized_views: Vec<DetectedView>,
    pub image_sizes: Vec<CalibrationImageSize>,
    pub imu_samples: Option<Vec<ccl_calib::camera::inertial::ImuSample>>,
    pub time_origin_ns: Option<i64>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DetectionSummary {
    pub input_views: usize,
    pub usable_views: usize,
    pub usable_frames_by_camera: BTreeMap<String, usize>,
    pub tags_by_camera: BTreeMap<String, usize>,
    pub points_by_camera: BTreeMap<String, usize>,
    pub ignored_tags: usize,
    pub refinement_failures: usize,
}

#[derive(Clone, Debug)]
pub struct FrameDetection {
    pub frame: Option<DetectedFrame>,
    pub accepted_tags: usize,
    pub points: usize,
    pub ignored_tags: usize,
    pub refinement_failures: usize,
}

#[derive(Clone)]
pub struct FrameDetector {
    options: apriltag::DetectorOptions,
    target: TargetProperties,
}

pub struct DetectionStats<'a> {
    settings: &'a RunSettings,
    usable_frames: Vec<usize>,
    tags: Vec<usize>,
    points: Vec<usize>,
    ignored_tags: usize,
    refinement_failures: usize,
}

impl FrameDetector {
    #[must_use]
    pub fn new(settings: &RunSettings) -> Self {
        Self {
            options: options(settings),
            target: target_properties(settings),
        }
    }

    pub fn detect(
        &self,
        camera_index: usize,
        image_size: CalibrationImageSize,
        image: detection::ImageView<'_>,
        context: &str,
    ) -> Result<FrameDetection> {
        let control = detection::DetectorControl::default();
        let report = apriltag::detect_tags_with_options(image, self.options, &control)
            .with_context(|| format!("detect AprilGrid in {context}"))?;
        let refinement_failures = report.failures.len();
        let mut ignored_tags = 0;
        let mut seen = HashSet::new();
        let mut target_indices = Vec::new();
        let mut image_points = Vec::new();
        let mut accepted_tags = 0;
        for tag in report.detections {
            let Some(&base) = self.target.base_index_by_tag.get(&tag.id) else {
                ignored_tags += 1;
                continue;
            };
            ensure!(
                seen.insert(tag.id),
                "duplicate target tag {} in {context}",
                tag.id
            );
            for (detector_corner, point) in tag.corners.iter().enumerate() {
                target_indices
                    .push(base + self.target.decoded_corner_to_front_corner[detector_corner]);
                image_points.push([f64::from(point.x), f64::from(point.y)]);
            }
            accepted_tags += 1;
        }
        let points = image_points.len();
        Ok(FrameDetection {
            frame: (accepted_tags > 0).then_some(DetectedFrame {
                camera_index,
                image_size,
                target_indices,
                pixels: image_points,
            }),
            accepted_tags,
            points,
            ignored_tags,
            refinement_failures,
        })
    }
}

impl<'a> DetectionStats<'a> {
    #[must_use]
    pub fn new(settings: &'a RunSettings) -> Self {
        Self {
            settings,
            usable_frames: vec![0; settings.cameras.len()],
            tags: vec![0; settings.cameras.len()],
            points: vec![0; settings.cameras.len()],
            ignored_tags: 0,
            refinement_failures: 0,
        }
    }

    pub fn record(&mut self, camera_index: usize, detection: &FrameDetection) {
        self.ignored_tags += detection.ignored_tags;
        self.refinement_failures += detection.refinement_failures;
        if detection.frame.is_some() {
            self.usable_frames[camera_index] += 1;
            self.tags[camera_index] += detection.accepted_tags;
            self.points[camera_index] += detection.points;
        }
    }

    #[must_use]
    pub fn summary(&self, input_views: usize, usable_views: usize) -> DetectionSummary {
        let by_camera = |counts: &[usize]| {
            self.settings
                .cameras
                .iter()
                .enumerate()
                .map(|(index, camera)| (camera.id.clone(), counts[index]))
                .collect()
        };
        DetectionSummary {
            input_views,
            usable_views,
            usable_frames_by_camera: by_camera(&self.usable_frames),
            tags_by_camera: by_camera(&self.tags),
            points_by_camera: by_camera(&self.points),
            ignored_tags: self.ignored_tags,
            refinement_failures: self.refinement_failures,
        }
    }
}

#[derive(Clone)]
struct TargetProperties {
    base_index_by_tag: HashMap<u32, usize>,
    decoded_corner_to_front_corner: [usize; 4],
}

fn target_properties(settings: &RunSettings) -> TargetProperties {
    let TargetConfig::Aprilgrid {
        id_rows_front_view_left_to_right,
        decoded_corner_to_front_corner,
        ..
    } = &settings.target;
    let base_index_by_tag = id_rows_front_view_left_to_right
        .iter()
        .flatten()
        .copied()
        .enumerate()
        .map(|(cell, tag_id)| (tag_id, cell * 4))
        .collect();
    TargetProperties {
        base_index_by_tag,
        decoded_corner_to_front_corner: *decoded_corner_to_front_corner,
    }
}

fn options(settings: &RunSettings) -> apriltag::DetectorOptions {
    let TargetConfig::Aprilgrid {
        family,
        border_bits,
        ..
    } = settings.target;
    let family = match family {
        TagFamilyConfig::Tag16h5 => apriltag::TagFamily::Tag16h5,
        TagFamilyConfig::Tag25h9 => apriltag::TagFamily::Tag25h9,
        TagFamilyConfig::Tag36h10 => apriltag::TagFamily::Tag36h10,
        TagFamilyConfig::Tag36h11 => apriltag::TagFamily::Tag36h11,
    };
    let refinement = match settings.detector.refinement.method {
        RefinementMethodConfig::Disabled => detection::SubpixelRefinement::Disabled,
        RefinementMethodConfig::Saddle => detection::SubpixelRefinement::Enabled {
            method: detection::RefinementMethod::Saddle,
            max_iterations: settings.detector.refinement.max_iterations,
        },
        RefinementMethodConfig::OuterContour => detection::SubpixelRefinement::Enabled {
            method: detection::RefinementMethod::OuterContour,
            max_iterations: settings.detector.refinement.max_iterations,
        },
    };
    apriltag::DetectorOptions {
        family,
        border_bits,
        max_hamming: settings.detector.max_hamming,
        refinement,
        max_pixels: settings.detector.budgets.max_pixels,
        max_clusters: settings.detector.budgets.max_clusters,
        max_candidates: settings.detector.budgets.max_candidates,
    }
}
