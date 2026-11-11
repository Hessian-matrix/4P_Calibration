//! Final solved trajectory serialization.
//!
//! Every constructor copies poses from the final solver result. Control poses are never
//! exported as trajectory samples. The continuous-time path evaluates the solver's own
//! spline at each observed camera time shifted by the solved camera-IMU offset, because
//! that is the time the solver used for the corresponding visual factor.

use anyhow::{Context, Result, ensure};
use ccl_calib::camera::SampleTime;
use ccl_calib::camera::spline::SplineTrajectory;
use ccl_calib::camera::visual_inertial::{InertialState, TimedView};
use ccl_geometry::Pose3;
use serde::{Deserialize, Serialize};

use crate::detection::DetectedView;
use crate::run_record::{RunContext, atomic_write};

pub const SCHEMA_VERSION: u32 = 1;
const VISUAL_FRAME: &str = "board";
const WORLD_FRAME: &str = "world";
const IMU_BODY: &str = "imu";

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrajectorySample {
    /// Camera exposure seconds relative to [`Trajectory::time_origin_ns`].
    pub time_s: f64,
    pub position_m: [f64; 3],
    /// Row-major rotation of `T_frame_from_body`.
    pub rotation: [[f64; 3]; 3],
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Trajectory {
    pub schema_version: u32,
    pub frame: String,
    pub body_frame: String,
    pub time_basis: String,
    /// Decimal nanosecond origin as a string so JavaScript never rounds it.
    pub time_origin_ns: String,
    pub samples: Vec<TrajectorySample>,
}

impl Trajectory {
    /// Final visible-rig trajectory.
    ///
    /// The solver reports `T_reference_camera_from_target` per shared view. The exported
    /// transform is `T_board_from_reference`, i.e. the inverse of that final view pose.
    /// `views` are the same shared views in the same order as the solve request.
    pub fn visual(
        reference_camera: usize,
        body_frame: &str,
        reference_from_target: &[Pose3],
        views: &[DetectedView],
        origin_ns: i64,
    ) -> Result<Self> {
        ensure!(
            !body_frame.is_empty(),
            "trajectory body frame must not be empty"
        );
        ensure!(
            !reference_from_target.is_empty(),
            "solved visual trajectory has no views"
        );
        ensure!(
            reference_from_target.len() == views.len(),
            "final reference-from-board poses and shared views differ: {} poses for {} views",
            reference_from_target.len(),
            views.len()
        );
        let mut samples = Vec::with_capacity(views.len());
        let mut previous_time_s = None;
        for (index, (pose, view)) in reference_from_target.iter().zip(views).enumerate() {
            ensure!(
                view.frames
                    .iter()
                    .any(|frame| frame.camera_index == reference_camera),
                "shared view {index} ({}) has no frame from reference camera {reference_camera} ({body_frame})",
                view.id
            );
            let timestamp_ns = view
                .timestamp_ns
                .with_context(|| format!("camera view {index} has no exposure timestamp"))?;
            let time_s = SampleTime::from_nanoseconds(timestamp_ns, origin_ns)
                .with_context(|| format!("camera view {index} timestamp is not representable"))?
                .seconds();
            ensure_strictly_increasing(&mut previous_time_s, time_s, index)?;
            let board_from_reference = pose.inverse().with_context(|| {
                format!("invert final reference-from-board pose for camera view {index}")
            })?;
            samples.push(sample(time_s, board_from_reference));
        }
        Ok(Self {
            schema_version: SCHEMA_VERSION,
            frame: VISUAL_FRAME.to_owned(),
            body_frame: body_frame.to_owned(),
            time_basis: "shared-view anchor exposure timestamps in seconds relative to time_origin_ns; the loader anchors shared views on the calibration reference/selected camera and every exported view is validated to contain that reference camera. Each sample is T_board_from_reference = inverse(T_reference_from_target) from the final solve at that shared view, with no interpolation".to_owned(),
            time_origin_ns: origin_ns.to_string(),
            samples,
        })
    }

    /// Final preintegration trajectory.
    ///
    /// `states` come from the final visual-inertial result, one per request view, and are
    /// the optimized `world_from_imu` poses at those camera times. The companion views
    /// are the exact same timed views supplied to the solve request.
    pub fn inertial(
        states: &[InertialState],
        views: &[TimedView<'_>],
        time_offset_s: f64,
        origin_ns: i64,
    ) -> Result<Self> {
        ensure!(
            time_offset_s.is_finite(),
            "inertial time offset must be finite"
        );
        ensure!(
            !states.is_empty(),
            "solved inertial trajectory has no states"
        );
        ensure!(
            states.len() == views.len(),
            "final inertial states and timed views differ: {} states for {} views",
            states.len(),
            views.len()
        );
        let mut samples = Vec::with_capacity(states.len());
        let mut previous_time_s = None;
        for (index, (state, view)) in states.iter().zip(views).enumerate() {
            ensure!(
                state.camera_time == view.time,
                "inertial state {index} camera time {:.9}s does not match its view time {:.9}s",
                state.camera_time.seconds(),
                view.time.seconds()
            );
            let expected_imu_time =
                SampleTime::from_seconds(state.camera_time.seconds() + time_offset_s)
                    .context("inertial state shifted time is not representable")?;
            ensure!(
                state.imu_time == expected_imu_time,
                "inertial state {index} imu time {:.9}s does not equal t_camera + time_offset_s ({:.9}s)",
                state.imu_time.seconds(),
                expected_imu_time.seconds()
            );
            let time_s = state.camera_time.seconds();
            ensure_strictly_increasing(&mut previous_time_s, time_s, index)?;
            samples.push(sample(time_s, state.world_from_imu));
        }
        Ok(Self {
            schema_version: SCHEMA_VERSION,
            frame: WORLD_FRAME.to_owned(),
            body_frame: IMU_BODY.to_owned(),
            time_basis: format!(
                "shared-view anchor exposure timestamps in seconds relative to time_origin_ns; each sample is the final optimized T_world_from_imu state at that shared-view time. Corelib inertial evaluation uses t_imu = t_camera + time_offset_s, time_offset_s = {time_offset_s:+.9} s, and every state was checked against that shift"
            ),
            time_origin_ns: origin_ns.to_string(),
            samples,
        })
    }

    /// Final continuous-time nominal trajectory.
    ///
    /// The final spline is sampled on the header plus global affine-clock timeline. Per-camera
    /// residual offset/skew parameters identify visual factors but do not retime this shared
    /// exported trajectory. Samples are actual observation header times, never control knots.
    pub fn continuous(
        trajectory: &SplineTrajectory,
        views: &[TimedView<'_>],
        time_offset_s: f64,
        clock_reference_time_s: f64,
        clock_skew_s_per_s: f64,
        origin_ns: i64,
    ) -> Result<Self> {
        ensure!(
            [time_offset_s, clock_reference_time_s, clock_skew_s_per_s]
                .iter()
                .all(|value| value.is_finite()),
            "continuous-time affine clock parameters must be finite"
        );
        ensure!(
            !views.is_empty(),
            "solved continuous-time trajectory has no observed views"
        );
        let mut samples = Vec::with_capacity(views.len());
        let mut previous_time_s = None;
        for (index, view) in views.iter().enumerate() {
            let time_s = view.time.seconds();
            ensure_strictly_increasing(&mut previous_time_s, time_s, index)?;
            let imu_time_s =
                time_s + time_offset_s + clock_skew_s_per_s * (time_s - clock_reference_time_s);
            ensure!(
                imu_time_s.is_finite(),
                "observation {index} shifted time is not finite"
            );
            let imu_time = SampleTime::from_seconds(imu_time_s).with_context(|| {
                format!("observation {index} shifted time is not representable")
            })?;
            let evaluated = trajectory.evaluate(imu_time).with_context(|| {
                format!(
                    "final continuous-time trajectory does not cover observation {index} at t_imu={imu_time_s:.9}s"
                )
            })?;
            samples.push(sample(time_s, evaluated.world_from_imu));
        }
        Ok(Self {
            schema_version: SCHEMA_VERSION,
            frame: WORLD_FRAME.to_owned(),
            body_frame: IMU_BODY.to_owned(),
            time_basis: format!(
                "exact observation header timestamps in seconds relative to time_origin_ns; each sample uses t_imu = header_stamp + {time_offset_s:+.9} + {clock_skew_s_per_s:+.12} * (header_stamp - {clock_reference_time_s:.9}). Per-camera residual affine terms are intentionally not applied to shared trajectory samples"
            ),
            time_origin_ns: origin_ns.to_string(),
            samples,
        })
    }

    /// Writes `trajectory.json` in the run root before result publication.
    ///
    /// Validation is repeated here so a hand-constructed value can never be published
    /// with non-finite samples, invalid rotations, duplicate times, or no samples.
    pub fn write(&self, run: &RunContext) -> Result<()> {
        self.validate()?;
        let bytes = serde_json::to_vec(self).context("serialize final trajectory")?;
        let path = run.root().join("trajectory.json");
        atomic_write(&path, &bytes).with_context(|| format!("write {}", path.display()))
    }

    fn validate(&self) -> Result<()> {
        ensure!(
            self.schema_version == SCHEMA_VERSION,
            "unsupported trajectory schema version {}",
            self.schema_version
        );
        ensure!(!self.frame.is_empty(), "trajectory frame must not be empty");
        ensure!(
            !self.body_frame.is_empty(),
            "trajectory body frame must not be empty"
        );
        ensure!(
            !self.time_basis.is_empty(),
            "trajectory time basis must not be empty"
        );
        ensure!(
            self.time_origin_ns.parse::<i64>().is_ok(),
            "trajectory time origin must be a decimal nanosecond integer"
        );
        ensure!(
            !self.samples.is_empty(),
            "trajectory must contain at least one sample"
        );
        let mut previous_time_s = None;
        for (index, sample) in self.samples.iter().enumerate() {
            ensure_strictly_increasing(&mut previous_time_s, sample.time_s, index)?;
            Pose3::try_new(sample.rotation, sample.position_m).with_context(|| {
                format!("trajectory sample {index} is not a finite right-handed pose")
            })?;
        }
        Ok(())
    }
}

fn sample(time_s: f64, pose: Pose3) -> TrajectorySample {
    TrajectorySample {
        time_s,
        position_m: pose.translation(),
        rotation: pose.rotation(),
    }
}

fn ensure_strictly_increasing(previous: &mut Option<f64>, time_s: f64, index: usize) -> Result<()> {
    ensure!(
        time_s.is_finite(),
        "trajectory sample {index} time is not finite"
    );
    if let Some(previous) = *previous {
        ensure!(
            time_s > previous,
            "trajectory sample {index} time {time_s:.9}s is not strictly after {previous:.9}s"
        );
    }
    *previous = Some(time_s);
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use ccl_calib::camera::CalibrationImageSize;
    use ccl_calib::camera::SampleTime;
    use ccl_calib::camera::spline::SplineTrajectory;
    use ccl_calib::camera::visual_inertial::TimedView;
    use ccl_geometry::Pose3;

    use crate::detection::{DetectedFrame, DetectedView};

    use super::{SCHEMA_VERSION, Trajectory};

    fn visual_view(id: &str, timestamp_ns: i64) -> DetectedView {
        DetectedView {
            id: id.to_owned(),
            timestamp_ns: Some(timestamp_ns),
            frames: vec![Arc::new(DetectedFrame {
                camera_index: 0,
                image_size: CalibrationImageSize::new(640, 480).unwrap(),
                target_indices: Vec::new(),
                pixels: Vec::new(),
                exposure_duration_s: None,
            })],
        }
    }

    #[test]
    fn visual_trajectory_inverts_the_final_reference_pose() {
        let reference_from_target =
            Pose3::from_rvec_tvec([0.0, 0.0, 0.7], [1.0, -2.0, 0.5]).unwrap();
        let views = [visual_view("view-a", 1_000_000_000)];
        let trajectory =
            Trajectory::visual(0, "camera0", &[reference_from_target], &views, 500_000_000)
                .unwrap();
        assert_eq!(trajectory.schema_version, SCHEMA_VERSION);
        assert_eq!(trajectory.frame, "board");
        assert_eq!(trajectory.body_frame, "camera0");
        assert_eq!(trajectory.time_origin_ns, "500000000");
        assert_eq!(trajectory.samples[0].time_s, 0.5);

        let expected = reference_from_target.inverse().unwrap();
        let exported = trajectory.samples[0];
        for (actual, expected) in exported
            .position_m
            .iter()
            .zip(expected.translation().iter())
        {
            assert!((actual - expected).abs() < 1.0e-12);
        }
        for row in 0..3 {
            for column in 0..3 {
                assert!(
                    (exported.rotation[row][column] - expected.rotation()[row][column]).abs()
                        < 1.0e-12
                );
            }
        }
    }

    #[test]
    fn inertial_trajectory_copies_state_and_rejects_mismatched_views() {
        use ccl_calib::camera::inertial::ImuBias;
        use ccl_calib::camera::visual_inertial::InertialState;

        let camera_time = SampleTime::from_seconds(0.4).unwrap();
        let time_offset_s = -0.002;
        let state = InertialState {
            camera_time,
            imu_time: SampleTime::from_seconds(camera_time.seconds() + time_offset_s).unwrap(),
            world_from_imu: Pose3::from_rvec_tvec([0.0; 3], [1.0, 2.0, 3.0]).unwrap(),
            velocity_world_m_s: [0.0; 3],
            bias: ImuBias::default(),
        };
        let view = TimedView {
            time: camera_time,
            observations: &[],
        };
        let trajectory =
            Trajectory::inertial(std::slice::from_ref(&state), &[view], time_offset_s, 123)
                .unwrap();
        assert_eq!(trajectory.frame, "world");
        assert_eq!(trajectory.body_frame, "imu");
        assert_eq!(trajectory.samples[0].time_s, 0.4);
        assert_eq!(trajectory.samples[0].position_m, [1.0, 2.0, 3.0]);

        let mismatched = TimedView {
            time: SampleTime::from_seconds(0.41).unwrap(),
            observations: &[],
        };
        assert!(
            Trajectory::inertial(
                std::slice::from_ref(&state),
                &[mismatched],
                time_offset_s,
                123
            )
            .is_err()
        );
    }

    #[test]
    fn continuous_trajectory_evaluates_at_affine_clock_time() {
        let start = SampleTime::from_seconds(0.0).unwrap();
        let knot_interval_s = 0.5;
        let velocity_m_s = 2.0;
        let knots = (0..6)
            .map(|index| {
                let nominal_s = (f64::from(index) - 1.0) * knot_interval_s;
                Pose3::from_rvec_tvec([0.0; 3], [velocity_m_s * nominal_s, 0.0, 0.0]).unwrap()
            })
            .collect::<Vec<_>>();
        let spline = SplineTrajectory::new(start, knot_interval_s, knots).unwrap();
        let times = [0.0, 0.75];
        let views = times
            .iter()
            .map(|seconds| TimedView {
                time: SampleTime::from_seconds(*seconds).unwrap(),
                observations: &[],
            })
            .collect::<Vec<_>>();
        let offset_s = 0.25;
        let reference_s = 0.5;
        let skew = 100.0e-6;
        let trajectory =
            Trajectory::continuous(&spline, &views, offset_s, reference_s, skew, -2_000_000_000)
                .unwrap();
        assert_eq!(trajectory.frame, "world");
        assert_eq!(trajectory.body_frame, "imu");
        assert_eq!(trajectory.time_origin_ns, "-2000000000");
        for (sample, camera_time_s) in trajectory.samples.iter().zip(times) {
            assert_eq!(sample.time_s, camera_time_s);
            let imu_time_s = camera_time_s + offset_s + skew * (camera_time_s - reference_s);
            let expected = velocity_m_s * imu_time_s;
            assert!((sample.position_m[0] - expected).abs() < 1.0e-12);
        }
    }
}
