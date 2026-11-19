mod intrinsic_seed;

use std::cell::RefCell;
use std::io::{BufWriter, Write};
use std::path::Path;
use web_time::{Duration, Instant};

use anyhow::{Context, Result, ensure};
use ccl_calib::camera::continuous_time::{
    ContinuousTimeCalibrationRequest, ContinuousTimeCalibrationResult,
    ContinuousTimeInitializationRequest, ContinuousTimeSolverConfig, MotionAlignedVisualWeights,
    calibrate_continuous_time, initialize_continuous_time, supported_camera_interval,
};
use ccl_calib::camera::inertial::{
    ImuBias, ImuNoise, ImuSequence, IntegrationConfig as CoreIntegrationConfig,
    IntegrationMethod as CoreIntegrationMethod,
};
use ccl_calib::camera::rig::{
    CameraRigCalibrationRequest, CameraRigCalibrationResult, RigInitialization, RigSolverConfig,
    SharedView,
};
use ccl_calib::camera::visual_inertial::{
    InertialInitialization, TimeOffset, TimedView, VisualInertialCalibrationRequest,
    VisualInertialCalibrationResult, VisualInertialSolverConfig, calibrate_visual_inertial,
};
use ccl_calib::camera::{
    CalibrationInformation, CameraObservation, CameraParameters, JointConvergence,
    JointSolveProgress, JointSolverConfig, PointCorrespondences, SampleTime, SolveActivity,
    SolveControl, TargetGeometry,
};
use ccl_geometry::Pose3;
use serde::Serialize;

use crate::config::{
    CalibrationConfig, CameraTimestampReference, InertialMethod, InitialIntrinsicsConfig,
    RunSettings, TargetConfig, TimeOffsetConfig,
};
use crate::detection::{DetectedDataset, DetectedView};
use crate::export::{
    AcceptanceStatus, CalibrationOutput, CameraOutput, CameraTimeOffsetOutput,
    ImuAxisCalibrationOutput, InertialOutput, NamedPose, PoseOutput, RESULT_SCHEMA_VERSION,
    publish,
};
#[cfg(not(target_arch = "wasm32"))]
use crate::export::load_result;
use crate::input;
use crate::run_record::RunContext;
use crate::trajectory::Trajectory;

#[derive(Clone, Debug, Serialize)]
pub struct CameraSolveSummary {
    pub camera_index: usize,
    pub used_points: usize,
    pub rejected_points: usize,
    pub rms_px: f64,
    pub p95_px: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct ParameterStandardDeviation {
    pub parameter: String,
    pub standard_deviation: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct SolveSummary {
    pub converged: bool,
    pub termination: String,
    pub iterations: usize,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub final_damping: f64,
    pub scaled_gradient_inf_norm: f64,
    pub accepted_steps: usize,
    pub rejected_steps: usize,
    pub information_rank: usize,
    pub information_parameters: usize,
    pub condition_number: f64,
    pub covariance_is_pseudoinverse: bool,
    pub camera_errors: Vec<CameraSolveSummary>,
    /// Empirical RMS in nominal whitened units; diagnostic variance scales are its square.
    pub visual_whitened_rms: Vec<f64>,
    pub gyro_whitened_rms: Option<f64>,
    pub accelerometer_whitened_rms: Option<f64>,
    pub inertial_whitened_rms: Option<f64>,
    /// Local-curvature marginal standard deviations in each optimization coordinate's unit.
    pub parameter_standard_deviations: Vec<ParameterStandardDeviation>,
}

#[derive(Clone, Debug, Serialize)]
struct ExposureDiagnostics {
    schema_version: u32,
    projection_model: &'static str,
    cameras: Vec<CameraExposureDiagnostics>,
}

#[derive(Clone, Debug, Serialize)]
struct CameraExposureDiagnostics {
    camera_index: usize,
    camera_id: String,
    timestamp_reference: CameraTimestampReference,
    fixed_exposure_duration_s: Option<f64>,
    exposure_metadata_topic: Option<String>,
    exposure_duration_s_range: Option<[f64; 2]>,
    status: &'static str,
    used_points: usize,
    rejected_points: usize,
    unsupported_points: usize,
    bins: Vec<ExposureBinSummary>,
    anisotropy_cross_validation: Option<AnisotropyCrossValidation>,
}

#[derive(Clone, Debug, Serialize)]
struct ExposureBinSummary {
    predicted_motion_px: &'static str,
    points: usize,
    mean_predicted_motion_px: f64,
    p95_predicted_motion_px: f64,
    rms_reprojection_px: f64,
    p95_reprojection_px: f64,
    rms_geometric_average_reprojection_px: f64,
    p95_geometric_average_reprojection_px: f64,
    mean_squared_error_delta_px2: f64,
    mean_geometric_midpoint_bias_px: f64,
    p95_geometric_midpoint_bias_px: f64,
    directional_points: usize,
    mean_parallel_residual_px: Option<f64>,
    rms_parallel_residual_px: Option<f64>,
    rms_perpendicular_residual_px: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct AnisotropyCrossValidation {
    temporal_block_s: f64,
    eligible_points: usize,
    folds: Vec<AnisotropyFoldSummary>,
}

#[derive(Clone, Debug, Serialize)]
struct AnisotropyFoldSummary {
    held_out_blocks: &'static str,
    training_points: usize,
    held_out_points: usize,
    fitted_isotropic_sigma_px: f64,
    fitted_perpendicular_sigma_px: f64,
    fitted_parallel_variance_per_motion_squared: f64,
    held_out_isotropic_mean_nll: f64,
    held_out_anisotropic_mean_nll: f64,
    held_out_mean_nll_delta: f64,
    held_out_isotropic_whitened_rms: f64,
    held_out_anisotropic_whitened_rms: f64,
    held_out_volume_preserving_mean_nll: f64,
    held_out_volume_preserving_mean_nll_delta: f64,
    held_out_volume_preserving_whitened_rms: f64,
}

#[derive(Clone, Copy)]
struct AnisotropySample {
    fold: usize,
    motion_squared: f64,
    parallel_squared: f64,
    perpendicular_squared: f64,
}

#[derive(Default)]
struct ExposureBinAccumulator {
    blur: Vec<f64>,
    error: Vec<f64>,
    geometric_average_error: Vec<f64>,
    bias: Vec<f64>,
    parallel_sum: f64,
    parallel_squared_sum: f64,
    perpendicular_squared_sum: f64,
    directional_points: usize,
}

#[derive(Default)]
struct CameraExposureAccumulator {
    used_points: usize,
    rejected_points: usize,
    unsupported_points: usize,
    bins: [ExposureBinAccumulator; 5],
    minimum_duration_s: Option<f64>,
    maximum_duration_s: Option<f64>,
    anisotropy_samples: Vec<AnisotropySample>,
}
#[derive(Clone, Debug)]
pub enum Event {
    Stage(&'static str),
    DetectionProgress(crate::input::DetectionProgress),
    Dataset(crate::detection::DetectionSummary),
    SolveProgress(JointSolveProgress),
    SolveActivity(SolveActivity),
    SolveSummary(SolveSummary),
    Message(String),
}

pub fn execute(
    config: &RunSettings,
    run: &RunContext,
    cancelled: &(dyn Fn() -> bool + Sync),
    emit: &mut dyn FnMut(Event),
) -> Result<CalibrationOutput> {
    run.set_mode(config.mode_name())?;
    run.write_yaml(Path::new("config.yaml"), config)?;

    emit(Event::Stage("scan_detect"));
    let started = Instant::now();
    let mut progress_writer = BufWriter::new(
        run.create_writer(Path::new("detection_progress.jsonl"))?,
    );
    let mut progress_write_error = None;
    let mut last_flush = Instant::now();
    let mut announced_workers = false;
    let detection_result = {
        let mut detection_event = |mut progress: input::DetectionProgress| {
            if !announced_workers {
                let message = format!("detection workers={}", progress.workers);
                let _ = run.log("INFO", &message);
                emit(Event::Message(message));
                announced_workers = true;
            }
            // Preview pixels are live browser telemetry, not persistent numeric diagnostics.
            // Move them aside while writing instead of cloning the bounded image buffer.
            let preview = progress.preview.take();
            if progress_write_error.is_none() {
                let write_result = (|| -> Result<()> {
                    serde_json::to_writer(&mut progress_writer, &progress)?;
                    writeln!(progress_writer)?;
                    Ok(())
                })();
                if let Err(error) = write_result {
                    progress_write_error = Some(error);
                }
            }
            progress.preview = preview;
            if last_flush.elapsed() >= Duration::from_millis(250) {
                if progress_write_error.is_none()
                    && let Err(error) = progress_writer.flush()
                {
                    progress_write_error = Some(error.into());
                }
                last_flush = Instant::now();
            }
            emit(Event::DetectionProgress(progress));
        };
        input::load_and_detect(config, cancelled, &mut detection_event)
    };
    progress_writer
        .flush()
        .context("flush detection progress")?;
    if let Some(error) = progress_write_error {
        return Err(error).context("write detection_progress.jsonl");
    }
    run.record_stage_duration("scan_detect", elapsed_millis(started))?;
    let (detected, input_summary, detection_summary) = detection_result?;
    run.write_yaml(Path::new("input.yaml"), &input_summary)?;
    log_json(run, "input_summary", &input_summary)?;
    run.write_yaml(Path::new("detection.yaml"), &detection_summary)?;
    emit(Event::Dataset(detection_summary.clone()));
    emit(Event::Message(format!(
        "loaded {} anchor-camera views, {} usable shared views, {} exact-time views",
        input_summary.view_count,
        detection_summary.usable_views,
        detected.views.len()
    )));

    ensure!(!cancelled(), "calibration cancelled");
    emit(Event::Stage("solve"));
    let started = Instant::now();
    let solve_result = {
        let emit_cell = RefCell::new(&mut *emit);
        let progress = |snapshot| {
            (emit_cell.borrow_mut())(Event::SolveProgress(snapshot));
        };
        let stage = |name| {
            (emit_cell.borrow_mut())(Event::Stage(name));
        };
        let activity = |snapshot| {
            (emit_cell.borrow_mut())(Event::SolveActivity(snapshot));
        };
        let control = SolveControl {
            cancelled: Some(cancelled),
            joint_progress: Some(&progress),
            activity: Some(&activity),
            ..SolveControl::default()
        };
        solve(config, &detected, &control, &stage, run)
    };
    run.record_stage_duration("solve", elapsed_millis(started))?;
    let (output, summary) = solve_result?;
    run.write_yaml(Path::new("solve.yaml"), &summary)?;
    emit(Event::SolveSummary(summary.clone()));

    ensure!(!cancelled(), "calibration cancelled");
    emit(Event::Stage("export"));
    let started = Instant::now();
    publish(run, &output, &summary)?;
    run.record_stage_duration("export", elapsed_millis(started))?;
    run.log("INFO", "published result/calibration.yaml")?;
    Ok(output)
}

fn solve(
    config: &RunSettings,
    detected: &DetectedDataset,
    control: &SolveControl<'_>,
    stage: &dyn Fn(&'static str),
    run: &RunContext,
) -> Result<(CalibrationOutput, SolveSummary)> {
    let target = target_geometry(config)?;
    match &config.calibration {
        CalibrationConfig::SingleCamera {
            camera,
            initial_intrinsics,
        } => solve_single(
            config,
            detected,
            &target,
            camera,
            initial_intrinsics,
            control,
            run,
        ),
        CalibrationConfig::MultiCamera {
            reference_camera,
            refine_intrinsics,
            initial_intrinsics,
            max_closure_rotation_deg,
            max_closure_translation_m,
        } => solve_multi(
            config,
            detected,
            &target,
            reference_camera,
            *refine_intrinsics,
            initial_intrinsics,
            *max_closure_rotation_deg,
            *max_closure_translation_m,
            control,
            run,
        ),
        CalibrationConfig::MultiCameraImu {
            method,
            visual_reference_camera,
            initial_intrinsics,
            time_offset,
            imu,
        } => solve_inertial(
            config,
            detected,
            &target,
            visual_reference_camera,
            initial_intrinsics,
            *method,
            *time_offset,
            imu,
            control,
            stage,
            run,
        ),
    }
}

#[allow(
    clippy::too_many_arguments,
    reason = "single-camera solve keeps cancellation, progress, and run recording explicit"
)]
fn solve_single(
    config: &RunSettings,
    detected: &DetectedDataset,
    target: &TargetGeometry,
    camera_id: &str,
    initial: &InitialIntrinsicsConfig,
    control: &SolveControl<'_>,
    run: &RunContext,
) -> Result<(CalibrationOutput, SolveSummary)> {
    let source_index = camera_index(config, camera_id)?;
    let mut cameras = initial_cameras(config, detected, &[source_index], initial)?;
    let observation_groups =
        build_observations(target, &detected.synchronized_views, Some(source_index))?;
    let shared = shared_views(&observation_groups);
    let mut solver = joint_solver(config);
    solver.pixel_sigmas = vec![config.cameras[source_index].pixel_sigma];
    if initial.path.is_none() {
        cameras[0] = bootstrap_camera(cameras[0], camera_id, &shared, &solver, control)?;
    }
    let request = CameraRigCalibrationRequest {
        cameras: &cameras,
        views: &shared,
        reference_camera: 0,
        initialization: RigInitialization::Planar,
        refine_intrinsics: true,
    };
    let result = ccl_calib::camera::rig::calibrate_camera_rig(
        &request,
        RigSolverConfig {
            solver,
            ..RigSolverConfig::default()
        },
        control,
    )
    .with_context(|| {
        format!(
            "single-camera solve failed for {} ({:?}); verify the selected camera model and angular coverage",
            config.cameras[source_index].id, config.cameras[source_index].model
        )
    })?;
    log_rig_diagnostics(run, &result)?;
    let summary = rig_solve_summary(&result, config);
    let trajectory = Trajectory::visual(
        source_index,
        camera_id,
        &result.reference_from_target,
        &detected.synchronized_views,
        detected
            .time_origin_ns
            .context("missing trajectory time origin")?,
    )?;
    trajectory.write(run)?;
    Ok((
        CalibrationOutput {
            schema_version: RESULT_SCHEMA_VERSION,
            mode: "single_camera".to_owned(),
            acceptance: AcceptanceStatus::PendingHumanReview,
            cameras: vec![camera_output(
                camera_id,
                detected.image_sizes[source_index],
                result.cameras[0],
            )],
            reference_camera: None,
            camera_from_reference: Vec::new(),
            inertial: None,
        },
        summary,
    ))
}

#[allow(clippy::too_many_arguments)]
fn solve_multi(
    config: &RunSettings,
    detected: &DetectedDataset,
    target: &TargetGeometry,
    reference_camera: &str,
    refine_intrinsics: bool,
    initial: &InitialIntrinsicsConfig,
    max_closure_rotation_deg: f64,
    max_closure_translation_m: f64,
    control: &SolveControl<'_>,
    run: &RunContext,
) -> Result<(CalibrationOutput, SolveSummary)> {
    let indices = (0..config.cameras.len()).collect::<Vec<_>>();
    let mut cameras = initial_cameras(config, detected, &indices, initial)?;
    let observations = build_observations(target, &detected.synchronized_views, None)?;
    if initial.path.is_none() {
        cameras = calibrate_automatic_intrinsics(
            config,
            &cameras,
            &observations.iter().map(Vec::as_slice),
            control,
            run,
        )?;
    }
    let shared = shared_views(&observations);
    let reference_index = camera_index(config, reference_camera)?;
    let result = ccl_calib::camera::rig::calibrate_camera_rig(
        &CameraRigCalibrationRequest {
            cameras: &cameras,
            views: &shared,
            reference_camera: reference_index,
            initialization: RigInitialization::Planar,
            refine_intrinsics,
        },
        RigSolverConfig {
            solver: joint_solver(config),
            max_closure_rotation_rad: max_closure_rotation_deg.to_radians(),
            max_closure_translation_m,
        },
        control,
    )
    .with_context(|| {
        format!(
            "multi-camera solve failed for models {}; verify each topic/model mapping and angular coverage",
            camera_model_summary(config)
        )
    })?;
    log_rig_diagnostics(run, &result)?;
    let summary = rig_solve_summary(&result, config);
    let trajectory = Trajectory::visual(
        reference_index,
        reference_camera,
        &result.reference_from_target,
        &detected.synchronized_views,
        detected
            .time_origin_ns
            .context("missing trajectory time origin")?,
    )?;
    trajectory.write(run)?;
    let cameras = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| {
            camera_output(
                &camera.id,
                detected.image_sizes[index],
                result.cameras[index],
            )
        })
        .collect();
    let transforms = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| NamedPose {
            direction: format!("T_{}_from_{}", camera.id, reference_camera),
            pose: PoseOutput::from_pose(result.camera_from_reference[index]),
        })
        .collect();
    Ok((
        CalibrationOutput {
            schema_version: RESULT_SCHEMA_VERSION,
            mode: "multi_camera".to_owned(),
            acceptance: AcceptanceStatus::PendingHumanReview,
            cameras,
            reference_camera: Some(reference_camera.to_owned()),
            camera_from_reference: transforms,
            inertial: None,
        },
        summary,
    ))
}

#[allow(
    clippy::too_many_arguments,
    reason = "the inertial solve keeps cancellation, progress, and run recording explicit"
)]
fn solve_inertial(
    config: &RunSettings,
    detected: &DetectedDataset,
    target: &TargetGeometry,
    reference_camera: &str,
    initial: &InitialIntrinsicsConfig,
    method: InertialMethod,
    time_offset_config: TimeOffsetConfig,
    imu_config: &crate::config::ImuConfig,
    control: &SolveControl<'_>,
    stage: &dyn Fn(&'static str),
    run: &RunContext,
) -> Result<(CalibrationOutput, SolveSummary)> {
    let indices = (0..config.cameras.len()).collect::<Vec<_>>();
    let mut cameras = initial_cameras(config, detected, &indices, initial)?;
    let reference_index = camera_index(config, reference_camera)?;
    let continuous_time = matches!(method, InertialMethod::ContinuousTime);
    let synchronized_observations = build_observations(target, &detected.synchronized_views, None)?;
    // Approximate synchronization exists only for the visual rig seed (and for the
    // preintegration path, whose shared views are inherently simultaneous). Continuous-time
    // intrinsic seeding and the joint solve consume exact per-camera observations within fixed IMU support.
    let exact_observations = continuous_time
        .then(|| build_observations(target, &detected.views, None))
        .transpose()?;
    if initial.path.is_none() && !continuous_time {
        stage("solve_intrinsics");
        cameras = calibrate_automatic_intrinsics(
            config,
            &cameras,
            &synchronized_observations.iter().map(Vec::as_slice),
            control,
            run,
        )?;
    }
    run.log(
        "INFO",
        if initial.path.is_some() {
            "using caller-supplied intrinsics as the seed; the visual rig refines them before the inertial stage freezes them"
        } else {
            "using automatic intrinsic initialization before visual rig refinement; continuous-time seeding uses only the cropped camera support"
        },
    )?;
    let origin = detected
        .time_origin_ns
        .context("missing inertial time origin")?;
    let seed_views = timed_views(
        origin,
        &detected.synchronized_views,
        &synchronized_observations,
    )?;
    let samples = detected
        .imu_samples
        .as_ref()
        .context("missing IMU samples")?;
    let imu = ImuSequence::from_samples(samples)?;
    if continuous_time {
        // The continuous-time solve consumes exact per-camera timestamps; the seed views
        // above stay approximately synchronized on the reference camera's own timeline.
        let exact_observations = exact_observations.context("missing exact-time observations")?;
        let exact_views = timed_views(origin, &detected.views, &exact_observations)?;
        return solve_continuous_inertial(
            config,
            detected,
            &cameras,
            reference_index,
            &exact_views,
            &seed_views,
            &imu,
            imu_config,
            time_offset_config,
            control,
            stage,
            run,
        );
    }
    stage("solve_preintegration");
    let stage_started = Instant::now();
    let visual = calibrate_visual_inertial(
        &VisualInertialCalibrationRequest {
            cameras: &cameras,
            views: &seed_views,
            imu: &imu,
            initialization: InertialInitialization::PlanarRigGyro {
                reference_camera: reference_index,
                reference_camera_from_imu: Pose3::identity(),
                bias: ImuBias::default(),
            },
            time_offset: core_time_offset(time_offset_config, None),
            refine_intrinsics: true,
        },
        VisualInertialSolverConfig {
            solver: joint_solver(config),
            imu_noise: ImuNoise {
                gyro_density_rad_s_sqrt_hz: isotropic_density(
                    imu_config.noise.gyro_covariance_rad2_s_hz,
                    "gyro",
                )?,
                accelerometer_density_m_s2_sqrt_hz: isotropic_density(
                    imu_config.noise.accelerometer_covariance_m2_s4_hz,
                    "accelerometer",
                )?,
                gyro_bias_walk_rad_s2_sqrt_hz: imu_config.noise.gyro_bias_walk_rad_s2_sqrt_hz,
                accelerometer_bias_walk_m_s3_sqrt_hz: imu_config
                    .noise
                    .accelerometer_bias_walk_m_s3_sqrt_hz,
            },
            integration: CoreIntegrationConfig {
                method: CoreIntegrationMethod::Midpoint,
                max_gap_s: imu_config.integration.max_gap_s,
                max_samples: imu_config.integration.max_samples,
            },
            gravity_magnitude_m_s2: imu_config.gravity_magnitude_m_s2,
        },
        control,
    )
    .context("preintegration initializer failed")?;
    run.record_stage_duration("solve_preintegration", elapsed_millis(stage_started))?;
    log_inertial_diagnostics(run, &visual)?;
    let summary = inertial_solve_summary(&visual, config);
    let trajectory =
        Trajectory::inertial(&visual.states, &seed_views, visual.time_offset_s, origin)?;
    trajectory.write(run)?;
    Ok((inertial_output(config, detected, &visual), summary))
}

/// Continuous-time calibration. The direct visual/gyro seed and its quartic spline use
/// `seed_views`, whose times are the reference camera's own timeline; the baseline and
/// refinement solves consume `exact_views`, one per distinct source timestamp, so every
/// camera keeps its own exposure time.
#[allow(
    clippy::too_many_arguments,
    reason = "the staged solve keeps inputs, cancellation, progress, and recording explicit"
)]
fn solve_continuous_inertial(
    config: &RunSettings,
    detected: &DetectedDataset,
    cameras: &[CameraParameters],
    reference_camera: usize,
    exact_views: &[TimedView<'_>],
    seed_views: &[TimedView<'_>],
    imu: &ImuSequence<'_>,
    imu_config: &crate::config::ImuConfig,
    time_offset_config: TimeOffsetConfig,
    control: &SolveControl<'_>,
    stage: &dyn Fn(&'static str),
    run: &RunContext,
) -> Result<(CalibrationOutput, SolveSummary)> {
    // Fix one observation set for the complete offset and affine-skew envelope; no trial value
    // may add or remove factors. The input midpoint centers intercept and slope.
    let camera_offset_config = imu_config.continuous_time.camera_time_offsets;
    let clock_skew_config = imu_config.continuous_time.clock_skews;
    let input_start_s = exact_views
        .first()
        .context("missing exact views")?
        .time
        .seconds();
    let input_end_s = exact_views
        .last()
        .context("missing exact views")?
        .time
        .seconds();
    let clock_reference_time_s = 0.5 * (input_start_s + input_end_s);
    let maximum_centered_time_s = (input_start_s - clock_reference_time_s)
        .abs()
        .max((input_end_s - clock_reference_time_s).abs());
    let maximum_skew_s_per_s = clock_skew_config
        .minimum_ppm
        .abs()
        .max(clock_skew_config.maximum_ppm.abs())
        * 1.0e-6;
    let skew_components = usize::from(clock_skew_config.refine_global)
        + usize::from(clock_skew_config.refine_cameras);
    let drift_margin_s = maximum_centered_time_s * maximum_skew_s_per_s * skew_components as f64;
    let (camera_start_s, camera_end_s) = supported_camera_interval(
        imu,
        summed_time_offset_bounds(time_offset_config, camera_offset_config, drift_margin_s),
        imu_config.integration.max_gap_s,
    )?;
    let retained = |views: &[TimedView<'_>]| {
        views.partition_point(|v| v.time.seconds() < camera_start_s)
            ..views.partition_point(|v| v.time.seconds() <= camera_end_s)
    };
    ensure!(
        !exact_views.windows(2).any(|w| w[0].time >= w[1].time)
            && !seed_views.windows(2).any(|w| w[0].time >= w[1].time),
        "continuous-time observations must be strictly time ordered"
    );
    let exact_range = retained(exact_views);
    let seed_range = retained(seed_views);
    let camera_counts = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| {
            let count = |views: &[TimedView<'_>]| {
                let mut frames = 0usize;
                let mut points = 0usize;
                for view in views {
                    for observation in view.observations {
                        if observation.camera_index == index {
                            frames += 1;
                            points += observation.points.len();
                        }
                    }
                }
                (frames, points)
            };
            let (input_frames, input_points) = count(exact_views);
            let (used_frames, used_points) = count(&exact_views[exact_range.clone()]);
            serde_json::json!({"camera": camera.id, "input_frames": input_frames,
            "input_points": input_points, "used_frames": used_frames, "used_points": used_points,
            "cropped_frames": input_frames-used_frames, "cropped_points": input_points-used_points})
        })
        .collect::<Vec<_>>();
    run.write_yaml(
        Path::new("time_support.yaml"),
        &serde_json::json!({
            "camera_start_s": camera_start_s, "camera_end_s": camera_end_s,
            "clock_reference_time_s": clock_reference_time_s,
            "clock_skew_bounds_ppm": [clock_skew_config.minimum_ppm, clock_skew_config.maximum_ppm],
            "drift_margin_s": drift_margin_s,
            "time_origin_ns": detected.time_origin_ns.map(|t| t.to_string()),
            "imu_samples": imu.samples().len(), "cameras": camera_counts,
            "input_seed_views": seed_views.len(), "used_seed_views": seed_range.len(),
            "optimizer": "basalt"
        }),
    )?;
    run.log("INFO", &format!(
        "continuous_time support=[{camera_start_s:.9},{camera_end_s:.9}] exact_views={}/{} seed_views={}/{} optimizer=basalt",
        exact_range.len(), exact_views.len(), seed_range.len(), seed_views.len()
    ))?;
    let exact_detected_views = &detected.views[exact_range.clone()];
    let exact_views = &exact_views[exact_range];
    let seed_views = &seed_views[seed_range];
    ensure!(
        exact_views.len() >= 5 && seed_views.len() >= 5,
        "insufficient camera views after complete time-offset support cropping"
    );
    let automatic_intrinsics = matches!(&config.calibration,
        CalibrationConfig::MultiCameraImu { initial_intrinsics, .. } if initial_intrinsics.path.is_none());
    let calibrated_cameras = if automatic_intrinsics {
        stage("solve_intrinsics");
        Some(calibrate_automatic_intrinsics(
            config,
            cameras,
            &exact_views.iter().map(|view| view.observations),
            control,
            run,
        )?)
    } else {
        None
    };
    let cameras = calibrated_cameras.as_deref().unwrap_or(cameras);
    let initial_offset_s = match time_offset_config {
        TimeOffsetConfig::Fixed { value_s } => value_s,
        TimeOffsetConfig::Estimate { initial_s, .. } => initial_s,
    };
    let solver_config = continuous_solver_config(config, imu_config, imu)?;
    let initial_imu_calibration = imu_config.axis_calibration.core();
    let initial_camera_time_offsets_s = vec![0.0; cameras.len()];
    let initial_camera_clock_skews_s_per_s = vec![0.0; cameras.len()];
    stage("solve_visual_inertial_seed");
    let stage_started = Instant::now();
    let seed = initialize_continuous_time(
        &ContinuousTimeInitializationRequest {
            cameras,
            views: exact_views,
            seed_views,
            imu,
            reference_camera,
            // All retained exposures are supported for the full offset range. The
            // initializer covers every IMU sample and uses the initial offset for alignment.
            time_offset: core_time_offset(time_offset_config, None),
            knot_interval_s: imu_config.continuous_time.knot_interval_s,
            max_control_poses: imu_config.integration.max_samples,
            initial_imu_calibration,
        },
        ContinuousTimeSolverConfig {
            solver: joint_solver(config),
            ..solver_config
        },
        control,
    )
    .context("direct visual/gyro continuous-time initialization failed")?;
    run.record_stage_duration("solve_visual_inertial_seed", elapsed_millis(stage_started))?;
    stage("solve_continuous_time");
    let stage_started = Instant::now();
    let baseline = calibrate_continuous_time(
        &ContinuousTimeCalibrationRequest {
            cameras: &seed.cameras,
            views: exact_views,
            imu,
            motion_aligned_visual_weights: None,
            initial_trajectory: &seed.trajectory,
            initial_camera_from_imu: &seed.camera_from_imu,
            initial_bias: seed.bias,
            initial_gravity_world_m_s2: seed.gravity_world_m_s2,
            time_offset: TimeOffset::Fixed(initial_offset_s),
            clock_reference_time_s,
            initial_clock_skew_s_per_s: 0.0,
            clock_skew_bounds_s_per_s: None,
            initial_imu_calibration,
            initial_camera_time_offsets_s: &initial_camera_time_offsets_s,
            camera_time_offset_reference: reference_camera,
            camera_time_offset_bounds_s: None,
            initial_camera_clock_skews_s_per_s: &initial_camera_clock_skews_s_per_s,
            camera_clock_skew_bounds_s_per_s: None,
            refine_intrinsics: false,
        },
        solver_config.clone(),
        control,
    )
    .context("continuous-time baseline solve failed")?;
    run.record_stage_duration("solve_continuous_time", elapsed_millis(stage_started))?;

    let refine_axes = imu_config.axis_calibration.refine;
    let refine_time_offset = matches!(time_offset_config, TimeOffsetConfig::Estimate { .. });
    let refine_camera_time_offsets = camera_offset_config.refine;
    let refine_clock_skews = clock_skew_config.refine_global || clock_skew_config.refine_cameras;
    let refined =
        if refine_axes || refine_time_offset || refine_camera_time_offsets || refine_clock_skews {
            log_continuous_time_diagnostics(run, &baseline)?;
            let refinement_stage = "solve_refine_continuous_time_parameters";
            stage(refinement_stage);
            let stage_started = Instant::now();
            let refined = calibrate_continuous_time(
                &ContinuousTimeCalibrationRequest {
                    cameras: &baseline.cameras,
                    views: exact_views,
                    imu,
                    motion_aligned_visual_weights: None,
                    initial_trajectory: &baseline.trajectory,
                    initial_camera_from_imu: &baseline.camera_from_imu,
                    initial_bias: baseline.bias,
                    initial_gravity_world_m_s2: baseline.gravity_world_m_s2,
                    time_offset: core_time_offset(time_offset_config, Some(baseline.time_offset_s)),
                    clock_reference_time_s,
                    initial_clock_skew_s_per_s: baseline.clock_skew_s_per_s,
                    clock_skew_bounds_s_per_s: clock_skew_config.refine_global.then_some((
                        clock_skew_config.minimum_ppm * 1.0e-6,
                        clock_skew_config.maximum_ppm * 1.0e-6,
                    )),
                    initial_imu_calibration: baseline.imu_calibration,
                    initial_camera_time_offsets_s: &baseline.camera_time_offsets_s,
                    camera_time_offset_reference: reference_camera,
                    camera_time_offset_bounds_s: refine_camera_time_offsets.then_some((
                        camera_offset_config.minimum_s,
                        camera_offset_config.maximum_s,
                    )),
                    initial_camera_clock_skews_s_per_s: &baseline.camera_clock_skews_s_per_s,
                    camera_clock_skew_bounds_s_per_s: clock_skew_config.refine_cameras.then_some((
                        clock_skew_config.minimum_ppm * 1.0e-6,
                        clock_skew_config.maximum_ppm * 1.0e-6,
                    )),
                    refine_intrinsics: false,
                },
                ContinuousTimeSolverConfig {
                    refine_imu_calibration: refine_axes,
                    ..solver_config.clone()
                },
                control,
            )
            .context("continuous-time time-offset/IMU-axis refinement failed")?;
            run.record_stage_duration(refinement_stage, elapsed_millis(stage_started))?;
            refined
        } else {
            baseline
        };
    let anisotropy_config = imu_config.continuous_time.visual_motion_anisotropy;
    let result = if anisotropy_config.enabled {
        log_continuous_time_diagnostics(run, &refined)?;
        let projected_motion_px =
            exposure_motion_vectors(&refined, exact_views, exact_detected_views)?;
        let refinement_stage = "solve_refine_visual_motion_anisotropy";
        stage(refinement_stage);
        let stage_started = Instant::now();
        let anisotropic = calibrate_continuous_time(
            &ContinuousTimeCalibrationRequest {
                cameras: &refined.cameras,
                views: exact_views,
                imu,
                motion_aligned_visual_weights: Some(MotionAlignedVisualWeights {
                    projected_motion_px: &projected_motion_px,
                    variance_ratio_per_motion_squared: anisotropy_config
                        .variance_ratio_per_motion_squared,
                }),
                initial_trajectory: &refined.trajectory,
                initial_camera_from_imu: &refined.camera_from_imu,
                initial_bias: refined.bias,
                initial_gravity_world_m_s2: refined.gravity_world_m_s2,
                time_offset: core_time_offset(time_offset_config, Some(refined.time_offset_s)),
                clock_reference_time_s,
                initial_clock_skew_s_per_s: refined.clock_skew_s_per_s,
                clock_skew_bounds_s_per_s: clock_skew_config.refine_global.then_some((
                    clock_skew_config.minimum_ppm * 1.0e-6,
                    clock_skew_config.maximum_ppm * 1.0e-6,
                )),
                initial_imu_calibration: refined.imu_calibration,
                initial_camera_time_offsets_s: &refined.camera_time_offsets_s,
                camera_time_offset_reference: reference_camera,
                camera_time_offset_bounds_s: refine_camera_time_offsets.then_some((
                    camera_offset_config.minimum_s,
                    camera_offset_config.maximum_s,
                )),
                initial_camera_clock_skews_s_per_s: &refined.camera_clock_skews_s_per_s,
                camera_clock_skew_bounds_s_per_s: clock_skew_config.refine_cameras.then_some((
                    clock_skew_config.minimum_ppm * 1.0e-6,
                    clock_skew_config.maximum_ppm * 1.0e-6,
                )),
                refine_intrinsics: false,
            },
            ContinuousTimeSolverConfig {
                refine_imu_calibration: refine_axes,
                ..solver_config
            },
            control,
        )
        .context("continuous-time visual motion anisotropy refinement failed")?;
        run.record_stage_duration(refinement_stage, elapsed_millis(stage_started))?;
        anisotropic
    } else {
        refined
    };
    log_continuous_time_diagnostics(run, &result)?;
    write_exposure_diagnostics(config, &result, exact_views, exact_detected_views, run)?;
    let summary = continuous_time_solve_summary(&result, config);
    let trajectory = Trajectory::continuous(
        &result.trajectory,
        exact_views,
        result.time_offset_s,
        result.clock_reference_time_s,
        result.clock_skew_s_per_s,
        detected
            .time_origin_ns
            .context("missing trajectory time origin")?,
    )?;
    trajectory.write(run)?;
    Ok((continuous_time_output(config, detected, &result), summary))
}

fn exposure_motion_vectors(
    result: &ContinuousTimeCalibrationResult,
    views: &[TimedView<'_>],
    detected_views: &[DetectedView],
) -> Result<Vec<Vec<Vec<[f64; 2]>>>> {
    ensure!(
        views.len() == detected_views.len(),
        "exposure motion view count mismatch"
    );
    views
        .iter()
        .zip(detected_views)
        .map(|(view, detected_view)| {
            view.observations
                .iter()
                .map(|observation| {
                    let duration_s = detected_view
                        .frames
                        .iter()
                        .find(|frame| frame.camera_index == observation.camera_index)
                        .and_then(|frame| frame.exposure_duration_s)
                        .context(
                            "visual motion anisotropy requires exposure duration for every frame",
                        )?;
                    (0..observation.points.len())
                        .map(|point| {
                            let object = observation
                                .points
                                .object_point(point)
                                .context("visual motion anisotropy object point is missing")?;
                            let start = exposure_projection(
                                result,
                                observation.camera_index,
                                view.time.seconds() - 0.5 * duration_s,
                                object,
                            )?;
                            let end = exposure_projection(
                                result,
                                observation.camera_index,
                                view.time.seconds() + 0.5 * duration_s,
                                object,
                            )?;
                            Ok([end[0] - start[0], end[1] - start[1]])
                        })
                        .collect()
                })
                .collect()
        })
        .collect()
}

#[allow(clippy::too_many_lines)]
fn write_exposure_diagnostics(
    config: &RunSettings,
    result: &ContinuousTimeCalibrationResult,
    views: &[TimedView<'_>],
    detected_views: &[DetectedView],
    run: &RunContext,
) -> Result<()> {
    let mut accumulators = (0..config.cameras.len())
        .map(|_| CameraExposureAccumulator::default())
        .collect::<Vec<_>>();
    for residual in &result.residuals {
        let accumulator = accumulators
            .get_mut(residual.camera_index)
            .context("exposure diagnostic camera index is out of range")?;
        if !residual.used {
            accumulator.rejected_points += 1;
            continue;
        }
        accumulator.used_points += 1;
        let detected_view = detected_views
            .get(residual.view_index)
            .context("exposure diagnostic detected view index is out of range")?;
        let duration_s = detected_view
            .frames
            .iter()
            .find(|frame| frame.camera_index == residual.camera_index)
            .and_then(|frame| frame.exposure_duration_s);
        let Some(duration_s) = duration_s else {
            continue;
        };
        accumulator.minimum_duration_s = Some(
            accumulator
                .minimum_duration_s
                .map_or(duration_s, |current| current.min(duration_s)),
        );
        accumulator.maximum_duration_s = Some(
            accumulator
                .maximum_duration_s
                .map_or(duration_s, |current| current.max(duration_s)),
        );
        let view = views
            .get(residual.view_index)
            .context("exposure diagnostic view index is out of range")?;
        let observation = view
            .observations
            .iter()
            .find(|observation| observation.camera_index == residual.camera_index)
            .context("exposure diagnostic observation is missing")?;
        let object = observation
            .points
            .object_point(residual.point_index)
            .context("exposure diagnostic object point is missing")?;
        let observed = observation
            .points
            .pixels()
            .get(residual.point_index)
            .copied()
            .context("exposure diagnostic image point is missing")?;
        let midpoint_camera_s = view.time.seconds();
        let predictions = [-0.5, 0.0, 0.5].map(|fraction| {
            exposure_projection(
                result,
                residual.camera_index,
                midpoint_camera_s + fraction * duration_s,
                object,
            )
        });
        let [Ok(start), Ok(midpoint), Ok(end)] = predictions else {
            accumulator.unsupported_points += 1;
            continue;
        };
        let reprojection = [midpoint[0] - observed[0], midpoint[1] - observed[1]];
        let error = reprojection[0].hypot(reprojection[1]);
        ensure!(
            (error - residual.error_px).abs() <= 1.0e-6,
            "exposure diagnostic midpoint projection disagrees with solved residual"
        );
        let motion = [end[0] - start[0], end[1] - start[1]];
        let blur = motion[0].hypot(motion[1]);
        let average = [
            (start[0] + 4.0 * midpoint[0] + end[0]) / 6.0,
            (start[1] + 4.0 * midpoint[1] + end[1]) / 6.0,
        ];
        let geometric_average_error = (average[0] - observed[0]).hypot(average[1] - observed[1]);
        let bias = (average[0] - midpoint[0]).hypot(average[1] - midpoint[1]);
        let bin = &mut accumulator.bins[exposure_bin(blur)];
        bin.blur.push(blur);
        bin.error.push(error);
        bin.geometric_average_error.push(geometric_average_error);
        bin.bias.push(bias);
        if blur >= 0.25 {
            let direction = [motion[0] / blur, motion[1] / blur];
            let parallel = reprojection[0] * direction[0] + reprojection[1] * direction[1];
            let perpendicular = -reprojection[0] * direction[1] + reprojection[1] * direction[0];
            bin.parallel_sum += parallel;
            bin.parallel_squared_sum += parallel * parallel;
            bin.perpendicular_squared_sum += perpendicular * perpendicular;
            accumulator.anisotropy_samples.push(AnisotropySample {
                fold: usize::from(view.time.seconds().rem_euclid(10.0) >= 5.0),
                motion_squared: blur * blur,
                parallel_squared: parallel * parallel,
                perpendicular_squared: perpendicular * perpendicular,
            });
            bin.directional_points += 1;
        }
    }

    let labels = ["<0.25", "0.25-0.5", "0.5-1", "1-2", ">=2"];
    let cameras = config
        .cameras
        .iter()
        .enumerate()
        .map(|(camera_index, camera)| {
            let accumulator = &mut accumulators[camera_index];
            let bins = accumulator
                .bins
                .iter_mut()
                .zip(labels)
                .filter_map(|(bin, label)| summarize_exposure_bin(label, bin))
                .collect();
            CameraExposureDiagnostics {
                camera_index,
                camera_id: camera.id.clone(),
                timestamp_reference: camera.timing.timestamp_reference,
                fixed_exposure_duration_s: camera.timing.fixed_exposure_duration_s,
                exposure_metadata_topic: camera.timing.exposure_metadata_topic.clone(),
                exposure_duration_s_range: accumulator
                    .minimum_duration_s
                    .zip(accumulator.maximum_duration_s)
                    .map(|(minimum, maximum)| [minimum, maximum]),
                status: if camera.timing.fixed_exposure_duration_s.is_some()
                    || camera.timing.exposure_metadata_topic.is_some()
                {
                    "available"
                } else {
                    "unavailable_no_exposure_duration"
                },
                used_points: accumulator.used_points,
                rejected_points: accumulator.rejected_points,
                unsupported_points: accumulator.unsupported_points,
                anisotropy_cross_validation: anisotropy_cross_validation(
                    &accumulator.anisotropy_samples,
                ),
                bins,
            }
        })
        .collect();
    run.write_yaml(
        Path::new("exposure_diagnostics.yaml"),
        &ExposureDiagnostics {
            schema_version: 1,
            projection_model: "simpson_geometric_average_diagnostic_only",
            cameras,
        },
    )
}

fn exposure_projection(
    result: &ContinuousTimeCalibrationResult,
    camera_index: usize,
    camera_time_s: f64,
    object: [f64; 3],
) -> Result<[f64; 2]> {
    let centered_time_s = camera_time_s - result.clock_reference_time_s;
    let imu_time_s = camera_time_s
        + result.time_offset_s
        + result.camera_time_offsets_s[camera_index]
        + centered_time_s
            * (result.clock_skew_s_per_s + result.camera_clock_skews_s_per_s[camera_index]);
    let sample = result
        .trajectory
        .evaluate(SampleTime::from_seconds(imu_time_s)?)?;
    let camera_from_world =
        result.camera_from_imu[camera_index].compose(&sample.world_from_imu.inverse()?)?;
    ccl_calib::camera::project(
        result.cameras[camera_index],
        camera_from_world.rotation_vector(),
        camera_from_world.translation(),
        object,
    )
    .context("finite-exposure projection is outside the camera model domain")
}

fn exposure_bin(motion_px: f64) -> usize {
    if motion_px < 0.25 {
        0
    } else if motion_px < 0.5 {
        1
    } else if motion_px < 1.0 {
        2
    } else if motion_px < 2.0 {
        3
    } else {
        4
    }
}

fn anisotropy_cross_validation(samples: &[AnisotropySample]) -> Option<AnisotropyCrossValidation> {
    const LABELS: [&str; 2] = ["even 5 s blocks", "odd 5 s blocks"];
    if samples.len() < 4 || (0..2).any(|fold| samples.iter().all(|sample| sample.fold != fold)) {
        return None;
    }
    let folds = LABELS
        .into_iter()
        .enumerate()
        .map(|(held_out_fold, label)| {
            let training = samples
                .iter()
                .filter(|sample| sample.fold != held_out_fold)
                .copied()
                .collect::<Vec<_>>();
            let held_out = samples
                .iter()
                .filter(|sample| sample.fold == held_out_fold)
                .copied()
                .collect::<Vec<_>>();
            anisotropy_fold(label, &training, &held_out)
        })
        .collect::<Option<Vec<_>>>()?;
    Some(AnisotropyCrossValidation {
        temporal_block_s: 5.0,
        eligible_points: samples.len(),
        folds,
    })
}

fn anisotropy_fold(
    label: &'static str,
    training: &[AnisotropySample],
    held_out: &[AnisotropySample],
) -> Option<AnisotropyFoldSummary> {
    let training_count = count_as_f64(training.len());
    let perpendicular_variance = training
        .iter()
        .map(|sample| sample.perpendicular_squared)
        .sum::<f64>()
        / training_count;
    let isotropic_variance = training
        .iter()
        .map(|sample| sample.parallel_squared + sample.perpendicular_squared)
        .sum::<f64>()
        / (2.0 * training_count);
    let numerator = training
        .iter()
        .map(|sample| sample.motion_squared * (sample.parallel_squared - perpendicular_variance))
        .sum::<f64>();
    let denominator = training
        .iter()
        .map(|sample| sample.motion_squared * sample.motion_squared)
        .sum::<f64>();
    let parallel_variance_per_motion_squared = (numerator / denominator).max(0.0);
    if !perpendicular_variance.is_finite()
        || perpendicular_variance <= 0.0
        || !isotropic_variance.is_finite()
        || isotropic_variance <= 0.0
        || !parallel_variance_per_motion_squared.is_finite()
    {
        return None;
    }
    let held_out_count = count_as_f64(held_out.len());
    let mut isotropic_nll = 0.0;
    let mut anisotropic_nll = 0.0;
    let mut volume_preserving_nll = 0.0;
    let mut isotropic_squared_whitened = 0.0;
    let mut anisotropic_squared_whitened = 0.0;
    let mut volume_preserving_squared_whitened = 0.0;
    for sample in held_out {
        let parallel_variance =
            perpendicular_variance + parallel_variance_per_motion_squared * sample.motion_squared;
        let isotropic_mahalanobis =
            (sample.parallel_squared + sample.perpendicular_squared) / isotropic_variance;
        let anisotropic_mahalanobis = sample.parallel_squared / parallel_variance
            + sample.perpendicular_squared / perpendicular_variance;
        let variance_ratio = parallel_variance / perpendicular_variance;
        let volume_preserving_parallel_variance = isotropic_variance * variance_ratio.sqrt();
        let volume_preserving_perpendicular_variance = isotropic_variance / variance_ratio.sqrt();
        let volume_preserving_mahalanobis = sample.parallel_squared
            / volume_preserving_parallel_variance
            + sample.perpendicular_squared / volume_preserving_perpendicular_variance;
        isotropic_nll += 0.5 * (isotropic_mahalanobis + 2.0 * isotropic_variance.ln());
        anisotropic_nll +=
            0.5 * (anisotropic_mahalanobis + parallel_variance.ln() + perpendicular_variance.ln());
        volume_preserving_nll +=
            0.5 * (volume_preserving_mahalanobis + 2.0 * isotropic_variance.ln());
        isotropic_squared_whitened += isotropic_mahalanobis;
        anisotropic_squared_whitened += anisotropic_mahalanobis;
        volume_preserving_squared_whitened += volume_preserving_mahalanobis;
    }
    let isotropic_mean_nll = isotropic_nll / held_out_count;
    let anisotropic_mean_nll = anisotropic_nll / held_out_count;
    let volume_preserving_mean_nll = volume_preserving_nll / held_out_count;
    Some(AnisotropyFoldSummary {
        held_out_blocks: label,
        training_points: training.len(),
        held_out_points: held_out.len(),
        fitted_isotropic_sigma_px: isotropic_variance.sqrt(),
        fitted_perpendicular_sigma_px: perpendicular_variance.sqrt(),
        fitted_parallel_variance_per_motion_squared: parallel_variance_per_motion_squared,
        held_out_isotropic_mean_nll: isotropic_mean_nll,
        held_out_anisotropic_mean_nll: anisotropic_mean_nll,
        held_out_mean_nll_delta: anisotropic_mean_nll - isotropic_mean_nll,
        held_out_isotropic_whitened_rms: (isotropic_squared_whitened / (2.0 * held_out_count))
            .sqrt(),
        held_out_anisotropic_whitened_rms: (anisotropic_squared_whitened / (2.0 * held_out_count))
            .sqrt(),
        held_out_volume_preserving_mean_nll: volume_preserving_mean_nll,
        held_out_volume_preserving_mean_nll_delta: volume_preserving_mean_nll - isotropic_mean_nll,
        held_out_volume_preserving_whitened_rms: (volume_preserving_squared_whitened
            / (2.0 * held_out_count))
            .sqrt(),
    })
}

fn summarize_exposure_bin(
    label: &'static str,
    bin: &mut ExposureBinAccumulator,
) -> Option<ExposureBinSummary> {
    let points = bin.error.len();
    if points == 0 {
        return None;
    }
    let point_count = count_as_f64(points);
    let directional_count = count_as_f64(bin.directional_points);
    Some(ExposureBinSummary {
        predicted_motion_px: label,
        points,
        mean_predicted_motion_px: bin.blur.iter().sum::<f64>() / point_count,
        p95_predicted_motion_px: p95(&mut bin.blur),
        rms_reprojection_px: (bin.error.iter().map(|value| value * value).sum::<f64>()
            / point_count)
            .sqrt(),
        p95_reprojection_px: p95(&mut bin.error),
        rms_geometric_average_reprojection_px: (bin
            .geometric_average_error
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            / point_count)
            .sqrt(),
        p95_geometric_average_reprojection_px: p95(&mut bin.geometric_average_error),
        mean_squared_error_delta_px2: (bin
            .geometric_average_error
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            - bin.error.iter().map(|value| value * value).sum::<f64>())
            / point_count,
        mean_geometric_midpoint_bias_px: bin.bias.iter().sum::<f64>() / point_count,
        p95_geometric_midpoint_bias_px: p95(&mut bin.bias),
        directional_points: bin.directional_points,
        mean_parallel_residual_px: (bin.directional_points > 0)
            .then_some(bin.parallel_sum / directional_count),
        rms_parallel_residual_px: (bin.directional_points > 0)
            .then_some((bin.parallel_squared_sum / directional_count).sqrt()),
        rms_perpendicular_residual_px: (bin.directional_points > 0)
            .then_some((bin.perpendicular_squared_sum / directional_count).sqrt()),
    })
}

fn p95(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    let rank = (95 * values.len()).div_ceil(100).saturating_sub(1);
    values[rank]
}

#[allow(clippy::cast_precision_loss)]
fn count_as_f64(value: usize) -> f64 {
    value as f64
}

fn continuous_solver_config(
    config: &RunSettings,
    imu_config: &crate::config::ImuConfig,
    _imu: &ImuSequence<'_>,
) -> Result<ContinuousTimeSolverConfig> {
    Ok(ContinuousTimeSolverConfig {
        solver: joint_solver(config),
        gyro_noise_covariance_rad2_s_hz: imu_config.noise.gyro_covariance_rad2_s_hz,
        accelerometer_noise_covariance_m2_s4_hz: imu_config.noise.accelerometer_covariance_m2_s4_hz,
        gravity_magnitude_m_s2: imu_config.gravity_magnitude_m_s2,
        max_imu_gap_s: imu_config.integration.max_gap_s,
        max_imu_samples: imu_config.integration.max_samples,
        refine_imu_calibration: false,
    })
}

fn inertial_output(
    config: &RunSettings,
    detected: &DetectedDataset,
    result: &VisualInertialCalibrationResult,
) -> CalibrationOutput {
    let cameras = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| {
            camera_output(
                &camera.id,
                detected.image_sizes[index],
                result.cameras[index],
            )
        })
        .collect();
    let camera_from_imu = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| NamedPose {
            direction: format!("T_{}_from_imu", camera.id),
            pose: PoseOutput::from_pose(result.camera_from_imu[index]),
        })
        .collect();
    CalibrationOutput {
        schema_version: RESULT_SCHEMA_VERSION,
        mode: "multi_camera_imu".to_owned(),
        acceptance: AcceptanceStatus::PendingHumanReview,
        cameras,
        reference_camera: None,
        camera_from_reference: Vec::new(),
        inertial: Some(InertialOutput {
            method: "preintegration".to_owned(),
            camera_from_imu,
            gyro_bias_rad_s: result.initial_bias.gyro_rad_s,
            accelerometer_bias_m_s2: result.initial_bias.accelerometer_m_s2,
            gravity_world_m_s2: result.gravity_world_m_s2,
            time_offset_s: result.time_offset_s,
            clock_reference_time_s: 0.0,
            clock_skew_ppm: 0.0,
            camera_time_offsets_s: Vec::new(),
            camera_clock_skews_ppm: Vec::new(),
            axis_calibration: ImuAxisCalibrationOutput::default(),
        }),
    }
}

fn continuous_time_output(
    config: &RunSettings,
    detected: &DetectedDataset,
    result: &ContinuousTimeCalibrationResult,
) -> CalibrationOutput {
    let cameras = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| {
            camera_output(
                &camera.id,
                detected.image_sizes[index],
                result.cameras[index],
            )
        })
        .collect();
    let camera_from_imu = config
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| NamedPose {
            direction: format!("T_{}_from_imu", camera.id),
            pose: PoseOutput::from_pose(result.camera_from_imu[index]),
        })
        .collect();
    CalibrationOutput {
        schema_version: RESULT_SCHEMA_VERSION,
        mode: "multi_camera_imu".to_owned(),
        acceptance: AcceptanceStatus::PendingHumanReview,
        cameras,
        reference_camera: match &config.calibration {
            CalibrationConfig::MultiCameraImu {
                visual_reference_camera,
                ..
            } => Some(visual_reference_camera.clone()),
            _ => None,
        },
        camera_from_reference: Vec::new(),
        inertial: Some(InertialOutput {
            method: "continuous_time".to_owned(),
            camera_from_imu,
            gyro_bias_rad_s: result.bias.gyro_rad_s,
            accelerometer_bias_m_s2: result.bias.accelerometer_m_s2,
            gravity_world_m_s2: result.gravity_world_m_s2,
            time_offset_s: result.time_offset_s,
            clock_reference_time_s: result.clock_reference_time_s,
            clock_skew_ppm: result.clock_skew_s_per_s * 1.0e6,
            camera_time_offsets_s: config
                .cameras
                .iter()
                .zip(&result.camera_time_offsets_s)
                .map(|(camera, &value_s)| CameraTimeOffsetOutput {
                    id: camera.id.clone(),
                    value_s,
                })
                .collect(),
            camera_clock_skews_ppm: config
                .cameras
                .iter()
                .zip(&result.camera_clock_skews_s_per_s)
                .map(
                    |(camera, &value_s_per_s)| crate::export::CameraClockSkewOutput {
                        id: camera.id.clone(),
                        value_ppm: value_s_per_s * 1.0e6,
                    },
                )
                .collect(),
            axis_calibration: ImuAxisCalibrationOutput {
                gyro_calibrated_from_raw: result.imu_calibration.gyro_calibrated_from_raw,
                accelerometer_calibrated_from_raw: result
                    .imu_calibration
                    .accelerometer_calibrated_from_raw,
            },
        }),
    }
}

fn build_observations<'a>(
    target: &'a TargetGeometry,
    views: &'a [DetectedView],
    selected_camera: Option<usize>,
) -> Result<Vec<Vec<CameraObservation<'a>>>> {
    views
        .iter()
        .map(|view| {
            view.frames
                .iter()
                .filter(|frame| {
                    selected_camera.is_none_or(|selected| frame.camera_index == selected)
                })
                .map(|frame| {
                    Ok(CameraObservation {
                        camera_index: if selected_camera.is_some() {
                            0
                        } else {
                            frame.camera_index
                        },
                        image_size: frame.image_size,
                        points: PointCorrespondences::from_indexed_points(
                            target,
                            &frame.target_indices,
                            &frame.pixels,
                        )?,
                    })
                })
                .collect()
        })
        .collect()
}

/// Pairs each view with its observations at that view's own source time. The two slices are
/// built from the same view list, so a count mismatch is a programming error rather than
/// something to zip away silently.
fn timed_views<'a>(
    origin_ns: i64,
    views: &'a [DetectedView],
    observations: &'a [Vec<CameraObservation<'a>>],
) -> Result<Vec<TimedView<'a>>> {
    ensure!(
        views.len() == observations.len(),
        "camera view and observation counts differ"
    );
    views
        .iter()
        .zip(observations)
        .map(|(view, observations)| {
            Ok(TimedView {
                time: SampleTime::from_nanoseconds(
                    view.timestamp_ns.context("missing camera view timestamp")?,
                    origin_ns,
                )?,
                observations,
            })
        })
        .collect()
}

fn shared_views<'a>(observations: &'a [Vec<CameraObservation<'a>>]) -> Vec<SharedView<'a>> {
    observations
        .iter()
        .map(|observations| SharedView { observations })
        .collect()
}

/// Seed intrinsics on a representative set before attempting the difficult per-view PnPs.
/// Selection is initialization only: the caller always fits the entire original view list.
fn bootstrap_camera(
    nominal: CameraParameters,
    camera_id: &str,
    views: &[SharedView<'_>],
    solver: &JointSolverConfig,
    control: &SolveControl<'_>,
) -> Result<CameraParameters> {
    let indices = intrinsic_seed::select(views);
    if indices.len() == views.len() {
        return Ok(nominal); // The caller's full solve is already the seed solve.
    }
    let seed_views: Vec<_> = indices.iter().map(|&i| views[i].clone()).collect();
    let result = ccl_calib::camera::rig::calibrate_camera_rig(
        &CameraRigCalibrationRequest {
            cameras: &[nominal], views: &seed_views, reference_camera: 0,
            initialization: RigInitialization::Planar, refine_intrinsics: true,
        },
        RigSolverConfig { solver: solver.clone(), ..RigSolverConfig::default() },
        control,
    ).with_context(|| format!("{camera_id}: intrinsic bootstrap failed on seed view indices {indices:?} (of {})", views.len()))?;
    Ok(result.cameras[0])
}

fn calibrate_automatic_intrinsics<'a>(
    config: &RunSettings,
    cameras: &[CameraParameters],
    observations: &(impl Iterator<Item = &'a [CameraObservation<'a>]> + Clone),
    control: &SolveControl<'_>,
    run: &RunContext,
) -> Result<Vec<CameraParameters>> {
    cameras
        .iter()
        .enumerate()
        .map(|(camera_index, &initial)| {
            let camera_observations = observations
                .clone()
                .filter_map(|view| {
                    view.iter()
                        .find(|observation| observation.camera_index == camera_index)
                        .cloned()
                        .map(|mut observation| {
                            observation.camera_index = 0;
                            [observation]
                        })
                })
                .collect::<Vec<_>>();
            let views = camera_observations
                .iter()
                .map(|observations| SharedView { observations })
                .collect::<Vec<_>>();
            let mut solver = joint_solver(config);
            solver.pixel_sigmas = vec![config.cameras[camera_index].pixel_sigma];
            let initial = bootstrap_camera(initial, &config.cameras[camera_index].id, &views, &solver, control)?;
            let result = ccl_calib::camera::rig::calibrate_camera_rig(
                &CameraRigCalibrationRequest {
                    cameras: &[initial],
                    views: &views,
                    reference_camera: 0,
                    initialization: RigInitialization::Planar,
                    refine_intrinsics: true,
                },
                RigSolverConfig {
                    solver,
                    ..RigSolverConfig::default()
                },
                control,
            )
            .with_context(|| {
                format!(
                    "automatic intrinsic initialization failed for {} ({:?})",
                    config.cameras[camera_index].id, config.cameras[camera_index].model
                )
            })?;
            let summary = &result.camera_errors[0];
            run.log("INFO", &format!(
                "automatic intrinsics {}: {} views, {} used / {} rejected points, RMS {:.4} px, P95 {:.4} px, model {:?}",
                config.cameras[camera_index].id, views.len(), summary.used_points, summary.rejected_points,
                summary.rms_px, summary.p95_px, result.cameras[0],
            ))?;
            Ok(result.cameras[0])
        })
        .collect()
}

fn initial_cameras(
    config: &RunSettings,
    detected: &DetectedDataset,
    indices: &[usize],
    initial: &InitialIntrinsicsConfig,
) -> Result<Vec<CameraParameters>> {
    let prior = initial
        .path
        .as_ref()
        .map(|path| {
            #[cfg(not(target_arch = "wasm32"))]
            { load_result(path) }
            #[cfg(target_arch = "wasm32")]
            { crate::export::load_result_bytes(&crate::browser::read_authorized_file(path)?) }
        })
        .transpose()?;
    indices
        .iter()
        .map(|&index| {
            let camera = &config.cameras[index];
            let image_size = detected.image_sizes[index];
            let Some(prior) = &prior else {
                return CameraParameters::initial_for_image(camera.model.core(), image_size)
                    .map_err(Into::into);
            };
            let previous = prior.camera(&camera.id)?;
            ensure!(
                previous.image_width == image_size.width
                    && previous.image_height == image_size.height,
                "prior image size mismatch for {}",
                camera.id
            );
            ensure!(
                previous.parameters.model() == camera.model.core(),
                "prior camera model mismatch for {}",
                camera.id
            );
            previous.parameters.to_core()
        })
        .collect()
}

fn camera_model_summary(config: &RunSettings) -> String {
    config
        .cameras
        .iter()
        .map(|camera| format!("{}={:?}", camera.id, camera.model))
        .collect::<Vec<_>>()
        .join(", ")
}

fn camera_index(config: &RunSettings, id: &str) -> Result<usize> {
    config
        .cameras
        .iter()
        .position(|camera| camera.id == id)
        .with_context(|| format!("unknown camera {id}"))
}

fn target_geometry(config: &RunSettings) -> Result<TargetGeometry> {
    let TargetConfig::Aprilgrid {
        tag_size_m,
        tag_gap_m,
        id_rows_front_view_left_to_right,
        ..
    } = &config.target;
    let rows = u16::try_from(id_rows_front_view_left_to_right.len())
        .context("target row count exceeds u16")?;
    let columns = u16::try_from(id_rows_front_view_left_to_right[0].len())
        .context("target column count exceeds u16")?;
    Ok(TargetGeometry::april_grid(
        rows,
        columns,
        *tag_size_m,
        *tag_gap_m,
    )?)
}

fn core_time_offset(config: TimeOffsetConfig, solved_initial_s: Option<f64>) -> TimeOffset {
    match config {
        TimeOffsetConfig::Fixed { value_s } => TimeOffset::Fixed(value_s),
        TimeOffsetConfig::Estimate {
            initial_s,
            minimum_s,
            maximum_s,
            initialization_steps,
        } => TimeOffset::Estimate {
            initial_s: solved_initial_s.unwrap_or(initial_s),
            minimum_s,
            maximum_s,
            initialization_steps,
        },
    }
}

fn summed_time_offset_bounds(
    global: TimeOffsetConfig,
    camera: crate::config::CameraTimeOffsetsConfig,
    drift_margin_s: f64,
) -> TimeOffset {
    let (initial_s, mut minimum_s, mut maximum_s, initialization_steps) = match global {
        TimeOffsetConfig::Fixed { value_s } => (value_s, value_s, value_s, 2),
        TimeOffsetConfig::Estimate {
            initial_s,
            minimum_s,
            maximum_s,
            initialization_steps,
        } => (initial_s, minimum_s, maximum_s, initialization_steps),
    };
    if camera.refine {
        minimum_s += camera.minimum_s;
        maximum_s += camera.maximum_s;
    }
    minimum_s -= drift_margin_s;
    maximum_s += drift_margin_s;
    if minimum_s == maximum_s {
        TimeOffset::Fixed(initial_s)
    } else {
        TimeOffset::Estimate {
            initial_s,
            minimum_s,
            maximum_s,
            initialization_steps,
        }
    }
}

fn isotropic_density(covariance: [[f64; 3]; 3], sensor: &str) -> Result<f64> {
    let variance = covariance[0][0];
    ensure!(
        covariance[0][1] == 0.0
            && covariance[0][2] == 0.0
            && covariance[1][0] == 0.0
            && covariance[1][2] == 0.0
            && covariance[2][0] == 0.0
            && covariance[2][1] == 0.0
            && covariance[1][1] == variance
            && covariance[2][2] == variance,
        "preintegration requires isotropic diagonal {sensor} noise covariance"
    );
    Ok(variance.sqrt())
}

fn joint_solver(config: &RunSettings) -> JointSolverConfig {
    JointSolverConfig {
        max_iterations: config.solver.max_iterations,
        pixel_sigmas: config
            .cameras
            .iter()
            .map(|camera| camera.pixel_sigma)
            .collect(),
        huber_delta: config.solver.huber_delta,
        point_rejection_px: config.solver.point_rejection_px,
    }
}

fn camera_output(
    id: &str,
    size: ccl_calib::camera::CalibrationImageSize,
    parameters: CameraParameters,
) -> CameraOutput {
    CameraOutput {
        id: id.to_owned(),
        image_width: size.width,
        image_height: size.height,
        parameters: crate::export::CameraParametersOutput::from_core(parameters),
    }
}

fn camera_solve_summaries(
    errors: &[ccl_calib::camera::rig::CameraResidualSummary],
) -> Vec<CameraSolveSummary> {
    errors
        .iter()
        .map(|summary| CameraSolveSummary {
            camera_index: summary.camera_index,
            used_points: summary.used_points,
            rejected_points: summary.rejected_points,
            rms_px: summary.rms_px,
            p95_px: summary.p95_px,
        })
        .collect()
}

fn solve_summary(
    convergence: &JointConvergence,
    information: &CalibrationInformation,
    camera_errors: &[ccl_calib::camera::rig::CameraResidualSummary],
    pixel_sigmas: &[f64],
    gyro_whitened_rms: Option<f64>,
    accelerometer_whitened_rms: Option<f64>,
    inertial_whitened_rms: Option<f64>,
) -> SolveSummary {
    SolveSummary {
        converged: convergence.termination.is_converged(),
        termination: format!("{:?}", convergence.termination),
        iterations: convergence.iterations,
        initial_cost: convergence.initial_cost,
        final_cost: convergence.final_cost,
        final_damping: convergence.damping,
        scaled_gradient_inf_norm: convergence.scaled_gradient_inf_norm,
        accepted_steps: convergence.accepted_steps,
        rejected_steps: convergence.rejected_steps,
        information_rank: information.rank,
        information_parameters: information.parameter_names.len(),
        condition_number: information.condition_number,
        covariance_is_pseudoinverse: information.covariance_is_pseudoinverse,
        camera_errors: camera_solve_summaries(camera_errors),
        visual_whitened_rms: camera_errors
            .iter()
            .map(|error| error.rms_px / pixel_sigmas[error.camera_index])
            .collect(),
        gyro_whitened_rms,
        accelerometer_whitened_rms,
        inertial_whitened_rms,
        parameter_standard_deviations: information
            .parameter_names
            .iter()
            .enumerate()
            .map(|(index, parameter)| ParameterStandardDeviation {
                parameter: parameter.clone(),
                standard_deviation: information.covariance[(index, index)].max(0.0).sqrt(),
            })
            .collect(),
    }
}

fn rig_solve_summary(result: &CameraRigCalibrationResult, config: &RunSettings) -> SolveSummary {
    solve_summary(
        &result.convergence,
        &result.information,
        &result.camera_errors,
        &config
            .cameras
            .iter()
            .map(|camera| camera.pixel_sigma)
            .collect::<Vec<_>>(),
        None,
        None,
        None,
    )
}

fn inertial_solve_summary(
    result: &VisualInertialCalibrationResult,
    config: &RunSettings,
) -> SolveSummary {
    let inertial_whitened_rms = (!result.inertial_residuals.is_empty()).then(|| {
        let squared = result
            .inertial_residuals
            .iter()
            .map(|residual| residual.whitened_rms.powi(2))
            .sum::<f64>();
        (squared / result.inertial_residuals.len() as f64).sqrt()
    });
    solve_summary(
        &result.convergence,
        &result.information,
        &result.camera_errors,
        &config
            .cameras
            .iter()
            .map(|camera| camera.pixel_sigma)
            .collect::<Vec<_>>(),
        None,
        None,
        inertial_whitened_rms,
    )
}

fn continuous_time_solve_summary(
    result: &ContinuousTimeCalibrationResult,
    config: &RunSettings,
) -> SolveSummary {
    let rms = |values: &[f64]| {
        (!values.is_empty()).then(|| {
            (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
        })
    };
    let gyro = result
        .imu_residuals
        .iter()
        .map(|residual| residual.gyro_whitened_rms)
        .collect::<Vec<_>>();
    let accel = result
        .imu_residuals
        .iter()
        .map(|residual| residual.accelerometer_whitened_rms)
        .collect::<Vec<_>>();
    let combined = result
        .imu_residuals
        .iter()
        .map(|residual| residual.whitened_rms)
        .collect::<Vec<_>>();
    solve_summary(
        &result.convergence,
        &result.information,
        &result.camera_errors,
        &config
            .cameras
            .iter()
            .map(|camera| camera.pixel_sigma)
            .collect::<Vec<_>>(),
        rms(&gyro),
        rms(&accel),
        rms(&combined),
    )
}

fn log_rig_diagnostics(run: &RunContext, result: &CameraRigCalibrationResult) -> Result<()> {
    run.log(
        "INFO",
        &format!(
            "termination={:?} converged={} iterations={} initial_cost={:.9} final_cost={:.9} information_rank={}/{} condition_number={:.9} covariance_pseudoinverse={}",
            result.convergence.termination,
            result.convergence.termination.is_converged(),
            result.convergence.iterations,
            result.convergence.initial_cost,
            result.convergence.final_cost,
            result.information.rank,
            result.information.parameter_names.len(),
            result.information.condition_number,
            result.information.covariance_is_pseudoinverse,
        ),
    )?;
    for summary in &result.camera_errors {
        run.log(
            "INFO",
            &format!(
                "camera={} used_points={} rejected_points={} rms_px={:.9} p95_px={:.9}",
                summary.camera_index,
                summary.used_points,
                summary.rejected_points,
                summary.rms_px,
                summary.p95_px
            ),
        )?;
    }
    for residual in &result.residuals {
        run.log(
            "DEBUG",
            &format!(
                "view={} camera={} point={} target_point={} used={} error_px={:.9}",
                residual.view_index,
                residual.camera_index,
                residual.point_index,
                residual.target_point_index,
                residual.used,
                residual.error_px
            ),
        )?;
    }
    for closure in &result.graph_closures {
        run.log("INFO", &format!("closure camera_a={} camera_b={} shared_views={} rotation_rad={:.9} translation_m={:.9}", closure.camera_a, closure.camera_b, closure.shared_views, closure.rotation_rad, closure.translation_m))?;
    }
    Ok(())
}

fn log_inertial_diagnostics(
    run: &RunContext,
    result: &VisualInertialCalibrationResult,
) -> Result<()> {
    run.log(
        "INFO",
        &format!(
            "termination={:?} converged={} iterations={} initial_cost={:.9} final_cost={:.9} information_rank={}/{} condition_number={:.9} covariance_pseudoinverse={}",
            result.convergence.termination,
            result.convergence.termination.is_converged(),
            result.convergence.iterations,
            result.convergence.initial_cost,
            result.convergence.final_cost,
            result.information.rank,
            result.information.parameter_names.len(),
            result.information.condition_number,
            result.information.covariance_is_pseudoinverse,
        ),
    )?;
    for summary in &result.camera_errors {
        run.log(
            "INFO",
            &format!(
                "camera={} used_points={} rejected_points={} rms_px={:.9} p95_px={:.9}",
                summary.camera_index,
                summary.used_points,
                summary.rejected_points,
                summary.rms_px,
                summary.p95_px
            ),
        )?;
    }
    for residual in &result.inertial_residuals {
        run.log("DEBUG", &format!("imu_interval={} rotation_error_rad={:?} velocity_error_m_s={:?} position_error_m={:?} whitened_rms={:.9} bias_walk_whitened_rms={:.9}", residual.interval_index, residual.rotation_error_rad, residual.velocity_error_m_s, residual.position_error_m, residual.whitened_rms, residual.bias_walk_whitened_rms))?;
    }
    for state in &result.states {
        run.log("DEBUG", &format!("state camera_time_s={:.9} imu_time_s={:.9} world_from_imu={:?} velocity_world_m_s={:?} gyro_bias={:?} accel_bias={:?}", state.camera_time.seconds(), state.imu_time.seconds(), state.world_from_imu.to_homogeneous(), state.velocity_world_m_s, state.bias.gyro_rad_s, state.bias.accelerometer_m_s2))?;
    }
    Ok(())
}

fn log_continuous_time_diagnostics(
    run: &RunContext,
    result: &ContinuousTimeCalibrationResult,
) -> Result<()> {
    run.log(
        "INFO",
        &format!(
            "continuous_time termination={:?} converged={} iterations={} initial_cost={:.9} final_cost={:.9} information_rank={}/{} condition_number={:.9} covariance_pseudoinverse={}",
            result.convergence.termination,
            result.convergence.termination.is_converged(),
            result.convergence.iterations,
            result.convergence.initial_cost,
            result.convergence.final_cost,
            result.information.rank,
            result.information.parameter_names.len(),
            result.information.condition_number,
            result.information.covariance_is_pseudoinverse,
        ),
    )?;
    run.log(
        "INFO",
        &format!(
            "affine_clock reference_time_s={:.9} global_offset_s={:+.9} global_skew_ppm={:+.6} camera_offsets_s={:?} camera_skews_ppm={:?}",
            result.clock_reference_time_s,
            result.time_offset_s,
            result.clock_skew_s_per_s * 1.0e6,
            result.camera_time_offsets_s,
            result
                .camera_clock_skews_s_per_s
                .iter()
                .map(|value| value * 1.0e6)
                .collect::<Vec<_>>()
        ),
    )?;
    run.log(
        "INFO",
        &format!(
            "imu_axis_calibration gyro_calibrated_from_raw={:?} accelerometer_calibrated_from_raw={:?}",
            result.imu_calibration.gyro_calibrated_from_raw,
            result.imu_calibration.accelerometer_calibrated_from_raw
        ),
    )?;
    for summary in &result.camera_errors {
        run.log(
            "INFO",
            &format!(
                "camera={} used_points={} rejected_points={} rms_px={:.9} p95_px={:.9}",
                summary.camera_index,
                summary.used_points,
                summary.rejected_points,
                summary.rms_px,
                summary.p95_px
            ),
        )?;
    }
    for residual in &result.imu_residuals {
        run.log(
            "DEBUG",
            &format!(
                "imu_sample={} time_s={:.9} gyro_error_rad_s={:?} force_error_m_s2={:?} whitened_rms={:.9}",
                residual.sample_index,
                residual.time.seconds(),
                residual.gyro_error_rad_s,
                residual.specific_force_error_m_s2,
                residual.whitened_rms
            ),
        )?;
    }
    Ok(())
}

fn log_json<T: serde::Serialize>(run: &RunContext, label: &str, value: &T) -> Result<()> {
    let json = serde_json::to_string(value).with_context(|| format!("serialize {label}"))?;
    run.log("INFO", &format!("{label}={json}"))
}

fn elapsed_millis(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::{AnisotropySample, anisotropy_cross_validation};

    #[test]
    fn held_out_anisotropy_model_recovers_motion_dependent_parallel_variance() {
        const MOTION_SQUARED: [f64; 10] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let samples = (0..40)
            .map(|index| {
                let motion_squared = MOTION_SQUARED[index % MOTION_SQUARED.len()];
                AnisotropySample {
                    fold: index % 2,
                    motion_squared,
                    parallel_squared: 0.04 + 0.02 * motion_squared,
                    perpendicular_squared: 0.04,
                }
            })
            .collect::<Vec<_>>();
        let validation = anisotropy_cross_validation(&samples).unwrap();
        assert_eq!(validation.eligible_points, samples.len());
        for fold in validation.folds {
            assert!((fold.fitted_perpendicular_sigma_px - 0.2).abs() < 1.0e-12);
            assert!((fold.fitted_parallel_variance_per_motion_squared - 0.02).abs() < 1.0e-12);
            assert!(fold.held_out_anisotropic_mean_nll < fold.held_out_isotropic_mean_nll);
        }
    }
}
