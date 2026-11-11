use std::fs;
use std::path::{Path, PathBuf};

use four_p_calibration::config::{
    CalibrationConfig, CameraTimestampReference, ImuAxisCalibrationConfig, RunSettings,
};
use four_p_calibration::export::load_result;

const IDENTITY: &str = "[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]";

fn inertial_source(method: &str, axis: &str) -> String {
    let base = fs::read_to_string("config/calibration.yaml").unwrap();
    let prefix = base
        .split("calibration:")
        .next()
        .unwrap()
        .trim_end()
        .to_owned()
        .replace(
            "format: ros2_mcap\n",
            "format: ros2_mcap\n  imu_topic: /imu\n",
        );
    format!(
        "{prefix}\ncalibration:
  type: multi_camera_imu
  method: {method}
  visual_reference_camera: cam0
  time_offset:
    type: fixed
    value_s: 0.0
  imu:
    gravity_magnitude_m_s2: 9.80665
    noise:
      gyro_covariance_rad2_s_hz: [[0.000001, 0.0, 0.0], [0.0, 0.000001, 0.0], [0.0, 0.0, 0.000001]]
      accelerometer_covariance_m2_s4_hz: [[0.0001, 0.0, 0.0], [0.0, 0.0001, 0.0], [0.0, 0.0, 0.0001]]
      gyro_bias_walk_rad_s2_sqrt_hz: 0.00001
      accelerometer_bias_walk_m_s3_sqrt_hz: 0.0001
    integration:
      method: midpoint
      max_gap_s: 0.02
      max_samples: 100
{axis}"
    )
}

fn axis_block(refine: bool, gyro: &str, accelerometer: &str) -> String {
    format!(
        "    axis_calibration:
      refine: {refine}
      gyro_calibrated_from_raw: {gyro}
      accelerometer_calibrated_from_raw: {accelerometer}
"
    )
}

fn write_settings(name: &str, source: &str) -> (tempfile::TempDir, PathBuf) {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join(name);
    fs::write(&path, source).unwrap();
    (directory, path)
}

fn load_inertial(name: &str, method: &str, axis: &str) -> anyhow::Result<RunSettings> {
    let (_directory, path) = write_settings(name, &inertial_source(method, axis));
    RunSettings::load(&path)
}

fn axis(settings: &RunSettings) -> ImuAxisCalibrationConfig {
    let CalibrationConfig::MultiCameraImu { imu, .. } = &settings.calibration else {
        panic!("expected multi-camera-IMU configuration");
    };
    imu.axis_calibration
}

#[test]
fn committed_rosbag_settings_are_valid() {
    RunSettings::load(Path::new("config/calibration.yaml")).unwrap();
}

#[test]
fn committed_ds_presets_match_their_intrinsic_priors() {
    for preset in [
        "config/presets/1-bag-ds.yaml",
        "config/presets/baton-bag-ds.yaml",
        "config/presets/baton-bag-ds-continuous-basalt.yaml",
    ] {
        let settings = RunSettings::load(Path::new(preset)).unwrap();
        let prior_path = match &settings.calibration {
            CalibrationConfig::SingleCamera {
                initial_intrinsics, ..
            }
            | CalibrationConfig::MultiCamera {
                initial_intrinsics, ..
            }
            | CalibrationConfig::MultiCameraImu {
                initial_intrinsics, ..
            } => initial_intrinsics.path.as_ref().unwrap(),
        };
        let prior = load_result(prior_path).unwrap();
        assert_eq!(prior.cameras.len(), settings.cameras.len());
        for camera in &settings.cameras {
            let parameters = &prior.camera(&camera.id).unwrap().parameters;
            assert_eq!(parameters.model(), camera.model.core());
        }
    }
}

#[test]
fn unknown_fields_are_rejected() {
    let source = fs::read_to_string("config/calibration.yaml").unwrap();
    let (_directory, path) = write_settings("unknown.yaml", &format!("unexpected: true\n{source}"));
    assert!(RunSettings::load(&path).is_err());
}

#[test]
fn camera_timing_defaults_to_unshifted_exposure_midpoint() {
    let settings = RunSettings::load(Path::new("config/calibration.yaml")).unwrap();
    for camera in settings.cameras {
        assert_eq!(
            camera.timing.timestamp_reference,
            CameraTimestampReference::ExposureMidpoint
        );
        assert_eq!(camera.timing.fixed_exposure_duration_s, None);
        assert_eq!(camera.timing.exposure_metadata_topic, None);
    }
}

#[test]
fn trigger_timestamps_require_a_positive_fixed_exposure_duration() {
    let source = fs::read_to_string("config/calibration.yaml").unwrap();
    let trigger_without_duration = source.replacen(
        "  pixel_sigma: 1.0\n",
        "  pixel_sigma: 1.0\n  timing:\n    timestamp_reference: trigger\n",
        1,
    );
    let (_directory, path) =
        write_settings("trigger-without-duration.yaml", &trigger_without_duration);
    assert!(RunSettings::load(&path).is_err());

    let trigger = source.replacen(
        "  pixel_sigma: 1.0\n",
        "  pixel_sigma: 1.0\n  timing:\n    timestamp_reference: trigger\n    fixed_exposure_duration_s: 0.01\n",
        1,
    );
    let (directory, path) = write_settings("trigger.yaml", &trigger);
    let settings = RunSettings::load(&path).unwrap();
    assert_eq!(
        settings.cameras[0].timing.timestamp_reference,
        CameraTimestampReference::Trigger
    );
    assert_eq!(
        settings.cameras[0].timing.fixed_exposure_duration_s,
        Some(0.01)
    );
    let round_trip = directory.path().join("trigger-round-trip.yaml");
    fs::write(&round_trip, serde_yaml::to_string(&settings).unwrap()).unwrap();
    RunSettings::load(&round_trip).unwrap();
}

#[test]
fn exposure_metadata_source_is_exclusive_with_fixed_duration() {
    let source = fs::read_to_string("config/calibration.yaml").unwrap();
    let metadata = source.replacen(
        "  pixel_sigma: 1.0\n",
        "  pixel_sigma: 1.0\n  timing:\n    timestamp_reference: exposure_midpoint\n    exposure_metadata_topic: /camera0/frame_metadata\n",
        1,
    );
    let (_directory, path) = write_settings("metadata-timing.yaml", &metadata);
    let settings = RunSettings::load(&path).unwrap();
    assert_eq!(
        settings.cameras[0]
            .timing
            .exposure_metadata_topic
            .as_deref(),
        Some("/camera0/frame_metadata")
    );

    let both = metadata.replace(
        "    exposure_metadata_topic: /camera0/frame_metadata\n",
        "    exposure_metadata_topic: /camera0/frame_metadata\n    fixed_exposure_duration_s: 0.01\n",
    );
    let (_directory, path) = write_settings("ambiguous-duration-source.yaml", &both);
    assert!(RunSettings::load(&path).is_err());
}
fn visual_settings(refine_intrinsics: bool) -> RunSettings {
    let mut settings = RunSettings::load(Path::new("config/calibration.yaml")).unwrap();
    settings.source.imu_topic = None;
    settings.calibration = CalibrationConfig::MultiCamera {
        reference_camera: settings.cameras[0].id.clone(),
        refine_intrinsics,
        initial_intrinsics: Default::default(),
        max_closure_rotation_deg: 2.0,
        max_closure_translation_m: 0.05,
    };
    settings.solver.point_rejection_px = Some(6.0);
    settings
}

#[test]
fn automatic_intrinsic_seed_is_valid_when_refinement_is_enabled() {
    let source = serde_yaml::to_string(&visual_settings(true)).unwrap();
    let (_directory, path) = write_settings("automatic.yaml", &source);
    let settings = RunSettings::load(&path).unwrap();
    let CalibrationConfig::MultiCamera {
        initial_intrinsics, ..
    } = settings.calibration
    else {
        panic!("expected multi-camera configuration");
    };
    assert!(initial_intrinsics.path.is_none());
}

#[test]
fn fixed_intrinsics_require_a_prior() {
    let source = serde_yaml::to_string(&visual_settings(false)).unwrap();
    let (_directory, path) = write_settings("fixed-without-prior.yaml", &source);
    assert!(RunSettings::load(&path).is_err());
}

#[test]
fn single_camera_mode_rejects_multiple_selected_topics() {
    let mut settings = visual_settings(true);
    settings.calibration = CalibrationConfig::SingleCamera {
        camera: settings.cameras[0].id.clone(),
        initial_intrinsics: Default::default(),
    };
    let single = serde_yaml::to_string(&settings).unwrap();
    let (_directory, path) = write_settings("single.yaml", &single);
    assert!(RunSettings::load(&path).is_err());
}

#[test]
fn continuous_time_axis_matrices_round_trip_through_settings() {
    let gyro = [[1.0, 0.1, 0.0], [0.05, 1.0, 0.1], [0.0, -0.1, 1.0]];
    let accelerometer = [[1.02, 0.0, 0.0], [0.01, 0.98, 0.0], [-0.02, 0.03, 1.01]];
    let settings = load_inertial(
        "axes.yaml",
        "continuous_time",
        &axis_block(true, &format!("{gyro:?}"), &format!("{accelerometer:?}")),
    )
    .unwrap();
    let loaded = axis(&settings);
    assert_eq!(loaded.gyro_calibrated_from_raw, gyro);
    assert_eq!(loaded.accelerometer_calibrated_from_raw, accelerometer);
    assert!(loaded.refine);

    let (_directory, path) = write_settings(
        "axes-round-trip.yaml",
        &serde_yaml::to_string(&settings).unwrap(),
    );
    let reloaded = axis(&RunSettings::load(&path).unwrap());
    assert_eq!(reloaded.gyro_calibrated_from_raw, gyro);
    assert_eq!(reloaded.accelerometer_calibrated_from_raw, accelerometer);
    assert!(reloaded.refine);
}

#[test]
fn accelerometer_upper_triangle_is_rejected() {
    let error = load_inertial(
        "accelerometer-gauge.yaml",
        "continuous_time",
        &axis_block(
            false,
            IDENTITY,
            "[[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]",
        ),
    );
    assert!(error.is_err());
}

#[test]
fn singular_gyro_axis_matrix_is_rejected() {
    let error = load_inertial(
        "singular-gyro.yaml",
        "continuous_time",
        &axis_block(
            false,
            "[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]]",
            IDENTITY,
        ),
    );
    assert!(error.is_err());
}

#[test]
fn orientation_reversing_gyro_axis_matrix_is_rejected() {
    let error = load_inertial(
        "reflected-gyro.yaml",
        "continuous_time",
        &axis_block(
            false,
            "[[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]",
            IDENTITY,
        ),
    );
    assert!(error.is_err());
}

#[test]
fn preintegration_rejects_axis_settings_it_would_ignore() {
    assert!(
        load_inertial(
            "preintegration-identity.yaml",
            "preintegration",
            &axis_block(false, IDENTITY, IDENTITY)
        )
        .is_ok()
    );
    let ignored = [
        axis_block(true, IDENTITY, IDENTITY),
        axis_block(
            false,
            "[[1.01, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]",
            IDENTITY,
        ),
        axis_block(
            false,
            IDENTITY,
            "[[1.0, 0.0, 0.0], [0.0, 0.99, 0.0], [0.0, 0.0, 1.0]]",
        ),
    ];
    for (index, axis) in ignored.iter().enumerate() {
        let name = format!("preintegration-ignored-{index}.yaml");
        assert!(
            load_inertial(&name, "preintegration", axis).is_err(),
            "accepted axis settings that preintegration would ignore: {axis}"
        );
    }
}

#[test]
fn obsolete_axis_field_names_are_rejected() {
    assert!(
        load_inertial(
            "current-axis-names.yaml",
            "continuous_time",
            &axis_block(false, IDENTITY, IDENTITY)
        )
        .is_ok()
    );
    for (old, new) in [
        ("gyro_calibrated_from_raw", "gyro_matrix"),
        ("accelerometer_calibrated_from_raw", "accelerometer_matrix"),
    ] {
        let axis = axis_block(false, IDENTITY, IDENTITY).replace(old, new);
        let name = format!("obsolete-{new}.yaml");
        assert!(
            load_inertial(&name, "continuous_time", &axis).is_err(),
            "accepted obsolete axis field {new}"
        );
    }
}

#[test]
fn camera_time_offset_defaults_are_fixed_at_five_milliseconds() {
    let settings = load_inertial("camera-offset-defaults.yaml", "continuous_time", "").unwrap();
    let CalibrationConfig::MultiCameraImu { imu, .. } = settings.calibration else {
        panic!("expected multi-camera-IMU configuration");
    };
    let offsets = imu.continuous_time.camera_time_offsets;
    assert!(!offsets.refine);
    assert_eq!(offsets.minimum_s, -0.005);
    assert_eq!(offsets.maximum_s, 0.005);
}

#[test]
fn clock_skew_defaults_are_fixed_at_two_hundred_ppm() {
    let settings = load_inertial("clock-skew-defaults.yaml", "continuous_time", "").unwrap();
    let CalibrationConfig::MultiCameraImu { imu, .. } = settings.calibration else {
        panic!("expected multi-camera-IMU configuration");
    };
    let skews = imu.continuous_time.clock_skews;
    assert!(!skews.refine_global);
    assert!(!skews.refine_cameras);
    assert_eq!(skews.minimum_ppm, -200.0);
    assert_eq!(skews.maximum_ppm, 200.0);
}

#[test]
fn visual_motion_anisotropy_is_explicit_and_continuous_time_only() {
    let default = load_inertial("anisotropy-default.yaml", "continuous_time", "").unwrap();
    let CalibrationConfig::MultiCameraImu { imu, .. } = default.calibration else {
        panic!("expected multi-camera-IMU configuration");
    };
    assert!(!imu.continuous_time.visual_motion_anisotropy.enabled);
    assert_eq!(
        imu.continuous_time
            .visual_motion_anisotropy
            .variance_ratio_per_motion_squared,
        0.015
    );

    let block = "    continuous_time:\n      visual_motion_anisotropy:\n        enabled: true\n        variance_ratio_per_motion_squared: 0.015\n";
    let source = inertial_source("continuous_time", "")
        .replace("    integration:\n", &format!("{block}    integration:\n"));
    let (_directory, path) = write_settings("anisotropy-enabled.yaml", &source);
    assert!(RunSettings::load(&path).is_ok());

    let source = inertial_source("preintegration", "")
        .replace("    integration:\n", &format!("{block}    integration:\n"));
    let (_directory, path) = write_settings("anisotropy-preintegration.yaml", &source);
    assert!(RunSettings::load(&path).is_err());
}

#[test]
fn committed_continuous_time_configs_keep_clock_skew_disabled() {
    for path in [
        "config/calibration.yaml",
        "config/presets/baton-bag-ds-continuous-basalt.yaml",
    ] {
        let settings = RunSettings::load(Path::new(path)).unwrap();
        let CalibrationConfig::MultiCameraImu { imu, .. } = settings.calibration else {
            panic!("expected multi-camera-IMU configuration in {path}");
        };
        assert!(!imu.continuous_time.clock_skews.refine_global, "{path}");
        assert!(!imu.continuous_time.clock_skews.refine_cameras, "{path}");
    }
}

#[test]
fn continuous_time_accepts_explicit_clock_skew_diagnostics() {
    let source = inertial_source("continuous_time", "").replace(
        "    integration:\n",
        "    continuous_time:\n      clock_skews:\n        refine_global: true\n        refine_cameras: true\n        minimum_ppm: -200\n        maximum_ppm: 200\n    integration:\n",
    );
    let (_directory, path) = write_settings("continuous-time-clock-skew.yaml", &source);
    let settings = RunSettings::load(&path).unwrap();
    let CalibrationConfig::MultiCameraImu { imu, .. } = settings.calibration else {
        panic!("expected multi-camera-IMU configuration");
    };
    assert!(imu.continuous_time.clock_skews.refine_global);
    assert!(imu.continuous_time.clock_skews.refine_cameras);
}

#[test]
fn preintegration_rejects_affine_clock_refinement() {
    let source = inertial_source("preintegration", "").replace(
        "    integration:\n",
        "    continuous_time:\n      clock_skews:\n        refine_global: true\n        minimum_ppm: -200\n        maximum_ppm: 200\n    integration:\n",
    );
    let (_directory, path) = write_settings("preintegration-clock-skew.yaml", &source);
    assert!(RunSettings::load(&path).is_err());
}

#[test]
fn preintegration_rejects_camera_time_offset_refinement() {
    let source = inertial_source("preintegration", "").replace(
        "    integration:\n",
        "    continuous_time:\n      camera_time_offsets:\n        refine: true\n        minimum_s: -0.005\n        maximum_s: 0.005\n    integration:\n",
    );
    let (_directory, path) = write_settings("preintegration-camera-offset.yaml", &source);
    assert!(RunSettings::load(&path).is_err());
}

fn with_solver(source: &str, solver: &str) -> String {
    format!("{source}solver:\n{solver}")
}

#[test]
fn continuous_time_huber_only_policy_round_trips() {
    let source = with_solver(
        &inertial_source("continuous_time", &axis_block(false, IDENTITY, IDENTITY)),
        "  max_iterations: 1000\n  huber_delta: 4.0\n  point_rejection_px: null\n",
    );
    let (directory, path) = write_settings("huber-only.yaml", &source);
    let settings = RunSettings::load(&path).unwrap();
    assert_eq!(settings.solver.point_rejection_px, None);
    let serialized = serde_yaml::to_string(&settings).unwrap();

    let round_trip = directory.path().join("huber-only-round-trip.yaml");
    fs::write(&round_trip, serialized).unwrap();
    let reloaded = RunSettings::load(&round_trip).unwrap();
    assert_eq!(reloaded.solver.point_rejection_px, None);
    assert_eq!(reloaded.solver.huber_delta, 4.0);
}

#[test]
fn huber_only_policy_is_rejected_outside_continuous_time() {
    let source = with_solver(
        &inertial_source("preintegration", &axis_block(false, IDENTITY, IDENTITY)),
        "  max_iterations: 1000\n  huber_delta: 3.0\n  point_rejection_px: null\n",
    );
    let (_directory, path) = write_settings("null-rejection-preintegration.yaml", &source);
    assert!(RunSettings::load(&path).is_err());
}
