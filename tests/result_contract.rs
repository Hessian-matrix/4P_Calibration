use ccl_calib::camera::{CalibrationImageSize, CameraModel, CameraParameters, PinholeModel};
use ccl_geometry::Pose3;
use four_p_calibration::export::{
    AcceptanceStatus, CalibrationOutput, CameraOutput, CameraParametersOutput,
    ImuAxisCalibrationOutput, InertialOutput, NamedPose, PoseOutput,
};

fn camera() -> CameraOutput {
    let size = CalibrationImageSize::new(640, 480).unwrap();
    let parameters =
        CameraParameters::initial_for_image(CameraModel::Pinhole(PinholeModel::None), size)
            .unwrap();
    CameraOutput {
        id: "cam0".to_owned(),
        image_width: size.width,
        image_height: size.height,
        parameters: CameraParametersOutput::from_core(parameters),
    }
}

fn inertial(axis_calibration: ImuAxisCalibrationOutput) -> InertialOutput {
    InertialOutput {
        method: "continuous_time".to_owned(),
        camera_from_imu: vec![NamedPose {
            direction: "T_cam0_from_imu".to_owned(),
            pose: PoseOutput::from_pose(Pose3::identity()),
        }],
        gyro_bias_rad_s: [0.01, -0.02, 0.003],
        accelerometer_bias_m_s2: [0.1, -0.2, 0.03],
        gravity_world_m_s2: [0.0, 0.0, 9.80665],
        time_offset_s: 0.002,
        axis_calibration,
    }
}

fn output(schema_version: u32, inertial: Option<InertialOutput>) -> CalibrationOutput {
    CalibrationOutput {
        schema_version,
        mode: "multi_camera".to_owned(),
        acceptance: AcceptanceStatus::PendingHumanReview,
        cameras: vec![camera()],
        reference_camera: Some("cam0".to_owned()),
        camera_from_reference: vec![NamedPose {
            direction: "T_cam0_from_cam0".to_owned(),
            pose: PoseOutput::from_pose(Pose3::identity()),
        }],
        inertial,
    }
}

#[test]
fn pure_result_round_trips_without_solver_diagnostics() {
    let mut calibration = output(4, None);
    let transform = Pose3::from_rvec_tvec([0.1, -0.2, 0.05], [0.12, -0.03, 0.01]).unwrap();
    calibration.camera_from_reference[0].pose = PoseOutput::from_pose(transform);
    calibration.validate().unwrap();
    let yaml = serde_yaml::to_string(&calibration).unwrap();
    for forbidden in ["rms", "cost", "iterations", "condition_number", "residual"] {
        assert!(!yaml.contains(forbidden));
    }
    let decoded: CalibrationOutput = serde_yaml::from_str(&yaml).unwrap();
    decoded.validate().unwrap();
    assert_eq!(decoded.acceptance, AcceptanceStatus::PendingHumanReview);

    let decoded_pose = calibration.camera_from_reference[0].pose.to_pose().unwrap();
    for (actual, expected) in decoded_pose
        .to_homogeneous()
        .iter()
        .flatten()
        .zip(transform.to_homogeneous().iter().flatten())
    {
        assert!((actual - expected).abs() < 1.0e-12);
    }
}

#[test]
fn pure_camera_results_from_schemas_one_through_three_remain_readable() {
    for schema_version in 1..=3 {
        output(schema_version, None).validate().unwrap();
    }
}

#[test]
fn schema_four_inertial_result_round_trips_calibrated_frame_axes() {
    let gyro = [[1.0, 0.1, 0.0], [0.05, 1.0, 0.1], [0.0, -0.1, 1.0]];
    let accelerometer = [[1.02, 0.0, 0.0], [0.01, 0.98, 0.0], [-0.02, 0.03, 1.01]];
    let calibration = output(
        4,
        Some(inertial(ImuAxisCalibrationOutput {
            gyro_calibrated_from_raw: gyro,
            accelerometer_calibrated_from_raw: accelerometer,
        })),
    );
    calibration.validate().unwrap();
    let yaml = serde_yaml::to_string(&calibration).unwrap();
    let decoded: CalibrationOutput = serde_yaml::from_str(&yaml).unwrap();
    decoded.validate().unwrap();
    let decoded_inertial = decoded.inertial.unwrap();
    assert_eq!(
        decoded_inertial.axis_calibration.gyro_calibrated_from_raw,
        gyro
    );
    assert_eq!(
        decoded_inertial
            .axis_calibration
            .accelerometer_calibrated_from_raw,
        accelerometer
    );
    assert_eq!(decoded_inertial.gyro_bias_rad_s, [0.01, -0.02, 0.003]);
    assert_eq!(decoded_inertial.accelerometer_bias_m_s2, [0.1, -0.2, 0.03]);
}

#[test]
fn older_inertial_results_are_rejected() {
    for schema_version in 1..=3 {
        let calibration = output(
            schema_version,
            Some(inertial(ImuAxisCalibrationOutput::default())),
        );
        assert!(
            calibration.validate().is_err(),
            "accepted schema {schema_version} inertial result with obsolete bias semantics"
        );
    }
}

#[test]
fn obsolete_inertial_axis_field_names_are_rejected() {
    let calibration = output(4, Some(inertial(ImuAxisCalibrationOutput::default())));
    let yaml = serde_yaml::to_string(&calibration).unwrap();
    for (current, obsolete) in [
        ("gyro_calibrated_from_raw", "gyro_raw_from_physical"),
        (
            "accelerometer_calibrated_from_raw",
            "accelerometer_raw_from_physical",
        ),
    ] {
        let stale = yaml.replace(current, obsolete);
        assert!(
            serde_yaml::from_str::<CalibrationOutput>(&stale).is_err(),
            "accepted obsolete axis field {obsolete}"
        );
    }
}

#[test]
fn invalid_exported_axis_matrices_are_rejected() {
    for axis_calibration in [
        ImuAxisCalibrationOutput {
            gyro_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]],
            ..ImuAxisCalibrationOutput::default()
        },
        ImuAxisCalibrationOutput {
            accelerometer_calibrated_from_raw: [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            ..ImuAxisCalibrationOutput::default()
        },
    ] {
        assert!(
            output(4, Some(inertial(axis_calibration)))
                .validate()
                .is_err()
        );
    }
}
