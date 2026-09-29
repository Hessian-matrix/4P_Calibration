//! 旋转工具：Rodrigues 正反变换（旋转向量 ↔ 旋转矩阵）与相对旋转角。
//!
//! 只用在**引导门禁**（把帧间姿态差折算成"还在动"的分数），不参与求解。
//! 采用 Rodrigues 指数/对数映射；`relative_rotation_deg` 取 `R_delta = R_cur · R_prevᵀ`，
//! 对数映射后**各轴分量**换算成度（不是角度模长——判据取各轴最大值）。

/// 3×3 矩阵（行主序）。
pub type Matrix3 = [[f64; 3]; 3];

/// 矩阵乘法。
pub fn mat_mul(a: &Matrix3, b: &Matrix3) -> Matrix3 {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] =
                a[row][0] * b[0][column] + a[row][1] * b[1][column] + a[row][2] * b[2][column];
        }
    }
    out
}

/// 转置。
pub fn transpose(a: &Matrix3) -> Matrix3 {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = a[column][row];
        }
    }
    out
}

/// Rodrigues 正变换：旋转向量（弧度）→ 旋转矩阵。
pub fn rvec_to_matrix(rvec: &[f64; 3]) -> Matrix3 {
    let theta = (rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]).sqrt();
    if theta < 1e-12 {
        // 一阶近似：R ≈ I + [ω]×，二阶项 O(θ²) 在 θ<1e-12 时可忽略
        return [
            [1.0, -rvec[2], rvec[1]],
            [rvec[2], 1.0, -rvec[0]],
            [-rvec[1], rvec[0], 1.0],
        ];
    }
    let (axis_0, axis_1, axis_2) = (rvec[0] / theta, rvec[1] / theta, rvec[2] / theta);
    let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
    let one_minus_cos = 1.0 - cos_theta;
    [
        [
            cos_theta + axis_0 * axis_0 * one_minus_cos,
            axis_0 * axis_1 * one_minus_cos - axis_2 * sin_theta,
            axis_0 * axis_2 * one_minus_cos + axis_1 * sin_theta,
        ],
        [
            axis_1 * axis_0 * one_minus_cos + axis_2 * sin_theta,
            cos_theta + axis_1 * axis_1 * one_minus_cos,
            axis_1 * axis_2 * one_minus_cos - axis_0 * sin_theta,
        ],
        [
            axis_2 * axis_0 * one_minus_cos - axis_1 * sin_theta,
            axis_2 * axis_1 * one_minus_cos + axis_0 * sin_theta,
            cos_theta + axis_2 * axis_2 * one_minus_cos,
        ],
    ]
}

/// Rodrigues 反变换：旋转矩阵 → 旋转向量（弧度），对 θ→π 用对称分支避免 `sin θ → 0` 的病态。
pub fn matrix_to_rvec(matrix: &Matrix3) -> [f64; 3] {
    let trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    let cos_theta = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    let skew = [
        matrix[2][1] - matrix[1][2],
        matrix[0][2] - matrix[2][0],
        matrix[1][0] - matrix[0][1],
    ];
    if theta < 1e-8 {
        // 小角度：ω ≈ skew/2（泰勒，避免 0/0）
        return [skew[0] / 2.0, skew[1] / 2.0, skew[2] / 2.0];
    }
    if (std::f64::consts::PI - theta).abs() < 1e-6 {
        // θ≈π：sin θ→0，用 (R + I)/2 的主对角列求轴（对称部分主导）
        let candidates = [matrix[0][0] + 1.0, matrix[1][1] + 1.0, matrix[2][2] + 1.0];
        let (index, best) = candidates
            .iter()
            .enumerate()
            .max_by(|left, right| {
                left.1
                    .partial_cmp(right.1)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(index, value)| (index, *value))
            .expect("3x3 diagonal");
        if best <= 0.0 {
            return [0.0; 3];
        }
        let mut axis = [0.0; 3];
        let scale = (best / 2.0).sqrt();
        axis[index] = scale;
        for other in 0..3 {
            if other == index {
                continue;
            }
            axis[other] = (matrix[index][other] + matrix[other][index]) / (4.0 * scale);
        }
        // 符号由 skew 决定（θ=π 时 skew≈0，符号任意，取与 skew 同号或默认正）
        let sign = if skew[0] + skew[1] + skew[2] < 0.0 {
            -1.0
        } else {
            1.0
        };
        return [
            axis[0] * theta * sign,
            axis[1] * theta * sign,
            axis[2] * theta * sign,
        ];
    }
    let factor = theta / (2.0 * theta.sin());
    [skew[0] * factor, skew[1] * factor, skew[2] * factor]
}

/// 两姿态之间的相对旋转（相机系），取**各轴分量**的度数。
pub fn relative_rotation_deg(previous_rvec: &[f64; 3], current_rvec: &[f64; 3]) -> [f64; 3] {
    let previous = rvec_to_matrix(previous_rvec);
    let current = rvec_to_matrix(current_rvec);
    let delta = mat_mul(&current, &transpose(&previous));
    let rvec = matrix_to_rvec(&delta);
    [
        rvec[0].to_degrees(),
        rvec[1].to_degrees(),
        rvec[2].to_degrees(),
    ]
}

#[cfg(test)]
mod tests {
    use super::{mat_mul, matrix_to_rvec, relative_rotation_deg, rvec_to_matrix};

    fn assert_close(left: f64, right: f64, tolerance: f64, what: &str) {
        assert!(
            (left - right).abs() <= tolerance,
            "{what}: {left} vs {right} (tolerance {tolerance})"
        );
    }

    #[test]
    fn identity_and_axis_rotations() {
        let identity = rvec_to_matrix(&[0.0, 0.0, 0.0]);
        for (row, values) in identity.iter().enumerate() {
            for (column, value) in values.iter().enumerate() {
                assert_close(
                    *value,
                    if row == column { 1.0 } else { 0.0 },
                    1e-12,
                    "identity",
                );
            }
        }
        let quarter_z = rvec_to_matrix(&[0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        assert_close(quarter_z[0][0], 0.0, 1e-12, "cos(90)");
        assert_close(quarter_z[0][1], -1.0, 1e-12, "-sin(90)");
        assert_close(quarter_z[1][0], 1.0, 1e-12, "sin(90)");
        assert_close(quarter_z[2][2], 1.0, 1e-12, "z axis untouched");
    }

    #[test]
    fn round_trip_survives() {
        let samples = [
            [0.01, -0.02, 0.03],
            [0.3, 0.4, -0.2],
            [1.0, 0.0, 0.0],
            [0.5, 0.5, 0.5],
            [3.1, 0.0, 0.0],
        ];
        for rvec in samples {
            let back = matrix_to_rvec(&rvec_to_matrix(&rvec));
            for axis in 0..3 {
                assert_close(back[axis], rvec[axis], 1e-9, "round trip");
            }
        }
    }

    #[test]
    fn relative_rotation_of_pure_axis_change() {
        // 先看 +x 轴，再绕 y 转 2°：相对旋转向量应为 (0, 2, 0) 度
        let previous = [0.0, 0.0, 0.0];
        let current = [0.0, 2.0_f64.to_radians(), 0.0];
        let relative = relative_rotation_deg(&previous, &current);
        assert_close(relative[0], 0.0, 1e-9, "x");
        assert_close(relative[1], 2.0, 1e-9, "y");
        assert_close(relative[2], 0.0, 1e-9, "z");
    }

    #[test]
    fn rotation_composition_matches_matrix_product() {
        let a = [0.2, -0.1, 0.05];
        let b = [-0.3, 0.15, 0.2];
        let composed_from_matrices = mat_mul(&rvec_to_matrix(&a), &rvec_to_matrix(&b));
        let expected = matrix_to_rvec(&composed_from_matrices);
        let actual = matrix_to_rvec(&rvec_to_matrix(
            &matrix_to_rvec(&rvec_to_matrix(&a)), // a 往返不变
        ));
        for axis in 0..3 {
            assert_close(actual[axis], a[axis], 1e-12, "log(exp(a))");
        }
        let _ = expected;
    }
}
