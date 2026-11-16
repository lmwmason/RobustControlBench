use nalgebra::DMatrix;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct StabilityRow {
    pub controller: String,
    pub capacity_percent: i32,
    pub heat_transfer_percent: i32,
    pub spectral_radius: f64,
    pub stable: bool,
}

fn spectral_radius(matrix: &DMatrix<f64>) -> f64 {
    matrix
        .complex_eigenvalues()
        .iter()
        .map(|value| value.norm())
        .fold(0.0, f64::max)
}

pub fn lqr_radius(a: f64, b: f64, gain: f64) -> f64 {
    (a - b * gain).abs()
}

pub fn lqi_radius(a: f64, b: f64, dt: f64, state_gain: f64, integral_gain: f64) -> f64 {
    spectral_radius(&DMatrix::from_row_slice(
        2,
        2,
        &[a - b * state_gain, -b * integral_gain, -dt, 1.0],
    ))
}

#[allow(clippy::too_many_arguments)]
pub fn pid_radius(a: f64, b: f64, dt: f64, kp: f64, ki: f64, kd: f64, filter_s: f64) -> f64 {
    let alpha = filter_s / (filter_s + dt);
    let derivative_factor = (1.0 - alpha) / dt;
    let matrix = DMatrix::from_row_slice(
        4,
        4,
        &[
            a + b * (-kp - ki * dt - kd * derivative_factor),
            b * ki,
            b * kd * alpha,
            b * kd * derivative_factor,
            -dt,
            1.0,
            0.0,
            0.0,
            -derivative_factor,
            0.0,
            alpha,
            derivative_factor,
            1.0,
            0.0,
            0.0,
            0.0,
        ],
    );
    spectral_radius(&matrix)
}
