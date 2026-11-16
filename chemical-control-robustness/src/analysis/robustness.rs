use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct StabilityRow {
    pub controller: String,
    pub capacity_percent: i32,
    pub heat_transfer_percent: i32,
    pub spectral_radius: f64,
    pub stable: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct IntervalCertificate {
    pub a_min: f64,
    pub a_max: f64,
    pub b_min: f64,
    pub b_max: f64,
}

impl IntervalCertificate {
    pub fn thermal(dt: f64, capacity_nominal: f64, heat_transfer_nominal: f64) -> Self {
        let capacity_min = 0.7 * capacity_nominal;
        let capacity_max = 1.3 * capacity_nominal;
        let heat_min = 0.7 * heat_transfer_nominal;
        let heat_max = 1.3 * heat_transfer_nominal;
        let a_min = (-heat_max * dt / capacity_min).exp();
        let a_max = (-heat_min * dt / capacity_max).exp();
        let b_min = (1.0 - (-heat_max * dt / capacity_max).exp()) / heat_max;
        let b_max = (1.0 - (-heat_min * dt / capacity_min).exp()) / heat_min;
        Self {
            a_min,
            a_max,
            b_min,
            b_max,
        }
    }

    fn bounded(
        dt: f64,
        capacity_min: f64,
        capacity_max: f64,
        heat_min: f64,
        heat_max: f64,
    ) -> Self {
        Self {
            a_min: (-heat_max * dt / capacity_min).exp(),
            a_max: (-heat_min * dt / capacity_max).exp(),
            b_min: (1.0 - (-heat_max * dt / capacity_max).exp()) / heat_max,
            b_max: (1.0 - (-heat_min * dt / capacity_min).exp()) / heat_min,
        }
    }

    pub fn lqr_margin(self, gain: f64) -> f64 {
        let lower = self.a_min - self.b_max * gain;
        let upper = self.a_max - self.b_min * gain;
        1.0 - lower.abs().max(upper.abs())
    }

    /// Conservative second-order Jury margins for LQI using independent A/B intervals.
    pub fn lqi_jury_margins(self, dt: f64, state_gain: f64, integral_gain: f64) -> [f64; 3] {
        let x_min = self.a_min - self.b_max * state_gain;
        let x_max = self.a_max - self.b_min * state_gain;
        let integral_term_min = self.b_min * (-integral_gain) * dt;
        let integral_term_max = self.b_max * (-integral_gain) * dt;
        let trace_min = x_min + 1.0;
        let trace_max = x_max + 1.0;
        let determinant_min = x_min + integral_term_min;
        let determinant_max = x_max + integral_term_max;
        [
            1.0 - trace_max + determinant_min,
            1.0 + trace_min + determinant_min,
            1.0 - determinant_max,
        ]
    }
}

pub fn subdivided_lqi_certificate(
    dt: f64,
    capacity_nominal: f64,
    heat_nominal: f64,
    state_gain: f64,
    integral_gain: f64,
    subdivisions: usize,
) -> [f64; 3] {
    let mut minima = [f64::INFINITY; 3];
    for capacity_index in 0..subdivisions {
        for heat_index in 0..subdivisions {
            let fraction = |index: usize| 0.7 + 0.6 * index as f64 / subdivisions as f64;
            let interval = IntervalCertificate::bounded(
                dt,
                capacity_nominal * fraction(capacity_index),
                capacity_nominal * fraction(capacity_index + 1),
                heat_nominal * fraction(heat_index),
                heat_nominal * fraction(heat_index + 1),
            );
            let margins = interval.lqi_jury_margins(dt, state_gain, integral_gain);
            for index in 0..3 {
                minima[index] = minima[index].min(margins[index]);
            }
        }
    }
    minima
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
