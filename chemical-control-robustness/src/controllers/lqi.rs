use super::{Controller, lqr::solve_dare};
use anyhow::Result;
use nalgebra::DMatrix;

#[derive(Clone, Debug)]
pub struct LqiController {
    state_gain: f64,
    integral_gain: f64,
    integral_error: f64,
    dt: f64,
    feedforward_w: f64,
    ambient_c: f64,
    minimum: f64,
    maximum: f64,
    residual: f64,
}

impl LqiController {
    #[allow(clippy::too_many_arguments)]
    pub fn design(
        a: f64,
        b: f64,
        dt: f64,
        state_weight: f64,
        integral_weight: f64,
        input_weight: f64,
        heat_transfer_w_per_k: f64,
        ambient_c: f64,
        minimum: f64,
        maximum: f64,
    ) -> Result<Self> {
        // z[k+1] = z[k] + dt*(reference - state); regulation coordinates use x-reference.
        let augmented_a = DMatrix::from_row_slice(2, 2, &[a, 0.0, -dt, 1.0]);
        let augmented_b = DMatrix::from_column_slice(2, 1, &[b, 0.0]);
        let q = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
            state_weight,
            integral_weight,
        ]));
        let r = DMatrix::from_element(1, 1, input_weight);
        let (_, gain, residual) = solve_dare(&augmented_a, &augmented_b, &q, &r)?;
        Ok(Self {
            state_gain: gain[(0, 0)],
            integral_gain: gain[(0, 1)],
            integral_error: 0.0,
            dt,
            feedforward_w: heat_transfer_w_per_k,
            ambient_c,
            minimum,
            maximum,
            residual,
        })
    }
    pub fn gains(&self) -> (f64, f64) {
        (self.state_gain, self.integral_gain)
    }
    pub fn residual(&self) -> f64 {
        self.residual
    }
}

impl Controller for LqiController {
    fn name(&self) -> &'static str {
        "LQI"
    }
    fn reset(&mut self) {
        self.integral_error = 0.0;
    }
    fn command(&mut self, reference_c: f64, measured_c: f64) -> f64 {
        self.integral_error += (reference_c - measured_c) * self.dt;
        let command = self.feedforward_w * (reference_c - self.ambient_c)
            - self.state_gain * (measured_c - reference_c)
            - self.integral_gain * self.integral_error;
        let saturated = command.clamp(self.minimum, self.maximum);
        // Conditional integration protects the integral state during saturation.
        if command != saturated && (reference_c - measured_c) * command.signum() > 0.0 {
            self.integral_error -= (reference_c - measured_c) * self.dt;
        }
        saturated
    }
}
