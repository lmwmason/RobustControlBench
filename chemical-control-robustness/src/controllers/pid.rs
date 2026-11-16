use super::Controller;

#[derive(Clone, Debug)]
pub struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    dt: f64,
    derivative_alpha: f64,
    anti_windup_gain: f64,
    minimum: f64,
    maximum: f64,
    integral: f64,
    previous_measurement: Option<f64>,
    filtered_derivative: f64,
}

impl PidController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        filter_s: f64,
        anti_windup_gain: f64,
        dt: f64,
        minimum: f64,
        maximum: f64,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            dt,
            derivative_alpha: filter_s / (filter_s + dt),
            anti_windup_gain,
            minimum,
            maximum,
            integral: 0.0,
            previous_measurement: None,
            filtered_derivative: 0.0,
        }
    }
}

impl Controller for PidController {
    fn name(&self) -> &'static str {
        "PID"
    }
    fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_measurement = None;
        self.filtered_derivative = 0.0;
    }
    fn command(&mut self, reference_c: f64, measured_c: f64) -> f64 {
        let error = reference_c - measured_c;
        let raw_derivative = self
            .previous_measurement
            .map_or(0.0, |last| -(measured_c - last) / self.dt);
        self.filtered_derivative = self.derivative_alpha * self.filtered_derivative
            + (1.0 - self.derivative_alpha) * raw_derivative;
        self.integral += error * self.dt;
        let unsaturated =
            self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative;
        let saturated = unsaturated.clamp(self.minimum, self.maximum);
        if self.ki.abs() > 1.0e-12 {
            self.integral += self.anti_windup_gain * (saturated - unsaturated) * self.dt / self.ki;
        }
        self.previous_measurement = Some(measured_c);
        saturated
    }
}
