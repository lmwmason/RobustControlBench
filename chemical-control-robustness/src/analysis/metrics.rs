use crate::simulation::SimulationResult;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct Metrics {
    pub iae_c_s: f64,
    pub ise_c2_s: f64,
    pub rmse_c: f64,
    pub maximum_absolute_error_c: f64,
    pub overshoot_c: f64,
    pub settling_time_s: Option<f64>,
    pub steady_state_error_c: f64,
    pub control_effort_w2_s: f64,
    pub saturation_duration_s: f64,
}

pub fn compute_metrics(result: &SimulationResult, dt: f64) -> Metrics {
    let records = &result.records;
    let errors: Vec<f64> = records.iter().map(|r| r.tracking_error_c).collect();
    // N+1 endpoint samples describe N held intervals; use left rectangles.
    let interval_records = &records[..records.len().saturating_sub(1)];
    let interval_errors = &errors[..errors.len().saturating_sub(1)];
    let iae = interval_errors.iter().map(|e| e.abs() * dt).sum();
    let ise = interval_errors.iter().map(|e| e * e * dt).sum();
    let max_error = errors.iter().map(|e| e.abs()).fold(0.0, f64::max);
    let reference = records[0].reference_c;
    let overshoot = records
        .iter()
        .map(|r| (r.true_temperature_c - reference).max(0.0))
        .fold(0.0, f64::max);
    let tolerance = (0.02 * (reference - records[0].true_temperature_c).abs()).max(0.1);
    let settling_time = records
        .iter()
        .enumerate()
        .find(|(index, _)| {
            records[*index..]
                .iter()
                .all(|r| r.tracking_error_c.abs() <= tolerance)
        })
        .map(|(_, r)| r.time_s);
    let tail = (records.len() / 10).max(1);
    let steady_state_error = errors[errors.len() - tail..].iter().sum::<f64>() / tail as f64;
    Metrics {
        iae_c_s: iae,
        ise_c2_s: ise,
        rmse_c: (ise / (interval_records.len() as f64 * dt)).sqrt(),
        maximum_absolute_error_c: max_error,
        overshoot_c: overshoot,
        settling_time_s: settling_time,
        steady_state_error_c: steady_state_error,
        control_effort_w2_s: interval_records
            .iter()
            .map(|r| r.control_power_w.powi(2) * dt)
            .sum(),
        saturation_duration_s: interval_records.iter().filter(|r| r.saturated).count() as f64 * dt,
    }
}
