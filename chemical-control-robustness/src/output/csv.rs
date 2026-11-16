use crate::{analysis::Metrics, simulation::SimulationResult};
use anyhow::Result;
use serde::Serialize;
use std::path::Path;

#[derive(Serialize)]
pub struct MetricRow<'a> {
    pub experiment: &'a str,
    pub scenario: &'a str,
    pub controller: &'a str,
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

impl<'a> MetricRow<'a> {
    pub fn new(experiment: &'a str, scenario: &'a str, controller: &'a str, m: &Metrics) -> Self {
        Self {
            experiment,
            scenario,
            controller,
            iae_c_s: m.iae_c_s,
            ise_c2_s: m.ise_c2_s,
            rmse_c: m.rmse_c,
            maximum_absolute_error_c: m.maximum_absolute_error_c,
            overshoot_c: m.overshoot_c,
            settling_time_s: m.settling_time_s,
            steady_state_error_c: m.steady_state_error_c,
            control_effort_w2_s: m.control_effort_w2_s,
            saturation_duration_s: m.saturation_duration_s,
        }
    }
}

pub fn write_trace(path: &Path, result: &SimulationResult) -> Result<()> {
    let mut writer = csv::Writer::from_path(path)?;
    for record in &result.records {
        writer.serialize(record)?;
    }
    writer.flush()?;
    Ok(())
}
