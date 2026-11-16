use super::runner::{ControllerConfig, LqiConfig, LqrConfig, PidConfig};
use crate::{
    analysis::compute_metrics,
    controllers::{Controller, LqiController, LqrController, PidController},
    model::ThermalParameters,
    simulation::{SimulationConfig, disturbances::Disturbance, engine::simulate},
};
use anyhow::Result;
use serde::Serialize;

pub const EVALUATION_BUDGET: usize = 64;

#[derive(Clone, Debug, Serialize)]
pub struct TuningRow {
    pub controller: String,
    pub candidate: usize,
    pub parameter_one: f64,
    pub parameter_two: f64,
    pub parameter_three: f64,
    pub training_score: f64,
    pub validation_score: f64,
    pub selected: bool,
}

fn radical_inverse(mut value: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut factor = 1.0 / base as f64;
    while value > 0 {
        result += factor * (value % base) as f64;
        value /= base;
        factor /= base as f64;
    }
    result
}

fn log_range(low: f64, high: f64, fraction: f64) -> f64 {
    (low.ln() + fraction * (high / low).ln()).exp()
}

fn controller_for(
    name: &str,
    design: &ControllerConfig,
    nominal: &ThermalParameters,
) -> Result<Box<dyn Controller>> {
    let d = nominal.discretize();
    Ok(match name {
        "PID" => Box::new(PidController::new(
            design.pid.kp,
            design.pid.ki,
            design.pid.kd,
            design.pid.derivative_filter_s,
            design.pid.anti_windup_gain,
            nominal.sample_period_s,
            nominal.minimum_power_w,
            nominal.maximum_power_w,
        )),
        "LQR" => Box::new(LqrController::design(
            d.state_factor,
            d.input_factor,
            design.lqr.state_weight,
            design.lqr.input_weight,
            nominal.heat_transfer_w_per_k,
            nominal.ambient_c,
            nominal.minimum_power_w,
            nominal.maximum_power_w,
        )?),
        _ => Box::new(LqiController::design(
            d.state_factor,
            d.input_factor,
            nominal.sample_period_s,
            design.lqi.state_weight,
            design.lqi.integral_weight,
            design.lqi.input_weight,
            nominal.heat_transfer_w_per_k,
            nominal.ambient_c,
            nominal.minimum_power_w,
            nominal.maximum_power_w,
        )?),
    })
}

fn objective(
    name: &str,
    design: &ControllerConfig,
    nominal: &ThermalParameters,
    validation: bool,
) -> Result<f64> {
    let mut scenarios = Vec::new();
    if validation {
        for (capacity, heat_transfer) in [(0.85, 1.0), (1.15, 1.0), (1.0, 0.85), (1.0, 1.15)] {
            let mut plant = nominal.clone();
            plant.mass_kg *= capacity;
            plant.heat_transfer_w_per_k *= heat_transfer;
            scenarios.push((
                plant,
                Disturbance::Step {
                    start_s: 1500.0,
                    power_w: -350.0,
                },
            ));
        }
    } else {
        scenarios.push((nominal.clone(), Disturbance::None));
        scenarios.push((
            nominal.clone(),
            Disturbance::Step {
                start_s: 1500.0,
                power_w: -350.0,
            },
        ));
        let mut warm_start = nominal.clone();
        warm_start.initial_c = 55.0;
        scenarios.push((warm_start, Disturbance::None));
    }
    let mut total = 0.0;
    for (plant, disturbance) in scenarios {
        let mut controller = controller_for(name, design, nominal)?;
        let result = simulate(
            &plant,
            controller.as_mut(),
            SimulationConfig {
                disturbance,
                measurement_noise_std_c: 0.0,
                seed: 20261116,
            },
        );
        let metrics = compute_metrics(&result, plant.sample_period_s);
        let horizon = plant.duration_s;
        let normalized_iae = metrics.iae_c_s / (30.0 * horizon);
        let tail_penalty = metrics.steady_state_error_c.abs() / 30.0;
        let saturation_penalty = metrics.saturation_duration_s / horizon;
        let effort = (metrics.control_effort_w2_s / horizon).sqrt() / plant.maximum_power_w;
        total += normalized_iae + 2.0 * tail_penalty + 0.15 * saturation_penalty + 0.05 * effort;
    }
    Ok(total)
}

pub(crate) fn tune_equal_budget(
    nominal: &ThermalParameters,
    original: &ControllerConfig,
) -> Result<(ControllerConfig, Vec<TuningRow>)> {
    let mut rows = Vec::new();
    let mut selected_design = original.clone();
    for name in ["PID", "LQR", "LQI"] {
        let mut candidates = Vec::new();
        for index in 1..=EVALUATION_BUDGET {
            let p1 = radical_inverse(index, 2);
            let p2 = radical_inverse(index, 3);
            let p3 = radical_inverse(index, 5);
            let mut design = original.clone();
            let parameters = match name {
                "PID" => {
                    design.pid.kp = log_range(100.0, 1200.0, p1);
                    design.pid.ki = log_range(0.2, 12.0, p2);
                    design.pid.kd = 3000.0 * p3;
                    (design.pid.kp, design.pid.ki, design.pid.kd)
                }
                "LQR" => {
                    design.lqr.state_weight = log_range(0.05, 20.0, p1);
                    design.lqr.input_weight = log_range(1.0e-7, 2.0e-3, p2);
                    (design.lqr.state_weight, design.lqr.input_weight, 0.0)
                }
                _ => {
                    design.lqi.state_weight = log_range(0.05, 20.0, p1);
                    design.lqi.integral_weight = log_range(1.0e-5, 2.0e-2, p2);
                    design.lqi.input_weight = log_range(1.0e-7, 2.0e-3, p3);
                    (
                        design.lqi.state_weight,
                        design.lqi.integral_weight,
                        design.lqi.input_weight,
                    )
                }
            };
            let training = objective(name, &design, nominal, false)?;
            candidates.push((index, design, parameters, training));
        }
        candidates.sort_by(|left, right| left.3.total_cmp(&right.3));
        let mut validation = Vec::new();
        for candidate in candidates.iter().take(8) {
            validation.push((candidate.0, objective(name, &candidate.1, nominal, true)?));
        }
        validation.sort_by(|left, right| left.1.total_cmp(&right.1));
        let winner = validation[0].0;
        for (index, design, parameters, training) in candidates {
            let validation_score = validation
                .iter()
                .find(|entry| entry.0 == index)
                .map_or(f64::NAN, |entry| entry.1);
            let selected = index == winner;
            if selected {
                match name {
                    "PID" => selected_design.pid = design.pid.clone(),
                    "LQR" => selected_design.lqr = design.lqr.clone(),
                    _ => selected_design.lqi = design.lqi.clone(),
                }
            }
            rows.push(TuningRow {
                controller: name.to_owned(),
                candidate: index,
                parameter_one: parameters.0,
                parameter_two: parameters.1,
                parameter_three: parameters.2,
                training_score: training,
                validation_score,
                selected,
            });
        }
    }
    Ok((selected_design, rows))
}

pub(crate) fn format_selected(design: &ControllerConfig) -> String {
    format!(
        "[pid]\nkp = {:.12}\nki = {:.12}\nkd = {:.12}\nderivative_filter_s = {:.12}\nanti_windup_gain = {:.12}\n\n[lqr]\nstate_weight = {:.12}\ninput_weight = {:.12}\n\n[lqi]\nstate_weight = {:.12}\nintegral_weight = {:.12}\ninput_weight = {:.12}\n",
        design.pid.kp,
        design.pid.ki,
        design.pid.kd,
        design.pid.derivative_filter_s,
        design.pid.anti_windup_gain,
        design.lqr.state_weight,
        design.lqr.input_weight,
        design.lqi.state_weight,
        design.lqi.integral_weight,
        design.lqi.input_weight
    )
}

#[allow(dead_code)]
fn _type_check(_: PidConfig, _: LqrConfig, _: LqiConfig) {}
