use crate::{
    controllers::Controller, model::ThermalParameters, simulation::disturbances::Disturbance,
};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rand_distr::StandardNormal;
use serde::Serialize;

#[derive(Clone, Copy, Debug)]
pub struct SimulationConfig {
    pub disturbance: Disturbance,
    pub measurement_noise_std_c: f64,
    pub seed: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct SimulationRecord {
    pub time_s: f64,
    pub true_temperature_c: f64,
    pub measured_temperature_c: f64,
    pub reference_c: f64,
    pub tracking_error_c: f64,
    pub control_power_w: f64,
    pub disturbance_power_w: f64,
    pub saturated: bool,
}

#[derive(Clone, Debug)]
pub struct SimulationResult {
    pub controller: String,
    pub records: Vec<SimulationRecord>,
}

pub fn simulate(
    parameters: &ThermalParameters,
    controller: &mut dyn Controller,
    config: SimulationConfig,
) -> SimulationResult {
    let plant = parameters.discretize();
    let steps = (parameters.duration_s / parameters.sample_period_s).round() as usize;
    let mut rng = ChaCha8Rng::seed_from_u64(config.seed);
    let mut temperature = parameters.initial_c;
    let mut records = Vec::with_capacity(steps + 1);
    controller.reset();
    for step in 0..=steps {
        let time = step as f64 * parameters.sample_period_s;
        let noise: f64 = if config.measurement_noise_std_c > 0.0 {
            rng.sample::<f64, _>(StandardNormal) * config.measurement_noise_std_c
        } else {
            0.0
        };
        let measured = temperature + noise;
        let requested = controller.command(parameters.reference_c, measured);
        let control = requested.clamp(parameters.minimum_power_w, parameters.maximum_power_w);
        let disturbance = config.disturbance.value(time);
        records.push(SimulationRecord {
            time_s: time,
            true_temperature_c: temperature,
            measured_temperature_c: measured,
            reference_c: parameters.reference_c,
            tracking_error_c: parameters.reference_c - temperature,
            control_power_w: control,
            disturbance_power_w: disturbance,
            saturated: control <= parameters.minimum_power_w + 1.0e-9
                || control >= parameters.maximum_power_w - 1.0e-9,
        });
        if step < steps {
            temperature = plant.update(temperature, control, disturbance);
        }
    }
    SimulationResult {
        controller: controller.name().to_owned(),
        records,
    }
}
