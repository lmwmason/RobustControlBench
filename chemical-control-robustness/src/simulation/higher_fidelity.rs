use crate::{
    controllers::Controller,
    model::ThermalParameters,
    simulation::{SimulationConfig, SimulationResult, engine::SimulationRecord},
};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rand_distr::StandardNormal;
use std::collections::VecDeque;

/// Two-time-scale simulation: exact fluid ZOH plus a first-order sensor lag and pure sample delay.
pub fn simulate_sensor_dynamics(
    parameters: &ThermalParameters,
    controller: &mut dyn Controller,
    config: SimulationConfig,
    sensor_time_constant_s: f64,
    measurement_delay_s: f64,
) -> SimulationResult {
    let plant = parameters.discretize();
    let steps = (parameters.duration_s / parameters.sample_period_s).round() as usize;
    let delay_steps = (measurement_delay_s / parameters.sample_period_s).round() as usize;
    let sensor_factor = (-parameters.sample_period_s / sensor_time_constant_s).exp();
    let mut fluid_temperature = parameters.initial_c;
    let mut sensor_temperature = parameters.initial_c;
    let mut history = VecDeque::from(vec![sensor_temperature; delay_steps + 1]);
    let mut rng = ChaCha8Rng::seed_from_u64(config.seed);
    let mut records = Vec::with_capacity(steps + 1);
    controller.reset();
    for step in 0..=steps {
        let time = step as f64 * parameters.sample_period_s;
        let noise = if config.measurement_noise_std_c > 0.0 {
            rng.sample::<f64, _>(StandardNormal) * config.measurement_noise_std_c
        } else {
            0.0
        };
        let measured = history.front().copied().unwrap_or(sensor_temperature) + noise;
        let control = controller
            .command(parameters.reference_c, measured)
            .clamp(parameters.minimum_power_w, parameters.maximum_power_w);
        let disturbance = config.disturbance.value(time);
        records.push(SimulationRecord {
            time_s: time,
            true_temperature_c: fluid_temperature,
            measured_temperature_c: measured,
            reference_c: parameters.reference_c,
            tracking_error_c: parameters.reference_c - fluid_temperature,
            control_power_w: control,
            disturbance_power_w: disturbance,
            saturated: control <= parameters.minimum_power_w + 1.0e-9
                || control >= parameters.maximum_power_w - 1.0e-9,
        });
        if step < steps {
            let next_fluid = plant.update(fluid_temperature, control, disturbance);
            sensor_temperature = next_fluid + sensor_factor * (sensor_temperature - next_fluid);
            fluid_temperature = next_fluid;
            history.push_back(sensor_temperature);
            history.pop_front();
        }
    }
    SimulationResult {
        controller: controller.name().to_owned(),
        records,
    }
}
