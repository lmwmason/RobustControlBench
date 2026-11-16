use super::scenarios::ExperimentsConfig;
use crate::{
    analysis::{
        Metrics, compute_metrics,
        statistics::{mean, sample_standard_deviation},
    },
    controllers::{LqiController, LqrController, PidController},
    model::ThermalParameters,
    output::{
        charts,
        csv::{self, MetricRow},
    },
    read_toml,
    simulation::{SimulationConfig, SimulationResult, disturbances::Disturbance, engine::simulate},
};
use anyhow::Result;
use serde::Deserialize;
use std::{collections::BTreeMap, fs, path::Path};

#[derive(Deserialize)]
struct ControllerConfig {
    pid: PidConfig,
    lqr: LqrConfig,
    lqi: LqiConfig,
}
#[derive(Deserialize)]
struct PidConfig {
    kp: f64,
    ki: f64,
    kd: f64,
    derivative_filter_s: f64,
    anti_windup_gain: f64,
}
#[derive(Deserialize)]
struct LqrConfig {
    state_weight: f64,
    input_weight: f64,
}
#[derive(Deserialize)]
struct LqiConfig {
    state_weight: f64,
    integral_weight: f64,
    input_weight: f64,
}

struct Outcome {
    experiment: String,
    scenario: String,
    result: SimulationResult,
    metrics: Metrics,
}

fn controllers(
    p: &ThermalParameters,
    c: &ControllerConfig,
) -> Result<Vec<Box<dyn crate::controllers::Controller>>> {
    let d = p.discretize();
    Ok(vec![
        Box::new(PidController::new(
            c.pid.kp,
            c.pid.ki,
            c.pid.kd,
            c.pid.derivative_filter_s,
            c.pid.anti_windup_gain,
            p.sample_period_s,
            p.minimum_power_w,
            p.maximum_power_w,
        )),
        Box::new(LqrController::design(
            d.state_factor,
            d.input_factor,
            c.lqr.state_weight,
            c.lqr.input_weight,
            p.heat_transfer_w_per_k,
            p.ambient_c,
            p.minimum_power_w,
            p.maximum_power_w,
        )?),
        Box::new(LqiController::design(
            d.state_factor,
            d.input_factor,
            p.sample_period_s,
            c.lqi.state_weight,
            c.lqi.integral_weight,
            c.lqi.input_weight,
            p.heat_transfer_w_per_k,
            p.ambient_c,
            p.minimum_power_w,
            p.maximum_power_w,
        )?),
    ])
}

fn run_set(
    outcomes: &mut Vec<Outcome>,
    nominal: &ThermalParameters,
    design: &ControllerConfig,
    plant: ThermalParameters,
    experiment: &str,
    scenario: &str,
    simulation: SimulationConfig,
) -> Result<()> {
    for mut controller in controllers(nominal, design)? {
        let result = simulate(&plant, controller.as_mut(), simulation);
        let metrics = compute_metrics(&result, plant.sample_period_s);
        outcomes.push(Outcome {
            experiment: experiment.to_owned(),
            scenario: scenario.to_owned(),
            result,
            metrics,
        });
    }
    Ok(())
}

pub fn run_all(root: &Path) -> Result<()> {
    let nominal: ThermalParameters = read_toml(root.join("configs/nominal.toml"))?;
    nominal.validate().map_err(anyhow::Error::msg)?;
    let design: ControllerConfig = read_toml(root.join("configs/controllers.toml"))?;
    let experiments: ExperimentsConfig = read_toml(root.join("configs/experiments.toml"))?;
    let data = root.join("results/data");
    let figures = root.join("results/figures");
    let summaries = root.join("results/summaries");
    fs::create_dir_all(&data)?;
    fs::create_dir_all(&figures)?;
    fs::create_dir_all(&summaries)?;
    let base = SimulationConfig {
        disturbance: Disturbance::None,
        measurement_noise_std_c: 0.0,
        seed: experiments.base_seed,
    };
    let mut outcomes = Vec::new();
    run_set(
        &mut outcomes,
        &nominal,
        &design,
        nominal.clone(),
        "nominal",
        "baseline",
        base,
    )?;
    for percent in &experiments.uncertainty_percent {
        let mut plant = nominal.clone();
        plant.mass_kg *= 1.0 + *percent as f64 / 100.0;
        run_set(
            &mut outcomes,
            &nominal,
            &design,
            plant,
            "capacity",
            &format!("{percent:+}%"),
            base,
        )?;
        let mut plant = nominal.clone();
        plant.heat_transfer_w_per_k *= 1.0 + *percent as f64 / 100.0;
        run_set(
            &mut outcomes,
            &nominal,
            &design,
            plant,
            "heat_transfer",
            &format!("{percent:+}%"),
            base,
        )?;
    }
    let disturbed = SimulationConfig {
        disturbance: Disturbance::Step {
            start_s: experiments.disturbance_start_s,
            power_w: experiments.disturbance_power_w,
        },
        ..base
    };
    run_set(
        &mut outcomes,
        &nominal,
        &design,
        nominal.clone(),
        "disturbance",
        "negative_step",
        disturbed,
    )?;
    let mut saturated = nominal.clone();
    saturated.maximum_power_w = 1200.0;
    saturated.minimum_power_w = -500.0;
    run_set(
        &mut outcomes,
        &nominal,
        &design,
        saturated,
        "saturation",
        "restricted_actuator",
        base,
    )?;
    for scenario in &experiments.combined {
        let mut plant = nominal.clone();
        plant.mass_kg *= 1.0 + scenario.capacity_percent as f64 / 100.0;
        plant.heat_transfer_w_per_k *= 1.0 + scenario.heat_transfer_percent as f64 / 100.0;
        plant.ambient_c *= 1.0 + scenario.ambient_percent as f64 / 100.0;
        run_set(
            &mut outcomes,
            &nominal,
            &design,
            plant,
            "combined",
            &scenario.name,
            disturbed,
        )?;
    }
    for replicate in 0..experiments.noise_replicates {
        run_set(
            &mut outcomes,
            &nominal,
            &design,
            nominal.clone(),
            "noise",
            &format!("seed_{replicate:02}"),
            SimulationConfig {
                measurement_noise_std_c: experiments.noise_standard_deviation_c,
                seed: experiments.base_seed + replicate as u64,
                ..base
            },
        )?;
    }
    let mut metric_writer = ::csv::Writer::from_path(data.join("metrics.csv"))?;
    for o in &outcomes {
        metric_writer.serialize(MetricRow::new(
            &o.experiment,
            &o.scenario,
            &o.result.controller,
            &o.metrics,
        ))?;
    }
    metric_writer.flush()?;
    for o in outcomes.iter().filter(|o| o.scenario == "baseline") {
        csv::write_trace(
            &data.join(format!(
                "nominal_{}.csv",
                o.result.controller.to_lowercase()
            )),
            &o.result,
        )?;
    }
    let selected = |experiment: &str, scenario: &str| {
        outcomes
            .iter()
            .filter(|o| o.experiment == experiment && o.scenario == scenario)
            .map(|o| o.result.clone())
            .collect::<Vec<_>>()
    };
    charts::trace_chart(
        &figures.join("nominal-temperature.svg"),
        "Nominal Temperature Response",
        "Temperature (°C)",
        &selected("nominal", "baseline"),
        false,
    )?;
    charts::trace_chart(
        &figures.join("controller-input.svg"),
        "Nominal Controller Input",
        "Thermal power (W)",
        &selected("nominal", "baseline"),
        true,
    )?;
    charts::trace_chart(
        &figures.join("capacity-uncertainty.svg"),
        "Thermal-Capacity Uncertainty (-30%)",
        "Temperature (°C)",
        &selected("capacity", "-30%"),
        false,
    )?;
    charts::trace_chart(
        &figures.join("heat-transfer-uncertainty.svg"),
        "Heat-Transfer Uncertainty (+30%)",
        "Temperature (°C)",
        &selected("heat_transfer", "+30%"),
        false,
    )?;
    charts::trace_chart(
        &figures.join("disturbance-rejection.svg"),
        "Disturbance Rejection (-700 W at 1800 s)",
        "Temperature (°C)",
        &selected("disturbance", "negative_step"),
        false,
    )?;
    charts::trace_chart(
        &figures.join("measurement-noise.svg"),
        "Measurement Noise (First Seed)",
        "Temperature (°C)",
        &selected("noise", "seed_00"),
        false,
    )?;
    charts::trace_chart(
        &figures.join("actuator-saturation.svg"),
        "Restricted Actuator Response",
        "Thermal power (W)",
        &selected("saturation", "restricted_actuator"),
        true,
    )?;
    let nominal_points = outcomes
        .iter()
        .filter(|o| o.experiment == "nominal")
        .map(|o| {
            (
                o.result.controller.clone(),
                String::new(),
                o.metrics.iae_c_s,
            )
        })
        .collect::<Vec<_>>();
    charts::summary_chart(
        &figures.join("tracking-error-comparison.svg"),
        "Nominal Tracking Error",
        &nominal_points,
        "IAE (°C·s)",
    )?;
    let robust_points = outcomes
        .iter()
        .filter(|o| ["capacity", "heat_transfer", "combined"].contains(&o.experiment.as_str()))
        .map(|o| {
            (
                o.result.controller.clone(),
                o.scenario.clone(),
                o.metrics.rmse_c,
            )
        })
        .collect::<Vec<_>>();
    charts::summary_chart(
        &figures.join("robustness-summary.svg"),
        "Robustness Across Uncertain Plants",
        &robust_points,
        "RMSE (°C)",
    )?;
    let mut grouped: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    for o in outcomes.iter().filter(|o| o.experiment == "noise") {
        grouped
            .entry(o.result.controller.clone())
            .or_default()
            .push(o.metrics.rmse_c);
    }
    let mut summary = String::from("controller,mean_rmse_c,standard_deviation_rmse_c\n");
    for (name, values) in grouped {
        summary.push_str(&format!(
            "{name},{:.8},{:.8}\n",
            mean(&values),
            sample_standard_deviation(&values)
        ));
    }
    fs::write(summaries.join("noise-statistics.csv"), summary)?;
    let nominal_text = outcomes
        .iter()
        .filter(|o| o.experiment == "nominal")
        .map(|o| {
            format!(
                "{}: IAE {:.2} °C·s, RMSE {:.3} °C, settling {}\n",
                o.result.controller,
                o.metrics.iae_c_s,
                o.metrics.rmse_c,
                o.metrics
                    .settling_time_s
                    .map_or_else(|| "not reached".to_owned(), |v| format!("{v:.0} s"))
            )
        })
        .collect::<String>();
    fs::write(summaries.join("key-results.txt"), nominal_text)?;
    Ok(())
}
