use super::scenarios::ExperimentsConfig;
use crate::{
    analysis::{
        Metrics, compute_metrics,
        robustness::{StabilityRow, lqi_radius, lqr_radius, pid_radius},
        statistics::{mean, paired_summary, sample_standard_deviation},
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

#[derive(Clone, Deserialize)]
struct ControllerConfig {
    pid: PidConfig,
    lqr: LqrConfig,
    lqi: LqiConfig,
}
#[derive(Clone, Deserialize)]
struct PidConfig {
    kp: f64,
    ki: f64,
    kd: f64,
    derivative_filter_s: f64,
    anti_windup_gain: f64,
}
#[derive(Clone, Deserialize)]
struct LqrConfig {
    state_weight: f64,
    input_weight: f64,
}
#[derive(Clone, Deserialize)]
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
    actuator: &ThermalParameters,
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
            actuator.minimum_power_w,
            actuator.maximum_power_w,
        )),
        Box::new(LqrController::design(
            d.state_factor,
            d.input_factor,
            c.lqr.state_weight,
            c.lqr.input_weight,
            p.heat_transfer_w_per_k,
            p.ambient_c,
            actuator.minimum_power_w,
            actuator.maximum_power_w,
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
            actuator.minimum_power_w,
            actuator.maximum_power_w,
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
    for mut controller in controllers(nominal, &plant, design)? {
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
    for scale in [0.8, 1.2] {
        let mut sensitivity = design.clone();
        sensitivity.pid.kp *= scale;
        sensitivity.pid.ki *= scale;
        sensitivity.pid.kd *= scale;
        sensitivity.lqr.state_weight *= scale;
        sensitivity.lqi.state_weight *= scale;
        sensitivity.lqi.integral_weight *= scale;
        run_set(
            &mut outcomes,
            &nominal,
            &sensitivity,
            nominal.clone(),
            "tuning_sensitivity",
            &format!("gain_scale_{scale:.1}"),
            base,
        )?;
    }
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
        plant.ambient_c += scenario.ambient_offset_c;
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
    for (experiment, filename, title) in [
        (
            "capacity",
            "capacity-uncertainty.svg",
            "Thermal-Capacity Uncertainty Sweep",
        ),
        (
            "heat_transfer",
            "heat-transfer-uncertainty.svg",
            "Heat-Transfer Uncertainty Sweep",
        ),
    ] {
        let mut series = Vec::new();
        for controller in ["PID", "LQR", "LQI"] {
            let mut points = outcomes
                .iter()
                .filter(|o| o.experiment == experiment && o.result.controller == controller)
                .map(|o| {
                    (
                        o.scenario.trim_end_matches('%').parse::<f64>().unwrap(),
                        o.metrics.rmse_c,
                    )
                })
                .collect::<Vec<_>>();
            points.sort_by(|left, right| left.0.total_cmp(&right.0));
            series.push((controller.to_owned(), points));
        }
        charts::uncertainty_sweep_chart(&figures.join(filename), title, &series)?;
    }
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

    // Noise analysis uses the final 1200 s, avoiding domination by deterministic start-up.
    let mut noise_tail: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    let mut replicate_writer = ::csv::Writer::from_path(data.join("noise-replicates.csv"))?;
    replicate_writer.write_record(["scenario", "controller", "tail_rmse_c"])?;
    for o in outcomes.iter().filter(|o| o.experiment == "noise") {
        let tail = o.result.records.iter().filter(|r| r.time_s >= 2400.0);
        let values: Vec<f64> = tail.map(|r| r.tracking_error_c).collect();
        let tail_rmse = (values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt();
        replicate_writer.write_record([
            &o.scenario,
            &o.result.controller,
            &format!("{tail_rmse:.12}"),
        ])?;
        noise_tail
            .entry(o.result.controller.clone())
            .or_default()
            .push(tail_rmse);
    }
    replicate_writer.flush()?;
    let mut paired_writer =
        ::csv::Writer::from_path(summaries.join("noise-paired-comparisons.csv"))?;
    paired_writer.write_record([
        "left",
        "right",
        "n",
        "mean_difference_c",
        "sd_difference_c",
        "ci95_low_c",
        "ci95_high_c",
        "cohen_dz",
    ])?;
    for (left, right) in [("LQI", "PID"), ("LQI", "LQR"), ("PID", "LQR")] {
        let stats = paired_summary(&noise_tail[left], &noise_tail[right]);
        paired_writer.write_record([
            left,
            right,
            &noise_tail[left].len().to_string(),
            &format!("{:.12}", stats.mean_difference),
            &format!("{:.12}", stats.standard_deviation),
            &format!("{:.12}", stats.ci95_low),
            &format!("{:.12}", stats.ci95_high),
            &format!("{:.8}", stats.standardized_effect),
        ])?;
    }
    paired_writer.flush()?;

    let disturbance_outcomes: Vec<&Outcome> = outcomes
        .iter()
        .filter(|o| o.experiment == "disturbance")
        .collect();
    let mut disturbance_writer =
        ::csv::Writer::from_path(summaries.join("disturbance-recovery.csv"))?;
    disturbance_writer.write_record([
        "controller",
        "post_step_iae_c_s",
        "maximum_post_step_error_c",
        "recovery_time_s",
        "tail_error_c",
    ])?;
    for o in disturbance_outcomes {
        let post: Vec<_> = o
            .result
            .records
            .iter()
            .filter(|r| r.time_s >= experiments.disturbance_start_s)
            .collect();
        let post_iae = post
            .iter()
            .take(post.len().saturating_sub(1))
            .map(|r| r.tracking_error_c.abs() * nominal.sample_period_s)
            .sum::<f64>();
        let maximum = post
            .iter()
            .map(|r| r.tracking_error_c.abs())
            .fold(0.0, f64::max);
        let recovery = post
            .iter()
            .enumerate()
            .find(|(i, _)| post[*i..].iter().all(|r| r.tracking_error_c.abs() <= 0.6))
            .map(|(_, r)| r.time_s - experiments.disturbance_start_s);
        disturbance_writer.write_record([
            &o.result.controller,
            &format!("{post_iae:.9}"),
            &format!("{maximum:.9}"),
            &recovery.map_or_else(String::new, |v| format!("{v:.1}")),
            &format!("{:.12}", o.metrics.steady_state_error_c),
        ])?;
    }
    disturbance_writer.flush()?;

    let nominal_discrete = nominal.discretize();
    let lqr = LqrController::design(
        nominal_discrete.state_factor,
        nominal_discrete.input_factor,
        design.lqr.state_weight,
        design.lqr.input_weight,
        nominal.heat_transfer_w_per_k,
        nominal.ambient_c,
        nominal.minimum_power_w,
        nominal.maximum_power_w,
    )?;
    let lqi = LqiController::design(
        nominal_discrete.state_factor,
        nominal_discrete.input_factor,
        nominal.sample_period_s,
        design.lqi.state_weight,
        design.lqi.integral_weight,
        design.lqi.input_weight,
        nominal.heat_transfer_w_per_k,
        nominal.ambient_c,
        nominal.minimum_power_w,
        nominal.maximum_power_w,
    )?;
    let (lqi_state, lqi_integral) = lqi.gains();
    let mut stability_writer =
        ::csv::Writer::from_path(summaries.join("linear-stability-grid.csv"))?;
    for capacity_percent in -30..=30 {
        for heat_transfer_percent in -30..=30 {
            let mut actual = nominal.clone();
            actual.mass_kg *= 1.0 + capacity_percent as f64 / 100.0;
            actual.heat_transfer_w_per_k *= 1.0 + heat_transfer_percent as f64 / 100.0;
            let discrete = actual.discretize();
            for (controller, radius) in [
                (
                    "PID",
                    pid_radius(
                        discrete.state_factor,
                        discrete.input_factor,
                        nominal.sample_period_s,
                        design.pid.kp,
                        design.pid.ki,
                        design.pid.kd,
                        design.pid.derivative_filter_s,
                    ),
                ),
                (
                    "LQR",
                    lqr_radius(discrete.state_factor, discrete.input_factor, lqr.gain()),
                ),
                (
                    "LQI",
                    lqi_radius(
                        discrete.state_factor,
                        discrete.input_factor,
                        nominal.sample_period_s,
                        lqi_state,
                        lqi_integral,
                    ),
                ),
            ] {
                stability_writer.serialize(StabilityRow {
                    controller: controller.to_owned(),
                    capacity_percent,
                    heat_transfer_percent,
                    spectral_radius: radius,
                    stable: radius < 1.0,
                })?;
            }
        }
    }
    stability_writer.flush()?;
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
