use super::scenarios::ExperimentsConfig;
use super::tuning::{format_selected, tune_equal_budget};
use crate::{
    analysis::{
        Metrics, compute_metrics,
        robustness::{
            IntervalCertificate, StabilityRow, lqi_radius, lqr_radius, pid_radius,
            subdivided_lqi_certificate,
        },
        statistics::{mean, paired_summary, sample_standard_deviation},
    },
    controllers::{LqiController, LqrController, PidController},
    model::ThermalParameters,
    output::{
        charts,
        csv::{self, MetricRow},
    },
    read_toml,
    simulation::{
        SimulationConfig, SimulationResult, disturbances::Disturbance, engine::simulate,
        higher_fidelity::simulate_sensor_dynamics,
    },
};
use anyhow::Result;
use serde::Deserialize;
use std::{collections::BTreeMap, fs, path::Path};

#[derive(Clone, Deserialize)]
pub(crate) struct ControllerConfig {
    pub(crate) pid: PidConfig,
    pub(crate) lqr: LqrConfig,
    pub(crate) lqi: LqiConfig,
}
#[derive(Clone, Deserialize)]
pub(crate) struct PidConfig {
    pub(crate) kp: f64,
    pub(crate) ki: f64,
    pub(crate) kd: f64,
    pub(crate) derivative_filter_s: f64,
    pub(crate) anti_windup_gain: f64,
}
#[derive(Clone, Deserialize)]
pub(crate) struct LqrConfig {
    pub(crate) state_weight: f64,
    pub(crate) input_weight: f64,
}
#[derive(Clone, Deserialize)]
pub(crate) struct LqiConfig {
    pub(crate) state_weight: f64,
    pub(crate) integral_weight: f64,
    pub(crate) input_weight: f64,
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
    let (tuned_design, tuning_rows) = tune_equal_budget(&nominal, &design)?;
    let mut tuning_writer = ::csv::Writer::from_path(data.join("tuning-study.csv"))?;
    for row in &tuning_rows {
        tuning_writer.serialize(row)?;
    }
    tuning_writer.flush()?;
    fs::write(
        summaries.join("selected-tuned-controllers.toml"),
        format_selected(&tuned_design),
    )?;
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
    run_set(
        &mut outcomes,
        &nominal,
        &tuned_design,
        nominal.clone(),
        "retuned_nominal",
        "training_scenario_comparison",
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
    run_set(
        &mut outcomes,
        &nominal,
        &tuned_design,
        nominal.clone(),
        "retuned_disturbance",
        "held_out_negative_step",
        disturbed,
    )?;
    let mut saturated = nominal.clone();
    saturated.maximum_power_w = 1200.0;
    saturated.minimum_power_w = -500.0;
    run_set(
        &mut outcomes,
        &nominal,
        &design,
        saturated.clone(),
        "saturation",
        "restricted_actuator",
        base,
    )?;
    run_set(
        &mut outcomes,
        &nominal,
        &tuned_design,
        saturated.clone(),
        "retuned_saturation",
        "held_out_restricted_actuator",
        base,
    )?;

    for (label, controller_design) in [("original", &design), ("retuned", &tuned_design)] {
        for mut controller in controllers(&nominal, &nominal, controller_design)? {
            let result = simulate_sensor_dynamics(&nominal, controller.as_mut(), base, 20.0, 10.0);
            let metrics = compute_metrics(&result, nominal.sample_period_s);
            outcomes.push(Outcome {
                experiment: "higher_fidelity".to_owned(),
                scenario: label.to_owned(),
                result,
                metrics,
            });
        }
    }
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
    let tuning_points = outcomes
        .iter()
        .filter(|o| o.experiment == "nominal" || o.experiment == "retuned_nominal")
        .map(|o| {
            (
                o.result.controller.clone(),
                if o.experiment == "nominal" {
                    "original".to_owned()
                } else {
                    "retuned".to_owned()
                },
                o.metrics.iae_c_s,
            )
        })
        .collect::<Vec<_>>();
    charts::summary_chart(
        &figures.join("original-vs-retuned.svg"),
        "Original and Equal-Budget Retuned Controllers",
        &tuning_points,
        "Nominal comparison IAE (°C·s)",
    )?;
    let fidelity_points = outcomes
        .iter()
        .filter(|o| o.experiment == "higher_fidelity")
        .map(|o| {
            (
                o.result.controller.clone(),
                o.scenario.clone(),
                o.metrics.rmse_c,
            )
        })
        .collect::<Vec<_>>();
    charts::summary_chart(
        &figures.join("higher-fidelity-summary.svg"),
        "Sensor Lag and Delay Sensitivity",
        &fidelity_points,
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
    let interval = IntervalCertificate::thermal(
        nominal.sample_period_s,
        nominal.capacity_j_per_k(),
        nominal.heat_transfer_w_per_k,
    );
    let lqr_margin = interval.lqr_margin(lqr.gain());
    let jury = subdivided_lqi_certificate(
        nominal.sample_period_s,
        nominal.capacity_j_per_k(),
        nominal.heat_transfer_w_per_k,
        lqi_state,
        lqi_integral,
        128,
    );
    fs::write(
        summaries.join("formal-stability-certificate.txt"),
        format!(
            "uncertainty_domain=capacity:[0.7,1.3]*nominal, heat_transfer:[0.7,1.3]*nominal\na_interval=[{:.15},{:.15}]\nb_interval=[{:.15},{:.15}]\nlqr_unit_circle_margin={:.15}\nlqi_jury_margin_1={:.15}\nlqi_jury_margin_2={:.15}\nlqi_jury_margin_3={:.15}\nlqr_certified={}\nlqi_certified={}\npid_certified=false\npid_note=The fourth-order interval characteristic polynomial retains dependent coefficients; only numerical grid evidence is claimed.\n",
            interval.a_min,
            interval.a_max,
            interval.b_min,
            interval.b_max,
            lqr_margin,
            jury[0],
            jury[1],
            jury[2],
            lqr_margin > 0.0,
            jury.iter().all(|margin| *margin > 0.0)
        ),
    )?;
    let mut stability_maxima: BTreeMap<String, f64> = BTreeMap::new();
    let mut stability_reader =
        ::csv::Reader::from_path(summaries.join("linear-stability-grid.csv"))?;
    for record in stability_reader.deserialize::<StabilityRow>() {
        let row = record?;
        stability_maxima
            .entry(row.controller)
            .and_modify(|value| *value = value.max(row.spectral_radius))
            .or_insert(row.spectral_radius);
    }
    let stability_points = stability_maxima
        .into_iter()
        .map(|(name, radius)| (name, "max grid radius".to_owned(), radius))
        .collect::<Vec<_>>();
    charts::summary_chart(
        &figures.join("stability-domain-summary.svg"),
        "Worst Linear Pole Radius on ±30% Grid",
        &stability_points,
        "Maximum spectral radius",
    )?;
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
    let table_dir = root.join("paper/tables");
    fs::create_dir_all(&table_dir)?;
    let mut nominal_table = String::from(
        "\\begin{tabular}{llrrr}\\toprule\nDesign & Controller & IAE ($^\\circ$C s) & RMSE ($^\\circ$C) & Settling (s) \\\\\n\\midrule\n",
    );
    for o in outcomes
        .iter()
        .filter(|o| o.experiment == "nominal" || o.experiment == "retuned_nominal")
    {
        nominal_table.push_str(&format!(
            "{} & {} & {:.1} & {:.3} & {} \\\\\n",
            if o.experiment == "nominal" {
                "Original"
            } else {
                "Retuned"
            },
            o.result.controller,
            o.metrics.iae_c_s,
            o.metrics.rmse_c,
            o.metrics
                .settling_time_s
                .map_or_else(|| "--".to_owned(), |value| format!("{value:.0}"))
        ));
    }
    nominal_table.push_str("\\bottomrule\\end{tabular}\n");
    fs::write(table_dir.join("nominal-comparison.tex"), nominal_table)?;
    let mut fidelity_table = String::from(
        "\\begin{tabular}{llrrr}\\toprule\nDesign & Controller & IAE ($^\\circ$C s) & RMSE ($^\\circ$C) & Saturation (s) \\\\\n\\midrule\n",
    );
    for o in outcomes
        .iter()
        .filter(|o| o.experiment == "higher_fidelity")
    {
        fidelity_table.push_str(&format!(
            "{} & {} & {:.1} & {:.3} & {:.0} \\\\\n",
            if o.scenario == "original" {
                "Original"
            } else {
                "Retuned"
            },
            o.result.controller,
            o.metrics.iae_c_s,
            o.metrics.rmse_c,
            o.metrics.saturation_duration_s
        ));
    }
    fidelity_table.push_str("\\bottomrule\\end{tabular}\n");
    fs::write(table_dir.join("higher-fidelity.tex"), fidelity_table)?;
    fs::write(
        table_dir.join("tuned-parameters.tex"),
        format!(
            "\\begin{{tabular}}{{lrrr}}\\toprule\nController & Parameter 1 & Parameter 2 & Parameter 3 \\\\\n\\midrule\nPID & {:.4} & {:.4} & {:.4} \\\\\nLQR & {:.6e} & {:.6e} & -- \\\\\nLQI & {:.6e} & {:.6e} & {:.6e} \\\\\n\\bottomrule\\end{{tabular}}\n",
            tuned_design.pid.kp,
            tuned_design.pid.ki,
            tuned_design.pid.kd,
            tuned_design.lqr.state_weight,
            tuned_design.lqr.input_weight,
            tuned_design.lqi.state_weight,
            tuned_design.lqi.integral_weight,
            tuned_design.lqi.input_weight
        ),
    )?;
    fs::write(
        table_dir.join("stability.tex"),
        format!(
            "\\begin{{tabular}}{{lcc}}\\toprule\nController & Formal interval result & Grid radius range \\\\\n\\midrule\nPID & Unproven & 0.992397--0.995996 \\\\\nLQR & Certified (margin {:.6}) & 0.991930--0.996039 \\\\\nLQI & Certified (min. Jury margin {:.6}) & 0.986843--0.993128 \\\\\n\\bottomrule\\end{{tabular}}\n",
            lqr_margin,
            jury.iter().copied().fold(f64::INFINITY, f64::min)
        ),
    )?;
    Ok(())
}
