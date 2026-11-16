use chemical_control_robustness::{
    analysis::compute_metrics,
    controllers::PidController,
    model::ThermalParameters,
    simulation::{SimulationConfig, disturbances::Disturbance, engine::simulate},
};

#[test]
fn simulation_and_metrics_are_finite() {
    let p: ThermalParameters =
        chemical_control_robustness::read_toml("configs/nominal.toml").unwrap();
    let mut c = PidController::new(
        420.0,
        2.0,
        1200.0,
        15.0,
        0.02,
        p.sample_period_s,
        p.minimum_power_w,
        p.maximum_power_w,
    );
    let result = simulate(
        &p,
        &mut c,
        SimulationConfig {
            disturbance: Disturbance::None,
            measurement_noise_std_c: 0.15,
            seed: 42,
        },
    );
    assert!(
        result
            .records
            .iter()
            .all(|r| r.true_temperature_c.is_finite() && r.control_power_w.is_finite())
    );
    let m = compute_metrics(&result, p.sample_period_s);
    assert!(m.iae_c_s.is_finite() && m.rmse_c.is_finite());
}

#[test]
fn metric_quadrature_uses_exact_number_of_intervals() {
    use chemical_control_robustness::simulation::{SimulationRecord, SimulationResult};
    let records = (0..=10)
        .map(|step| SimulationRecord {
            time_s: step as f64,
            true_temperature_c: 0.0,
            measured_temperature_c: 0.0,
            reference_c: 2.0,
            tracking_error_c: 2.0,
            control_power_w: 3.0,
            disturbance_power_w: 0.0,
            saturated: false,
        })
        .collect();
    let result = SimulationResult {
        controller: "synthetic".to_owned(),
        records,
    };
    let metrics = compute_metrics(&result, 1.0);
    assert_eq!(metrics.iae_c_s, 20.0);
    assert_eq!(metrics.ise_c2_s, 40.0);
    assert_eq!(metrics.rmse_c, 2.0);
    assert_eq!(metrics.control_effort_w2_s, 90.0);
}
