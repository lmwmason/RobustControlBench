use chemical_control_robustness::{
    controllers::PidController,
    model::ThermalParameters,
    simulation::{SimulationConfig, disturbances::Disturbance, engine::simulate},
};

#[test]
fn seeded_noise_is_reproducible() {
    let p: ThermalParameters =
        chemical_control_robustness::read_toml("configs/nominal.toml").unwrap();
    let config = SimulationConfig {
        disturbance: Disturbance::None,
        measurement_noise_std_c: 0.2,
        seed: 1234,
    };
    let make = || {
        PidController::new(
            420.0,
            2.0,
            1200.0,
            15.0,
            0.02,
            p.sample_period_s,
            p.minimum_power_w,
            p.maximum_power_w,
        )
    };
    let (mut a, mut b) = (make(), make());
    let ra = simulate(&p, &mut a, config);
    let rb = simulate(&p, &mut b, config);
    assert_eq!(ra.records.len(), rb.records.len());
    for (x, y) in ra.records.iter().zip(&rb.records) {
        assert_eq!(
            x.measured_temperature_c.to_bits(),
            y.measured_temperature_c.to_bits()
        );
    }
}

#[test]
fn csv_rows_are_consistent() {
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
            measurement_noise_std_c: 0.0,
            seed: 1,
        },
    );
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("trace.csv");
    chemical_control_robustness::output::csv::write_trace(&path, &result).unwrap();
    let rows = csv::Reader::from_path(path).unwrap().records().count();
    assert_eq!(rows, result.records.len());
}

#[test]
fn delayed_sensor_simulation_is_reproducible() {
    use chemical_control_robustness::simulation::higher_fidelity::simulate_sensor_dynamics;
    let p: ThermalParameters =
        chemical_control_robustness::read_toml("configs/nominal.toml").unwrap();
    let make = || {
        PidController::new(
            420.0,
            2.0,
            1200.0,
            15.0,
            0.02,
            p.sample_period_s,
            p.minimum_power_w,
            p.maximum_power_w,
        )
    };
    let config = SimulationConfig {
        disturbance: Disturbance::None,
        measurement_noise_std_c: 0.1,
        seed: 99,
    };
    let (mut left, mut right) = (make(), make());
    let a = simulate_sensor_dynamics(&p, &mut left, config, 20.0, 10.0);
    let b = simulate_sensor_dynamics(&p, &mut right, config, 20.0, 10.0);
    assert!(
        a.records
            .iter()
            .zip(&b.records)
            .all(
                |(x, y)| x.true_temperature_c.to_bits() == y.true_temperature_c.to_bits()
                    && x.measured_temperature_c.to_bits() == y.measured_temperature_c.to_bits()
            )
    );
}
