use chemical_control_robustness::model::{ThermalParameters, discretization::scalar_zoh};

fn nominal() -> ThermalParameters {
    chemical_control_robustness::read_toml("configs/nominal.toml").unwrap()
}

#[test]
fn dimensions_and_parameters_are_physical() {
    assert!(nominal().validate().is_ok());
    assert!((nominal().capacity_j_per_k() - 41_800.0).abs() < 1.0e-10);
}

#[test]
fn equilibrium_is_preserved() {
    let p = nominal();
    let d = p.discretize();
    let t = 50.0;
    let next = d.update(t, d.equilibrium_power(t), 0.0);
    assert!((next - t).abs() < 1.0e-11);
}

#[test]
fn zoh_matches_analytical_solution() {
    let p = nominal();
    let d = p.discretize();
    let t0 = 31.0;
    let u = 900.0;
    let equilibrium = p.ambient_c + u / p.heat_transfer_w_per_k;
    let expected = equilibrium
        + (t0 - equilibrium)
            * (-p.heat_transfer_w_per_k / p.capacity_j_per_k() * p.sample_period_s).exp();
    assert!((d.update(t0, u, 0.0) - expected).abs() < 1.0e-12);
    let (a, b, e) = scalar_zoh(0.0, 2.0, 3.0, 0.5);
    assert_eq!((a, b, e), (1.0, 1.0, 1.5));
}

#[test]
fn multi_step_zoh_matches_analytical_solution_for_multiple_sample_periods() {
    for dt in [0.1, 1.0, 7.5, 60.0] {
        let mut p = nominal();
        p.sample_period_s = dt;
        let plant = p.discretize();
        let (power, disturbance, initial) = (725.0, -125.0, 27.0);
        let equilibrium = p.ambient_c + (power + disturbance) / p.heat_transfer_w_per_k;
        let mut numerical = initial;
        for step in 1..=100 {
            numerical = plant.update(numerical, power, disturbance);
            let time = step as f64 * dt;
            let analytical = equilibrium
                + (initial - equilibrium)
                    * (-p.heat_transfer_w_per_k * time / p.capacity_j_per_k()).exp();
            assert!(
                (numerical - analytical).abs() < 2.0e-11,
                "dt={dt}, step={step}"
            );
        }
    }
}
