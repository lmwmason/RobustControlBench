use chemical_control_robustness::{
    analysis::validation::{scalar_dare_closed_form, scalar_lqr_gain, two_by_two_eigenvalues},
    controllers::{Controller, LqiController, LqrController, PidController, lqr::solve_dare},
    model::ThermalParameters,
};
use nalgebra::DMatrix;

fn nominal() -> ThermalParameters {
    chemical_control_robustness::read_toml("configs/nominal.toml").unwrap()
}

#[test]
fn scalar_dare_agrees_with_independent_closed_form() {
    let p = nominal();
    let d = p.discretize();
    let (computed, gain, _) = solve_dare(
        &DMatrix::from_element(1, 1, d.state_factor),
        &DMatrix::from_element(1, 1, d.input_factor),
        &DMatrix::from_element(1, 1, 1.0),
        &DMatrix::from_element(1, 1, 0.00002),
    )
    .unwrap();
    let exact = scalar_dare_closed_form(d.state_factor, d.input_factor, 1.0, 0.00002);
    assert!((computed[(0, 0)] - exact).abs() / exact < 2.0e-12);
    let exact_gain = scalar_lqr_gain(d.state_factor, d.input_factor, exact, 0.00002);
    assert!((gain[(0, 0)] - exact_gain).abs() / exact_gain.abs() < 2.0e-12);
}

#[test]
fn lqi_matrix_eigenvalues_match_closed_form() {
    let p = nominal();
    let d = p.discretize();
    let lqi = LqiController::design(
        d.state_factor,
        d.input_factor,
        p.sample_period_s,
        1.0,
        0.0008,
        0.00002,
        p.heat_transfer_w_per_k,
        p.ambient_c,
        p.minimum_power_w,
        p.maximum_power_w,
    )
    .unwrap();
    let (kx, kz) = lqi.gains();
    let a = d.state_factor - d.input_factor * kx;
    let b = -d.input_factor * kz;
    let (root_one, root_two, imaginary) = two_by_two_eigenvalues(a, b, -p.sample_period_s, 1.0);
    let expected_radius = (root_one * root_one + imaginary * imaginary)
        .sqrt()
        .max((root_two * root_two + imaginary * imaginary).sqrt());
    let calculated = chemical_control_robustness::analysis::robustness::lqi_radius(
        d.state_factor,
        d.input_factor,
        p.sample_period_s,
        kx,
        kz,
    );
    assert!((expected_radius - calculated).abs() < 1.0e-12);
    assert!(calculated < 1.0);
}

#[test]
fn riccati_residual_is_small_and_closed_loop_is_stable() {
    let p = nominal();
    let d = p.discretize();
    let lqr = LqrController::design(
        d.state_factor,
        d.input_factor,
        1.0,
        0.00002,
        p.heat_transfer_w_per_k,
        p.ambient_c,
        p.minimum_power_w,
        p.maximum_power_w,
    )
    .unwrap();
    assert!(lqr.residual() < 1.0e-8);
    assert!((d.state_factor - d.input_factor * lqr.gain()).abs() < 1.0);
    let (_, _, residual) = solve_dare(
        &DMatrix::from_element(1, 1, d.state_factor),
        &DMatrix::from_element(1, 1, d.input_factor),
        &DMatrix::identity(1, 1),
        &DMatrix::from_element(1, 1, 0.00002),
    )
    .unwrap();
    assert!(residual < 1.0e-8);
}

#[test]
fn augmented_system_is_controllable_and_lqi_is_valid() {
    let p = nominal();
    let d = p.discretize();
    let controllability_determinant = -p.sample_period_s * d.input_factor * d.input_factor;
    assert!(controllability_determinant.abs() > 1.0e-12);
    let lqi = LqiController::design(
        d.state_factor,
        d.input_factor,
        p.sample_period_s,
        1.0,
        0.0008,
        0.00002,
        p.heat_transfer_w_per_k,
        p.ambient_c,
        p.minimum_power_w,
        p.maximum_power_w,
    )
    .unwrap();
    assert!(lqi.residual() < 1.0e-8);
    assert!(lqi.gains().1 < 0.0);
}

#[test]
fn controllers_respect_saturation_and_pid_recovers() {
    let mut pid = PidController::new(1000.0, 10.0, 0.0, 1.0, 0.1, 1.0, -100.0, 100.0);
    for _ in 0..100 {
        assert_eq!(pid.command(50.0, 0.0), 100.0);
    }
    assert!(pid.command(0.0, 50.0) < 0.0);
}
