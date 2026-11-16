/// Exact zero-order-hold discretization of x_dot = a*x + b*u + e*d.
pub fn scalar_zoh(a: f64, b: f64, e: f64, dt: f64) -> (f64, f64, f64) {
    assert!(dt > 0.0);
    if a.abs() < 1.0e-12 {
        (1.0, b * dt, e * dt)
    } else {
        let ad = (a * dt).exp();
        let factor = (ad - 1.0) / a;
        (ad, factor * b, factor * e)
    }
}
