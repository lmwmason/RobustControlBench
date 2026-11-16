/// Positive stabilizing solution of the scalar discrete Riccati equation.
pub fn scalar_dare_closed_form(a: f64, b: f64, q: f64, r: f64) -> f64 {
    let linear = r * (1.0 - a * a) - q * b * b;
    (-linear + (linear * linear + 4.0 * b * b * q * r).sqrt()) / (2.0 * b * b)
}

pub fn scalar_lqr_gain(a: f64, b: f64, p: f64, r: f64) -> f64 {
    b * p * a / (r + b * b * p)
}

pub fn two_by_two_eigenvalues(a: f64, b: f64, c: f64, d: f64) -> (f64, f64, f64) {
    let trace = a + d;
    let determinant = a * d - b * c;
    let discriminant = trace * trace - 4.0 * determinant;
    if discriminant >= 0.0 {
        let root = discriminant.sqrt();
        ((trace + root) / 2.0, (trace - root) / 2.0, 0.0)
    } else {
        (trace / 2.0, trace / 2.0, (-discriminant).sqrt() / 2.0)
    }
}
