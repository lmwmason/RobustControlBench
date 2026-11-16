use super::Controller;
use anyhow::{Result, bail};
use nalgebra::DMatrix;

pub fn solve_dare(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
) -> Result<(DMatrix<f64>, DMatrix<f64>, f64)> {
    let mut p = q.clone();
    for _ in 0..100_000 {
        let middle = r + b.transpose() * &p * b;
        let inverse = middle
            .try_inverse()
            .ok_or_else(|| anyhow::anyhow!("singular DARE input term"))?;
        let next =
            a.transpose() * &p * a - a.transpose() * &p * b * inverse * b.transpose() * &p * a + q;
        let change = (&next - &p).norm();
        p = next;
        if change < 1.0e-13 {
            break;
        }
    }
    let gain = (r + b.transpose() * &p * b)
        .try_inverse()
        .ok_or_else(|| anyhow::anyhow!("singular gain term"))?
        * b.transpose()
        * &p
        * a;
    let residual_matrix = a.transpose() * &p * a
        - &p
        - a.transpose()
            * &p
            * b
            * (r + b.transpose() * &p * b).try_inverse().unwrap()
            * b.transpose()
            * &p
            * a
        + q;
    let residual = residual_matrix.norm();
    if residual > 1.0e-8 {
        bail!("DARE residual {residual:e} exceeds tolerance");
    }
    Ok((p, gain, residual))
}

#[derive(Clone, Debug)]
pub struct LqrController {
    gain: f64,
    feedforward_w: f64,
    ambient_c: f64,
    minimum: f64,
    maximum: f64,
    residual: f64,
}

impl LqrController {
    #[allow(clippy::too_many_arguments)]
    pub fn design(
        a: f64,
        b: f64,
        state_weight: f64,
        input_weight: f64,
        heat_transfer_w_per_k: f64,
        ambient_c: f64,
        minimum: f64,
        maximum: f64,
    ) -> Result<Self> {
        let (_, gain, residual) = solve_dare(
            &DMatrix::from_element(1, 1, a),
            &DMatrix::from_element(1, 1, b),
            &DMatrix::from_element(1, 1, state_weight),
            &DMatrix::from_element(1, 1, input_weight),
        )?;
        Ok(Self {
            gain: gain[(0, 0)],
            feedforward_w: heat_transfer_w_per_k,
            ambient_c,
            minimum,
            maximum,
            residual,
        })
    }
    pub fn gain(&self) -> f64 {
        self.gain
    }
    pub fn residual(&self) -> f64 {
        self.residual
    }
}

impl Controller for LqrController {
    fn name(&self) -> &'static str {
        "LQR"
    }
    fn reset(&mut self) {}
    fn command(&mut self, reference_c: f64, measured_c: f64) -> f64 {
        (self.feedforward_w * (reference_c - self.ambient_c)
            + self.gain * (reference_c - measured_c))
            .clamp(self.minimum, self.maximum)
    }
}
