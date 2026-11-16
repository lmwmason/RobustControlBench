#[derive(Clone, Copy, Debug)]
pub enum Disturbance {
    None,
    Step { start_s: f64, power_w: f64 },
}

impl Disturbance {
    pub fn value(self, time_s: f64) -> f64 {
        match self {
            Self::None => 0.0,
            Self::Step { start_s, power_w } if time_s >= start_s => power_w,
            Self::Step { .. } => 0.0,
        }
    }
}
