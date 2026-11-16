use super::discretization::scalar_zoh;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ThermalParameters {
    pub mass_kg: f64,
    pub specific_heat_j_per_kg_k: f64,
    pub heat_transfer_w_per_k: f64,
    pub ambient_c: f64,
    pub sample_period_s: f64,
    pub initial_c: f64,
    pub reference_c: f64,
    pub duration_s: f64,
    pub minimum_power_w: f64,
    pub maximum_power_w: f64,
}

impl ThermalParameters {
    pub fn capacity_j_per_k(&self) -> f64 {
        self.mass_kg * self.specific_heat_j_per_kg_k
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.mass_kg <= 0.0
            || self.specific_heat_j_per_kg_k <= 0.0
            || self.heat_transfer_w_per_k <= 0.0
            || self.sample_period_s <= 0.0
            || self.duration_s <= 0.0
            || self.minimum_power_w >= self.maximum_power_w
        {
            return Err("physical parameters and time must be positive and limits ordered");
        }
        Ok(())
    }

    pub fn discretize(&self) -> DiscretePlant {
        let capacity = self.capacity_j_per_k();
        let a = -self.heat_transfer_w_per_k / capacity;
        let b = 1.0 / capacity;
        let e = 1.0 / capacity;
        let (state_factor, input_factor, disturbance_factor) =
            scalar_zoh(a, b, e, self.sample_period_s);
        DiscretePlant {
            state_factor,
            input_factor,
            disturbance_factor,
            ambient_c: self.ambient_c,
            heat_transfer_w_per_k: self.heat_transfer_w_per_k,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DiscretePlant {
    pub state_factor: f64,
    pub input_factor: f64,
    pub disturbance_factor: f64,
    pub ambient_c: f64,
    pub heat_transfer_w_per_k: f64,
}

impl DiscretePlant {
    /// Exact update with constant actuator and disturbance power over one sample.
    pub fn update(&self, temperature_c: f64, power_w: f64, disturbance_w: f64) -> f64 {
        self.ambient_c
            + self.state_factor * (temperature_c - self.ambient_c)
            + self.input_factor * power_w
            + self.disturbance_factor * disturbance_w
    }

    pub fn equilibrium_power(&self, temperature_c: f64) -> f64 {
        self.heat_transfer_w_per_k * (temperature_c - self.ambient_c)
    }
}
