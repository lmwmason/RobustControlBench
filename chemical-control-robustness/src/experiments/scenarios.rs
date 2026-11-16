use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
pub struct ExperimentsConfig {
    pub uncertainty_percent: Vec<i32>,
    pub noise_standard_deviation_c: f64,
    pub noise_replicates: usize,
    pub base_seed: u64,
    pub disturbance_start_s: f64,
    pub disturbance_power_w: f64,
    pub combined: Vec<CombinedScenario>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct CombinedScenario {
    pub name: String,
    pub capacity_percent: i32,
    pub heat_transfer_percent: i32,
    pub ambient_offset_c: f64,
}
