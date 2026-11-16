pub fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

pub fn sample_standard_deviation(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let average = mean(values);
    (values.iter().map(|v| (v - average).powi(2)).sum::<f64>() / (values.len() - 1) as f64).sqrt()
}

#[derive(Clone, Copy, Debug)]
pub struct PairedSummary {
    pub mean_difference: f64,
    pub standard_deviation: f64,
    pub ci95_low: f64,
    pub ci95_high: f64,
    pub standardized_effect: f64,
}

pub fn paired_summary(left: &[f64], right: &[f64]) -> PairedSummary {
    assert_eq!(left.len(), right.len());
    let differences: Vec<f64> = left.iter().zip(right).map(|(a, b)| a - b).collect();
    let average = mean(&differences);
    let standard_deviation = sample_standard_deviation(&differences);
    // 1.984 is the two-sided 95% t critical value for 99 degrees of freedom.
    let half_width = 1.984 * standard_deviation / (differences.len() as f64).sqrt();
    PairedSummary {
        mean_difference: average,
        standard_deviation,
        ci95_low: average - half_width,
        ci95_high: average + half_width,
        standardized_effect: if standard_deviation > 0.0 {
            average / standard_deviation
        } else {
            0.0
        },
    }
}
