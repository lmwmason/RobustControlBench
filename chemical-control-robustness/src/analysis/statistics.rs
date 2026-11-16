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
