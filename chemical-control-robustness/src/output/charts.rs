use crate::simulation::SimulationResult;
use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::Path;

fn color(name: &str) -> RGBColor {
    match name {
        "PID" => RGBColor(0, 114, 178),
        "LQR" => RGBColor(213, 94, 0),
        _ => RGBColor(0, 158, 115),
    }
}

pub fn trace_chart(
    path: &Path,
    title: &str,
    y_label: &str,
    results: &[SimulationResult],
    power: bool,
) -> Result<()> {
    let root = SVGBackend::new(path, (1100, 650)).into_drawing_area();
    root.fill(&WHITE)?;
    let duration = results[0].records.last().unwrap().time_s;
    let values = results.iter().flat_map(|r| {
        r.records.iter().map(|x| {
            if power {
                x.control_power_w
            } else {
                x.true_temperature_c
            }
        })
    });
    let (mut low, mut high) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
        (lo.min(v), hi.max(v))
    });
    let margin = ((high - low) * 0.08).max(1.0);
    low -= margin;
    high += margin;
    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(45)
        .y_label_area_size(65)
        .build_cartesian_2d(0.0..duration, low..high)?;
    chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc(y_label)
        .label_style(("sans-serif", 16))
        .axis_desc_style(("sans-serif", 18))
        .draw()?;
    for result in results {
        let series_color = color(&result.controller);
        chart
            .draw_series(LineSeries::new(
                result.records.iter().map(|x| {
                    (
                        x.time_s,
                        if power {
                            x.control_power_w
                        } else {
                            x.true_temperature_c
                        },
                    )
                }),
                series_color.stroke_width(2),
            ))?
            .label(result.controller.clone())
            .legend(move |(x, y)| {
                PathElement::new([(x, y), (x + 25, y)], series_color.stroke_width(3))
            });
    }
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .label_font(("sans-serif", 17))
        .draw()?;
    root.present()
        .with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

pub fn summary_chart(
    path: &Path,
    title: &str,
    points: &[(String, String, f64)],
    x_label: &str,
) -> Result<()> {
    let root = SVGBackend::new(path, (1100, 650)).into_drawing_area();
    root.fill(&WHITE)?;
    let max_x = points.iter().map(|p| p.2).fold(0.0, f64::max) * 1.1;
    let labels: Vec<String> = points.iter().map(|p| format!("{} {}", p.0, p.1)).collect();
    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(60)
        .y_label_area_size(180)
        .build_cartesian_2d(0.0..max_x, 0..points.len())?;
    chart
        .configure_mesh()
        .x_desc(x_label)
        .disable_y_mesh()
        .y_labels(points.len())
        .y_label_formatter(&|v| labels.get(*v).cloned().unwrap_or_default())
        .label_style(("sans-serif", 15))
        .axis_desc_style(("sans-serif", 18))
        .draw()?;
    chart.draw_series(points.iter().enumerate().map(|(index, point)| {
        Rectangle::new(
            [(0.0, index), (point.2, index + 1)],
            color(&point.0).filled(),
        )
    }))?;
    root.present()?;
    Ok(())
}
