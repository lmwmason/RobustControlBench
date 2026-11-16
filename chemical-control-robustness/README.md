# Chemical Control Robustness

This Rust research package compares PID, discrete LQR, and discrete LQI control of a simulated, well-mixed solution vessel. Its central question is: **how does increasing model uncertainty influence their relative performance?** The work is a numerical study, not evidence from a real chemical process.

## Chemical and mathematical model

A homogeneous, non-reacting liquid has thermal capacity `C = m cp`. Conservation of energy gives

`m cp dT/dt = Q - UA(T - Ta) + d(t)`.

Here `T` and `Ta` are solution and ambient temperatures (°C or K differences), `Q` and `d` are controlled and external thermal power (W), `m` is mass (kg), `cp` is specific heat (J kg⁻¹ K⁻¹), and `UA` is heat loss per temperature difference (W K⁻¹). The illustrative assumptions are 10 kg, 4180 J kg⁻¹ K⁻¹, 35 W K⁻¹, and 20 °C ambient. They are not measurements.

With `x = T - Ta`, the model is `x_dot = A x + B u + E d`, `y = x`, where `A=-UA/C` and `B=E=1/C`. Exact zero-order-hold coefficients are `Ad=exp(A dt)` and `Bd=(Ad-1)B/A`. A unit test checks this against the analytical first-order solution.

## Controllers

- PID uses proportional and integral error, a filtered derivative on measurement, saturation, and back-calculation anti-windup.
- LQR solves the discrete algebraic Riccati equation and combines state feedback with nominal equilibrium feedforward. It does not pretend that a zero-reference regulator tracks by itself.
- LQI augments the plant with `z[k+1]=z[k]+dt(r-y)` and solves the two-state DARE. Its augmented controllability is tested.

All gains are designed once from the nominal model and frozen. All controllers observe the same measured temperature and share the same 1 s sample, 3600 s horizon, plant, limits, disturbances, and noise realizations. The bidirectional actuator represents a combined heater/chiller: nominal limits are -2000 W cooling and +3000 W heating.

## Reproduce

Install Rust 1.100, then run:

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
cargo run --release -- run-all
```

Configurations are in `configs/`. The last command deterministically regenerates `results/data`, `results/figures`, and `results/summaries`. Rust is the only project implementation language.

## Outputs and limitations

`results/data/metrics.csv` contains every scenario/controller metric. Nominal traces are separate CSVs, nine publication-ready SVGs are under `results/figures`, and noise statistics are under `results/summaries`. See the [audited manuscript](paper/manuscript.md), [scientific audit](docs/audit/scientific-audit.md), and [reproducibility record](docs/reproducibility.md). The earlier final report is retained as a pre-audit historical artifact; its saturation ranking is superseded.

The vessel is perfectly mixed, properties are constant, transport delay and actuator dynamics are absent, uncertainty is structured and bounded, and tuning is intentionally modest. A bounded finite simulation is not a proof of nonlinear or real-world stability.

