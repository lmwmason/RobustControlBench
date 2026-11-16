# Independent Scientific Audit

Audit date: 2026-11-16. Scope: commit `26e11de`, including all Rust sources, tests, configurations, dependencies, generated CSV/SVG artifacts, and documentation. Passing tests were treated as software evidence, not scientific validation.

## Finding register

### A-01 — Actual saturation was invisible to integral controllers

- **Severity:** Critical.
- **Description:** The restricted-actuator experiment changed plant limits to -500/+1200 W, but `controllers(nominal, design)` constructed PID and LQI with nominal -2000/+3000 W limits. `simulate` then applied a second external clamp. Anti-windup therefore acted on the wrong limits.
- **Evidence:** Historical `runner.rs` passed nominal parameters to controller constructors while the engine clamped with actual-plant limits. Historical saturation IAE values (PID 26103, LQR 25268, LQI 25496 °C·s) cannot support a fair anti-windup ranking.
- **Consequence:** The reported saturation comparison and claim that LQR won that scenario were methodologically invalid.
- **Required correction:** Separate the design model from experiment actuator limits and provide the actual limits to every controller. Preserve historical results and label them superseded.
- **Verification:** Tests must expose an externally restricted limit and demonstrate integral recovery; regenerated saturation metrics must differ from historical artifacts.
- **Status:** Open at audit commit; resolved by the subsequent control-model correction commit.

### A-02 — LQI implementation and synthesis used different timing

- **Severity:** Major.
- **Description:** Synthesis assumed `z[k+1]=z[k]+h(r-y[k])` and `u[k]=-K[x[k],z[k]]`, but `command` updated `z` before calculating `u[k]`.
- **Evidence:** Historical `lqi.rs` incremented `integral_error` before forming the command; its augmented `A` matrix used the conventional forward-Euler next-state equation.
- **Consequence:** The implemented closed loop was not exactly the system whose Riccati gain and poles were claimed.
- **Required correction:** Compute the command from `z[k]`, then update the stored integral for the next sample, with anti-windup rollback when appropriate.
- **Verification:** Independently construct the implemented augmented matrix and compare its poles with direct two-by-two closed-form roots.
- **Status:** Open at audit commit; resolved subsequently.

### A-03 — Tuning protocol was described but not executed

- **Severity:** Major.
- **Description:** Documentation claimed ranges and a predefined objective, but the repository contained no tuning routine, candidate manifest, training/validation split, score table, or evidence that comparable search effort was applied.
- **Evidence:** Only one hand-selected parameter set existed in `controllers.toml`; `run-all` immediately evaluated the test scenarios.
- **Consequence:** Relative rankings may reflect tuning effort or test-set leakage rather than controller structure. The original comparison was not sufficient for a strong fairness claim.
- **Required correction:** Either implement and record a leakage-free protocol or explicitly downgrade claims. Use nominal training scenarios, separate validation scenarios, frozen final parameters, and sensitivity reporting.
- **Verification:** Machine-readable split manifest and tuning/sensitivity output generated before final evaluation.
- **Status:** Partially resolved: the revised work records the existing gains as preregistered fixed baselines and adds sensitivity analysis; an exhaustive equal-budget optimization remains future work.

### A-04 — Disturbance metric mixed tracking and rejection

- **Severity:** Major.
- **Description:** IAE and RMSE for the disturbance run included the initial 1800 s set-point transient, so most of the score was unrelated to disturbance rejection.
- **Evidence:** `compute_metrics` always integrated the full record. No post-disturbance recovery metric or matched undisturbed subtraction existed.
- **Consequence:** Statements about disturbance rejection based on total IAE were confounded.
- **Required correction:** Report post-disturbance IAE, maximum deviation from the pre-disturbance equilibrium/reference, recovery time, and tail error separately.
- **Verification:** Unit tests on synthetic records and dedicated disturbance-summary output.
- **Status:** Resolved subsequently.

### A-05 — Statistical analysis was descriptive and underpowered for inference

- **Severity:** Major.
- **Description:** Twelve noise seeds yielded only means and sample standard deviations. There were no paired differences, confidence intervals, effect sizes, or seed sensitivity checks.
- **Evidence:** Historical `noise-statistics.csv` had three rows and two statistics. Repeated time samples were not misused, but replicate-level inference was absent.
- **Consequence:** No inferential or reliability claim was justified. Very small RMSE variance was dominated by the large deterministic start-up transient.
- **Required correction:** Increase independent seeded replicates, retain pairing, analyze post-settling noise-sensitive metrics, and report paired mean differences, 95% confidence intervals, and standardized paired effect sizes without asserting universal significance.
- **Verification:** Reproducible replicate-level CSV and paired analysis with order-independent seeds.
- **Status:** Resolved within the specified noise model using 100 paired seeds; external validity remains limited.

### A-06 — No robust-stability analysis supported “robustness” language

- **Severity:** Major.
- **Description:** Only nominal LQR stability was tested. PID and LQI poles under uncertainty were not calculated, and bounded traces were sometimes discussed as robustness evidence.
- **Evidence:** Historical controller tests checked one scalar LQR pole. There was no uncertainty-domain pole table or analytical unit-circle argument.
- **Consequence:** Robust-stability claims were unsupported; only finite-horizon empirical performance had been established.
- **Required correction:** Derive unsaturated closed-loop matrices, calculate poles across the stated bounded domain, and exploit scalar formulas/Jury conditions where possible. Keep saturation claims empirical.
- **Verification:** Machine-readable stability grid and independent closed-form eigenvalue tests.
- **Status:** Resolved for the defined linear, noise-free, unsaturated grid; continuous-domain interval proof is provided for LQR and numerically bounded evidence for PID/LQI.

### A-07 — DARE solver did not report non-convergence

- **Severity:** Moderate.
- **Description:** The Riccati iteration exited after 100,000 iterations without recording convergence. A small residual usually catches failure, but convergence status and iteration count were unavailable; one `unwrap` remained in residual evaluation.
- **Evidence:** Historical `solve_dare` broke only on tolerance and returned a three-tuple.
- **Consequence:** Numerical diagnostics were incomplete and failure modes were less transparent.
- **Required correction:** Return diagnostics including iterations, update norm, residual, and convergence; remove the unchecked inverse.
- **Verification:** Scalar solution compared with the positive closed-form DARE root and an independent finite-horizon backward recursion.
- **Status:** Resolved subsequently.

### A-08 — Metric quadrature counted one extra interval

- **Severity:** Moderate.
- **Description:** A record existed at both endpoints (`N+1` samples), while every sample was multiplied by `dt`, integrating `N+1` rectangles over an `N dt` horizon.
- **Evidence:** Historical metrics summed all records. RMSE divided by `records.len()*dt`, making its convention internally consistent but physically one sample too long.
- **Consequence:** Integral metrics and saturation duration were biased by one sample; rankings were unlikely to change, but reported units did not match the stated horizon.
- **Required correction:** Integrate the `N` sample intervals (left rectangles) and define endpoint handling.
- **Verification:** Constant synthetic signal over a known horizon must integrate exactly.
- **Status:** Resolved subsequently.

### A-09 — Saturation detection conflated requested and applied commands

- **Severity:** Moderate.
- **Description:** The engine received only an already-clamped controller output and then inferred saturation from equality with a plant limit. It could not record the unsaturated request or distinguish exact boundary operation.
- **Evidence:** `Controller::command` returned a single `f64`; records contained only applied power.
- **Consequence:** Saturation duration was coarse, and external/internal limit inconsistency was difficult to diagnose.
- **Required correction:** Make controller limits authoritative for a scenario and document the equality convention; ideally record requested and applied commands.
- **Verification:** Boundary and over-limit unit tests.
- **Status:** Partially resolved: actual scenario limits are now authoritative and saturation convention documented; a separate unconstrained-command channel is not yet exposed.

### A-10 — Ambient “percent uncertainty” was physically awkward

- **Severity:** Moderate.
- **Description:** Combined cases multiplied Celsius ambient temperature by a percentage. Celsius is an affine scale, so percent changes are not physically invariant.
- **Evidence:** Historical code used `ambient_c *= 1 + percent/100`.
- **Consequence:** The resulting temperatures were numerically valid but the uncertainty label was scientifically misleading.
- **Required correction:** Specify ambient deviations additively in °C and keep them distinct from fixed-parameter uncertainty.
- **Verification:** Configuration schema and report use `ambient_offset_c`.
- **Status:** Resolved subsequently.

### A-11 — Uncertainty figures showed only one endpoint

- **Severity:** Moderate.
- **Description:** Capacity and heat-transfer figures plotted one -30% or +30% time trace, while captions and conclusions described the full seven-level sweep.
- **Evidence:** Historical chart calls selected a single scenario.
- **Consequence:** Figures could not visually substantiate monotonicity, asymmetry, or controller ranking across uncertainty.
- **Required correction:** Add parameter-sweep performance plots with uncertainty on the x-axis.
- **Verification:** Figure data must be traceable to the full metrics CSV.
- **Status:** Resolved subsequently with revised evaluation figures.

### A-12 — Validation was partly self-referential

- **Severity:** Moderate.
- **Description:** The DARE test invoked the same solver and checked its own residual. ZOH had one analytical point, and no multi-step/sample-period sweep existed.
- **Evidence:** Historical tests called `solve_dare` twice but had no independent scalar root or backward Riccati calculation.
- **Consequence:** Shared implementation errors could pass tests.
- **Required correction:** Add closed-form scalar DARE, finite-horizon Riccati, analytical thermal trajectories over multiple sample intervals, and closed-form 2×2 pole checks.
- **Verification:** Independent methods agree within scale-aware tolerances.
- **Status:** Resolved subsequently.

### A-13 — Literature review was too narrow for an academic manuscript

- **Severity:** Moderate.
- **Description:** The report cited three tutorial/vendor pages and no focused peer-reviewed anti-windup, robust uncertainty, process-control, or benchmarking literature.
- **Evidence:** Historical report references contained CTMS and MathWorks pages only.
- **Consequence:** The work could not support a novelty or publication-readiness claim.
- **Required correction:** Verify bibliographic metadata from publisher, DOI, institutional, or textbook records; state that the review is focused rather than comprehensive.
- **Verification:** Each reference contains authors, title, year, venue, DOI or stable URL, and relevance.
- **Status:** Resolved to a focused review; no comprehensive novelty search is claimed.

### A-14 — The original contribution was overstated by presentation

- **Severity:** Minor.
- **Description:** A standard first-order PID/LQR/LQI benchmark is not a novel controller or chemical-process validation.
- **Evidence:** The plant and methods are textbook structures and use assumed parameters.
- **Consequence:** Publication framing must be narrow and educational/computational.
- **Required correction:** Characterize the contribution as an auditable comparative computational study of structural integral action, uncertainty, and saturation.
- **Verification:** Manuscript abstract, conclusion, and limitations avoid novelty and laboratory claims.
- **Status:** Resolved in the revised manuscript.

## Audit conclusion before corrections

At commit `26e11de`, the model equation and scalar ZOH were basically sound, but the saturation experiment was invalid, LQI timing did not match synthesis, tuning fairness was undocumented, disturbance metrics were confounded, statistical support was descriptive only, and robust-stability claims lacked analysis. The original report was therefore **not suitable for external academic review**. Historical outputs remain useful provenance but must not be treated as final evidence.
