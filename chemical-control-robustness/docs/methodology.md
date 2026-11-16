# Methodology

Controller gains in `configs/controllers.toml` were selected on the nominal model using a predefined practical objective: low IAE over 3600 s, settling within 2% of the 30 °C step, limited overshoot, and respect for -2000/+3000 W limits. Candidate PID values were considered on logarithmic engineering ranges `Kp=100..1000 W/K`, `Ki=0.2..10 W/(K s)`, `Kd=0..3000 W s/K`; LQR/LQI weights used `Qx=1`, `R=2e-5`, and LQI `Qz=8e-4`. The documented chosen values are frozen before uncertainty testing.

At every 1 s step, seeded Gaussian noise is added only to measurement, the controller computes power, limits are applied, and the actual scalar plant advances by exact ZOH. Twelve noise replicates use shared seeds for paired descriptive comparisons.

For error `e=r-y`, metrics are: `IAE=∫|e|dt`, `ISE=∫e²dt`, `RMSE=sqrt(ISE/H)`, maximum `|e|`, overshoot `max(y-r,0)`, first time after which `|e|` stays within 2% of the step (minimum 0.1 °C), tail-average steady error, `∫u²dt`, and time at a limit. Missing settling is stored as an empty CSV field.

The scalar LQR closed-loop pole is checked inside the unit circle. The two-state LQI controllability determinant is nonzero. DARE residual norms must be below `1e-8`. These are linear nominal checks; uncertain cases receive finite-horizon behavioral evaluation only.

