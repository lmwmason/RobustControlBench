# Research Plan

The revised study freezes nominal PID, LQR, and LQI gains, then evaluates them on identical actual plants. Experiments cover nominal tracking; isolated thermal-capacity and heat-transfer deviations at -30%, -20%, -10%, 0%, +10%, +20%, and +30%; a -700 W step load; 100 paired noise seeds; restricted actuator limits; and three combined cases. Primary comparisons use IAE and RMSE, while overshoot, settling, steady error, effort, and saturation expose tradeoffs. The machine-readable training, validation, and test separation is in `configs/experiment-manifest.toml`.

Acceptance checks cover physical parameter validity, equilibrium, analytical propagation, DARE residuals, closed-loop poles, augmented controllability, saturation, anti-windup recovery, reproducibility, finite values, and CSV row counts.

