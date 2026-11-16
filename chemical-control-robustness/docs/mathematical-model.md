# Mathematical Model

## Assumptions and units

The liquid is homogeneous and perfectly mixed; no reaction occurs; `m`, `cp`, and `UA` are constant within a run; the ambient is uniform; and applied powers are constant during each sample. The baseline `m=10 kg`, `cp=4180 J/(kg K)`, `UA=35 W/K`, and `Ta=20 °C` are plausible illustrative assumptions, not measurements.

Energy conservation gives `C dT/dt = Q-UA(T-Ta)+d`, with `C=m cp=41800 J/K`. Let `x=T-Ta`, `u=Q`, and output `y=x`. Then `A=-UA/C`, `B=1/C`, disturbance matrix `E=1/C`, and output matrix `Cy=1`. Distinct code names avoid confusing capacity `C` with `Cy`.

For sample time `h`, exact ZOH gives `Ad=exp(Ah)`, `Bd=(Ad-1)B/A`, and `Ed=(Ad-1)E/A`. The implementation also handles the `A→0` limit. At equilibrium, `Q=UA(T-Ta)`; an automated test confirms the discrete update preserves it.

Ambient changes are modeled separately from fixed-parameter uncertainty. Capacity and heat transfer alter physical coefficients, while ambient changes the forcing/equilibrium.

