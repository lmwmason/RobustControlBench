# Limitations

- All results are simulation outputs from assumed parameters, not laboratory measurements.
- Perfect mixing, constant properties, linear heat loss, instantaneous sensing, and instantaneous actuator response omit important plant effects.
- Gaussian independent noise is only one sensor model.
- The uncertainty grid is structured and limited to ±30%; three combined cases are illustrative rather than exhaustive.
- Controller tuning is reproducible but not a global optimization, so rankings can change with weights and objectives.
- LQR has nominal feedforward but no integral disturbance model; that design choice explains persistent errors under mismatch and is not a universal weakness of all LQR tracking architectures.
- A stable nominal pole and bounded 3600 s traces do not prove robustness for arbitrary uncertainty or nonlinear hardware.

