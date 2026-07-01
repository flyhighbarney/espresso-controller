# Simulink Plant Model

## Plant Model (`plant.slx`)

Build this in Simulink with the following blocks:

### Pump
- Input: PWM duty (0–1)
- Model: `K_pump * duty`, first-order lag `1/(tau_pump*s + 1)`, saturation [0, 15 bar]
- Parameters: `K_pump = 12 bar`, `tau_pump = 0.3 s`

### Transport Line / Group Head
- Pure delay block: configurable 0.5–2.0 s
- Represents hydraulic transport from pump to puck

### Puck (Nonlinear Resistance)
- `R(t) = R0 * (1 + alpha * integral(flow) dt) * (1 + beta * noise)`
- R0 = 8–12 bar*s/mL (depends on grind)
- alpha = 0.01–0.05 (compaction rate)
- beta = 0.02 (channeling noise)
- Output: `pressure_drop = flow * R(t)`

### Sensors
- Pressure: additive Gaussian noise (sigma = 0.15 bar) + 12-bit quantization
- Flow: pulse-counting quantization + Gaussian noise (sigma = 0.05 mL/s)
- Temperature: NTC nonlinearity + noise (sigma = 0.3 C)

## Controllers (`controllers.slx`)

Implement each variant as a subsystem with the same I/O interface:
- Input: [setpoint, pressure_meas, flow_meas]
- Output: [pump_duty, est_resistance, phase]

Variants:
1. PID baseline (tunable gains from workspace)
2. Kalman + adaptive PID (MATLAB Function block)
3. Full adaptive + ML (MATLAB Function block calling predict())

## Code Generation

Use Simulink Coder to generate C from the controller subsystem:
```matlab
slbuild('controllers/adaptive_pid')
```

Output goes to `generated/` and integrates via `USE_SIMULINK_CONTROLLER` build flag.

The generated code implements the same `controller_interface.h` API as the hand-written version.
