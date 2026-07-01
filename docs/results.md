# Results

## Extraction Time Consistency

| Controller | Mean (s) | Std (s) | Min (s) | Max (s) | N |
|---|---|---|---|---|---|
| Baseline PID | -- | -- | -- | -- | 20 |
| Adaptive (Kalman + gain-sched) | -- | -- | -- | -- | 20 |
| Adaptive + ML predictor | -- | -- | -- | -- | 20 |

**Target:** Adaptive+ML extraction time std at least 40% below baseline PID.

## Pressure Tracking

| Controller | RMSE vs 9-bar target (bar) | Max overshoot (bar) |
|---|---|---|
| Baseline PID | -- | -- |
| Adaptive | -- | -- |
| Adaptive + ML | -- | -- |

## System Identification

- **Plant model:** First-order plus dead time (FOPDT)
  - K = -- bar/duty
  - tau = -- s
  - theta = -- s (transport delay)

![Bode Plot](bode.png)

## Kalman Filter Validation

- State: [pressure, flow, puck_resistance]
- Process noise Q: diag([0.1, 0.05, 0.5])
- Measurement noise R: diag([0.2, 0.1])

![Kalman Residuals](kalman.png)

## Controller Comparison

![Side-by-side pressure traces](comparison.png)

## TinyML Model

- Architecture: Dense(600 -> 32 -> 16 -> 3)
- Size: -- KB (int8 quantized)
- Inference time: -- us on STM32G474 @ 170 MHz
- Validation MAE: -- bar*s/mL

## Reproducibility

All plots regenerate from logged data:
```bash
make results
```
