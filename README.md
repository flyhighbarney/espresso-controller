# Adaptive espresso extraction controller

An STM32-based espresso rig that regulates pressure and flow during extraction. A Kalman filter estimates the coffee puck's hydraulic resistance in real time, an adaptive PID compensates for the transport delay between pump and group head, and a small neural network predicts the puck's resistance curve from the first 3 seconds of each shot.

Shot-to-shot extraction time standard deviation drops by >40% compared to a tuned baseline PID, same grind, dose, and tamp.

**[Try the live simulator](https://flyhighbarney.github.io/espresso-controller/)** (runs entirely in your browser, no hardware needed)

## How it works

Three FreeRTOS tasks run on an STM32G474RE (Cortex-M4F, 170 MHz):

- Control at 1 kHz: reads pressure and flow via ADC DMA, runs the Kalman filter, computes controller output, writes pump PWM
- Telemetry at 100 Hz: streams COBS-framed binary packets over USB CDC to the host dashboard
- Supervisor at 10 Hz: manages the state machine (idle, preheat, pre-infusion, extraction, flush, fault) and enforces safety interlocks

The host side has a real-time PyQt dashboard, system identification scripts, an ML training pipeline, and the browser simulator linked above.

## Control strategy

Each layer is testable against the one before it:

1. Open-loop characterization: chirp excitation, FOPDT model fit (gain K, time constant tau, dead time theta), Bode plot
2. Baseline PID: Ziegler-Nichols initial tune, hand-adjusted. Works, but lags on pucks with different grind or age because the transport delay shifts
3. Kalman filter: 3-state EKF tracking pressure, flow, and puck hydraulic resistance. Resistance isn't directly measured; the filter infers it from pressure = flow * resistance
4. Adaptive controller: gain-scheduled PID keyed on shot phase (pre-infusion, ramp, extraction, decline) and on the Kalman-estimated resistance. A Smith predictor uses a circular delay buffer to compensate the pump-to-group-head transport delay
5. TinyML predictor: a small fully-connected network (300 samples x 2 channels, two hidden layers of 32 and 16 ReLU units, outputs R_initial, compaction rate alpha, R_final). Under 5 KB quantized via CMSIS-NN or TFLite Micro. Feeds the first 3 seconds of each shot to pre-tune the adaptive controller before the Kalman estimate converges

## Firmware

Written in C on FreeRTOS. The control stack includes PID with anti-windup, a 3-state extended Kalman filter with tunable covariances, gain-scheduled adaptive control with per-phase gain tables, and Smith predictor delay compensation via a 64-sample circular buffer.

The ML inference context collects 3 seconds of downsampled sensor data at 100 Hz and runs a quantized neural network on-chip.

Sensor drivers cover a 0-200 psi pressure transducer (0.5-4.5V), NTC 10k thermistor (Steinhart-Hart), and Hall-effect flow meter (input capture ISR). Actuators are a vibratory pump PWM, cartridge heater PWM with a safety duty cap, and a 3-way solenoid valve.

The supervisor state machine handles fault detection: overpressure (>12 bar), overtemperature (>110C), dry-fire protection, sensor plausibility checks, and a watchdog timer. Telemetry is COBS-framed over USB CDC with packed binary state packets.

There's also a build flag to swap in a Simulink Coder-generated controller.

## Host tools

`host/dashboard/dashboard.py` is a PyQt5 + pyqtgraph GUI with live pressure/flow/resistance plots, PID gain tuning sliders, controller mode selection, and shot logging to CSV. Falls back to console mode without PyQt. Has a simulation mode for testing without hardware.

`host/sysid/system_identification.py` generates logarithmic chirp excitation signals, fits FOPDT models via least-squares, computes Bode plots, and renders step response overlays.

`host/training/train_resistance_model.py` trains the puck resistance predictor on real or synthetic data, exports to TFLite with int8 quantization, and writes a C header with the model weights for direct firmware embedding.

`host/sysid/benchmark.py` runs automated comparison across controller modes.

## Browser simulator

The [live simulator](https://flyhighbarney.github.io/espresso-controller/) reimplements the full control stack in JavaScript: plant model with configurable resistance, transport delay, and sensor noise; PID, Kalman filter, adaptive controller, and ML predictor; canvas-based live plots; slider controls for all parameters; and a 20-shot automated comparison that runs all three controller modes and reports extraction time statistics.

You can also run it locally by opening `host/simulator/index.html`.

## Hardware

| Component | Purpose |
|---|---|
| STM32 Nucleo-G474RE | Cortex-M4F, 170 MHz, hardware FPU, DSP instructions |
| Ulka EP5 vibratory pump | Drives water through the puck, controlled via PWM |
| 0-200 psi pressure transducer | Line pressure measurement (0.5-4.5V analog) |
| YF-S401 Hall-effect flow meter | Pulse output, counted via timer input capture |
| 100W cartridge heater + aluminum block | Temperature control (simulates a boiler) |
| NTC 10k thermistor | Temperature feedback for heater PID |
| 3-way solenoid valve | Pre-infusion and depressurization |
| 58mm bottomless portafilter | Holds the coffee puck |

Total BOM is around $120.

## Safety

- Pump cuts and solenoid opens if pressure exceeds 12 bar
- Heater cuts if temperature exceeds 110C
- Heater won't fire without recent flow (dry-fire protection)
- Watchdog timer resets the MCU on missed control deadlines
- Sensor plausibility checks reject readings outside physical bounds
- All fluid-path materials are food-safe (PTFE, 304 stainless)

## Quick start

### Browser simulator

Open the [live demo](https://flyhighbarney.github.io/espresso-controller/), click "Pull Shot," and adjust the sliders.

### Dashboard in simulation mode

```bash
pip install -r host/requirements.txt
make dashboard
```

### Build firmware

```bash
# requires arm-none-eabi-gcc
make firmware
```

### Unit tests

```bash
make tests
```

### Train and export the ML model

```bash
make train-export
```

Trains on synthetic data, quantizes to int8, and writes both `model.tflite` and a C header (`firmware/App/ml/model_data.h`) with the weights baked in.

## Repository layout

```
espresso-controller/
  firmware/              STM32 firmware (C, FreeRTOS)
    Core/Src/main.c      Entry point, FreeRTOS task setup
    App/control/         PID, Kalman filter, adaptive controller, Smith predictor
    App/sensors/         ADC pressure, NTC thermistor, flow meter ISR
    App/actuators/       Pump PWM, heater PWM, solenoid
    App/ml/              TinyML inference context
    App/telemetry/       COBS framing, USB CDC streaming
    App/supervisor/      State machine, fault detection, safety interlocks
    Tests/               Host-compiled unit tests (PID, Kalman, COBS)
  simulink/              Simulink plant model + Coder output
  host/
    dashboard/           Real-time PyQt dashboard
    sysid/               System identification and benchmarking
    training/            TinyML training pipeline (TensorFlow -> TFLite -> C header)
    simulator/           Browser-based interactive simulator (vanilla JS)
  data/                  Logged shots (CSV)
  docs/                  Results, plots, writeup
```

## Build flags

| Flag | Effect |
|---|---|
| `USE_SIMULINK_CONTROLLER` | Swaps in a Simulink Coder-generated controller |
| `UNIT_TEST` | Compiles for host-side testing without the HAL |

## Results

See [docs/results.md](docs/results.md) for extraction time tables, pressure tracking RMSE, Kalman residuals, and controller comparison plots.

All plots regenerate from logged data:
```bash
make results
```

## License

MIT
