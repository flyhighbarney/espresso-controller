# Adaptive espresso extraction controller

An STM32-based espresso rig that regulates pressure and flow rate during extraction. A 3-state Kalman filter estimates the coffee puck's hydraulic resistance on the fly, a gain-scheduled PID with Smith predictor handles the transport delay between pump and group head, and a small neural network predicts the puck's resistance curve from the first 3 seconds of each shot.

Shot-to-shot extraction time standard deviation drops by >40% compared to a tuned baseline PID, same grind, dose, and tamp.

## What it does

The controller runs three FreeRTOS tasks on an STM32G474RE (Cortex-M4F, 170 MHz):

- **Control task at 1 kHz** -- reads pressure and flow via ADC DMA, runs the Kalman filter, computes the controller output, writes pump PWM
- **Telemetry task at 100 Hz** -- streams COBS-framed binary packets over USB CDC to the host dashboard
- **Supervisor task at 10 Hz** -- manages the state machine (idle, preheat, pre-infusion, extraction, flush, fault) and enforces safety interlocks

The host side includes a real-time PyQt dashboard, system identification scripts, a TinyML training pipeline, and a browser-based simulator you can play with without any hardware.

## Control strategy

The project builds up in layers, each one testable against the last:

1. **Open-loop characterization** -- chirp excitation, FOPDT model fit (gain K, time constant tau, dead time theta), Bode plot
2. **Baseline PID** -- Ziegler-Nichols initial tune, hand-adjusted. Works, but lags on pucks with different grind or age because the transport delay shifts
3. **Kalman filter** -- 3-state extended Kalman filter tracking pressure, flow, and puck hydraulic resistance. Resistance is not directly measured; the filter infers it from pressure = flow * resistance
4. **Adaptive controller** -- gain-scheduled PID keyed on shot phase (pre-infusion, ramp, extraction, decline) and on the Kalman-estimated resistance. Wrapped in a Smith predictor that uses a circular delay buffer to compensate the pump-to-group-head transport delay
5. **TinyML predictor** -- a small fully-connected network (input: 300 samples x 2 channels = 600 features, hidden: 32 ReLU then 16 ReLU, output: R_initial, compaction rate alpha, R_final). Under 5 KB quantized via CMSIS-NN or TFLite Micro. Feeds the first 3 seconds of each shot to pre-tune the adaptive controller before the Kalman estimate converges

## Features

### Firmware (C, FreeRTOS)

- PID with anti-windup, configurable integral limits, and runtime gain adjustment
- 3-state Kalman filter with tunable process and measurement noise covariances
- Gain-scheduled adaptive controller with per-phase gain tables
- Smith predictor delay compensation via 64-sample circular buffer
- ML inference context that collects 3 seconds of downsampled sensor data (100 Hz effective) and runs a quantized neural network on-chip
- Sensor drivers for a 0-200 psi pressure transducer (0.5-4.5V), NTC 10k thermistor (Steinhart-Hart), and Hall-effect flow meter (input capture ISR)
- Actuator control for vibratory pump PWM, cartridge heater PWM with safety duty cap, and 3-way solenoid valve
- Supervisor state machine with fault detection: overpressure (>12 bar), overtemperature (>110C), dry-fire protection, sensor plausibility checks, watchdog timer
- COBS-framed telemetry over USB CDC with packed binary state packets (pressure, flow, temperature, pump duty, heater duty, setpoint, estimated resistance, shot phase, machine state)
- Optional Simulink Coder controller swap via build flag

### Host tools (Python)

- **Dashboard** (`host/dashboard/dashboard.py`) -- PyQt5 + pyqtgraph real-time GUI with live pressure/flow/resistance plots, PID gain tuning sliders, controller mode selector (baseline PID, adaptive, adaptive+ML), shot logging to CSV. Falls back to console mode without PyQt. Includes a simulation mode that generates synthetic shot profiles for testing without hardware
- **System identification** (`host/sysid/system_identification.py`) -- generates logarithmic chirp excitation signals, fits FOPDT models via least-squares, computes model and empirical Bode plots, renders step response overlays
- **ML training** (`host/training/train_resistance_model.py`) -- trains the puck resistance predictor on real or synthetic shot data, exports to TFLite with int8 quantization, generates a C header with the model weights for direct firmware embedding
- **Benchmarking** (`host/sysid/benchmark.py`) -- automated comparison runs

### Browser simulator (JavaScript)

A standalone in-browser simulation (`host/simulator/`) with the full control stack reimplemented in JS: plant model with configurable resistance, transport delay, and sensor noise; PID, Kalman filter, adaptive controller, and ML predictor; canvas-based live plots for pressure, flow, resistance, and pump duty; slider controls for all plant and controller parameters; and a 20-shot automated comparison that runs all three controller modes back-to-back and reports extraction time mean and standard deviation.

## Hardware

| Component | What it does |
|---|---|
| STM32 Nucleo-G474RE | MCU -- Cortex-M4F, 170 MHz, hardware FPU, DSP instructions |
| Ulka EP5 vibratory pump | Drives water through the puck, controlled via PWM |
| 0-200 psi pressure transducer | Measures line pressure (0.5-4.5V analog output) |
| YF-S401 Hall-effect flow meter | Pulse output, counted via timer input capture |
| 100W cartridge heater + aluminum block | Simulates a boiler for temperature control |
| NTC 10k thermistor | Temperature feedback for heater PID |
| 3-way solenoid valve | Pre-infusion and depressurization |
| 58mm bottomless portafilter | Holds the real coffee puck |

Total BOM is around $120.

## Safety

- Pump cuts and solenoid opens if pressure exceeds 12 bar
- Heater cuts if temperature exceeds 110C
- Heater won't fire without recent flow (dry-fire protection)
- Watchdog timer resets the MCU on missed control deadlines
- Sensor plausibility checks reject readings outside physical bounds (pressure: -0.5 to 16 bar, flow: 0 to 20 mL/s, temperature: -40 to 150C, ADC values within 10 counts of rails)
- All fluid-path materials are food-safe (PTFE, 304 stainless)

## Quick start

### Run the browser simulator (no dependencies)

Open `host/simulator/index.html` in a browser. Click "Start Shot" and adjust the sliders.

### Run the dashboard in simulation mode

```bash
pip install -r host/requirements.txt
make dashboard
```

### Build firmware

```bash
# requires arm-none-eabi-gcc
make firmware
```

### Run unit tests

```bash
make tests
```

### Train and export the ML model

```bash
make train-export
```

This trains on synthetic data, quantizes to int8, and writes both `model.tflite` and a C header (`firmware/App/ml/model_data.h`) with the weights baked in.

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

| Flag | What it does |
|---|---|
| `USE_SIMULINK_CONTROLLER` | Swaps in a Simulink Coder-generated controller instead of the hand-written one |
| `UNIT_TEST` | Compiles for host-side testing without the HAL |

## Results

See [docs/results.md](docs/results.md) for extraction time tables, pressure tracking RMSE, Kalman residuals, and side-by-side controller comparison plots.

All plots regenerate from logged data:
```bash
make results
```

## License

MIT
