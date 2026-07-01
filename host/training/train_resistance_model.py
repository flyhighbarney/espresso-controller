"""
Train a TinyML model that predicts puck resistance from the first 3 seconds
of a shot, for deployment on STM32 via TFLite Micro or CMSIS-NN.

Architecture:
    Input:  300 samples x 2 channels (pressure + flow at 100 Hz)
    Hidden: Dense(32, ReLU) → Dense(16, ReLU)
    Output: 3 values [R_initial, alpha, R_final]

Target size: < 5 KB quantized weights.

Usage:
    python train_resistance_model.py --data-dir ../../data --epochs 200
    python train_resistance_model.py --export-tflite model.tflite
    python train_resistance_model.py --export-c-array model_weights.h
"""

import argparse
import json
import os
import struct
import tempfile
from pathlib import Path

import numpy as np

try:
    import tensorflow as tf
    from tensorflow import keras
    HAS_TF = True
except ImportError:
    HAS_TF = False

ML_INPUT_SAMPLES = 300
ML_INPUT_CHANNELS = 2
ML_OUTPUT_SIZE = 3


def load_shot_data(data_dir: str) -> list:
    """Load shot CSVs and extract first 3 seconds + resistance labels."""
    data_path = Path(data_dir)
    shots = []

    for csv_file in sorted(data_path.glob('shot_*.csv')):
        data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        if data.shape[0] < ML_INPUT_SAMPLES:
            continue

        # Columns: timestamp, pressure, flow, temperature, pump_duty, heater_duty,
        #          setpoint, est_resistance, phase, state
        pressure = data[:ML_INPUT_SAMPLES, 1]  # pressure_bar
        flow = data[:ML_INPUT_SAMPLES, 2]      # flow_ml_s
        resistance = data[:, 7]                 # est_resistance

        # Labels: resistance at start, compaction rate, final resistance
        r_initial = np.mean(resistance[ML_INPUT_SAMPLES:ML_INPUT_SAMPLES + 50])
        r_final = np.mean(resistance[-50:])
        alpha = (r_final - r_initial) / (len(resistance) * 0.01 + 1e-6)

        x = np.column_stack([pressure, flow])
        y = np.array([r_initial, alpha, r_final])

        shots.append((x, y))

    return shots


def generate_synthetic_data(n_shots: int = 200) -> tuple:
    """Generate synthetic training data when real shot data is unavailable."""
    rng = np.random.RandomState(42)
    X, Y = [], []

    for _ in range(n_shots):
        r_initial = rng.uniform(5.0, 15.0)
        alpha = rng.uniform(0.0, 0.3)
        r_final = r_initial * (1.0 + alpha * 25.0) + rng.normal(0, 0.5)

        t = np.arange(ML_INPUT_SAMPLES) * 0.01  # 3 seconds at 100 Hz
        base_pressure = np.minimum(t * 3.0, 9.0) + rng.normal(0, 0.1, ML_INPUT_SAMPLES)
        base_flow = base_pressure / (r_initial + alpha * np.cumsum(t) * 0.1)
        base_flow += rng.normal(0, 0.02, ML_INPUT_SAMPLES)

        x = np.column_stack([base_pressure, np.maximum(base_flow, 0)])
        y = np.array([r_initial, alpha, r_final])

        X.append(x)
        Y.append(y)

    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)


def build_model():
    """Build a small model suitable for MCU deployment."""
    model = keras.Sequential([
        keras.layers.InputLayer(input_shape=(ML_INPUT_SAMPLES, ML_INPUT_CHANNELS)),
        keras.layers.Flatten(),
        keras.layers.Dense(32, activation='relu'),
        keras.layers.Dense(16, activation='relu'),
        keras.layers.Dense(ML_OUTPUT_SIZE),
    ])
    model.compile(optimizer='adam', loss='mse', metrics=['mae'])
    return model


def export_tflite(model, output_path: str, quantize: bool = True):
    """Export to TFLite format, optionally with int8 quantization."""
    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    if quantize:
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.target_spec.supported_types = [tf.int8]

        def representative_dataset():
            X_cal, _ = generate_synthetic_data(100)
            for i in range(min(100, len(X_cal))):
                yield [X_cal[i:i + 1]]

        converter.representative_dataset = representative_dataset

    tflite_model = converter.convert()

    with open(output_path, 'wb') as f:
        f.write(tflite_model)

    size_kb = len(tflite_model) / 1024
    print(f"Exported TFLite model: {output_path} ({size_kb:.1f} KB)")
    return tflite_model


def export_c_array(tflite_model: bytes, output_path: str, var_name: str = 'g_model_data'):
    """Export TFLite model as a C header file for embedding in firmware."""
    with open(output_path, 'w') as f:
        f.write(f'#ifndef MODEL_DATA_H\n#define MODEL_DATA_H\n\n')
        f.write(f'#include <stdint.h>\n\n')
        f.write(f'const unsigned int {var_name}_len = {len(tflite_model)};\n')
        f.write(f'alignas(16) const uint8_t {var_name}[] = {{\n')

        for i in range(0, len(tflite_model), 16):
            chunk = tflite_model[i:i + 16]
            hex_str = ', '.join(f'0x{b:02x}' for b in chunk)
            f.write(f'    {hex_str},\n')

        f.write(f'}};\n\n#endif\n')

    print(f"Exported C array: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Train puck resistance predictor')
    parser.add_argument('--data-dir', type=str, default='../../data')
    parser.add_argument('--epochs', type=int, default=200)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--export-tflite', type=str, default=None)
    parser.add_argument('--export-c-array', type=str, default=None)
    parser.add_argument('--synthetic', action='store_true', help='Use synthetic data')
    args = parser.parse_args()

    if not HAS_TF:
        print("TensorFlow required: pip install tensorflow")
        print("For MCU deployment: pip install tensorflow tflite-runtime")
        return

    # Load or generate data
    shots = load_shot_data(args.data_dir) if not args.synthetic else []

    if len(shots) < 20:
        print(f"Only {len(shots)} real shots found. Using synthetic data for training.")
        X_train, Y_train = generate_synthetic_data(500)
        X_val, Y_val = generate_synthetic_data(100)
    else:
        X = np.array([s[0] for s in shots], dtype=np.float32)
        Y = np.array([s[1] for s in shots], dtype=np.float32)
        split = int(0.8 * len(X))
        X_train, Y_train = X[:split], Y[:split]
        X_val, Y_val = X[split:], Y[split:]

    print(f"Training: {len(X_train)} shots, Validation: {len(X_val)} shots")

    # Normalize inputs
    X_mean = X_train.mean(axis=(0, 1), keepdims=True)
    X_std = X_train.std(axis=(0, 1), keepdims=True) + 1e-6
    X_train = (X_train - X_mean) / X_std
    X_val = (X_val - X_mean) / X_std

    # Build and train
    model = build_model()
    model.summary()

    history = model.fit(
        X_train, Y_train,
        validation_data=(X_val, Y_val),
        epochs=args.epochs,
        batch_size=args.batch_size,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(patience=20, restore_best_weights=True),
            keras.callbacks.ReduceLROnPlateau(factor=0.5, patience=10),
        ]
    )

    # Evaluate
    val_loss, val_mae = model.evaluate(X_val, Y_val, verbose=0)
    print(f"\nValidation MSE: {val_loss:.4f}, MAE: {val_mae:.4f}")

    # Count parameters
    total_params = model.count_params()
    param_bytes = total_params * 4  # float32
    print(f"Parameters: {total_params} ({param_bytes / 1024:.1f} KB float32)")

    # Export
    if args.export_tflite:
        tflite_data = export_tflite(model, args.export_tflite)
        if args.export_c_array:
            export_c_array(tflite_data, args.export_c_array)
    elif args.export_c_array:
        with tempfile.NamedTemporaryFile(suffix='.tflite', delete=False) as tmp:
            tmp_path = tmp.name
        try:
            tflite_data = export_tflite(model, tmp_path)
            export_c_array(tflite_data, args.export_c_array)
        finally:
            os.unlink(tmp_path)

    # Save normalization constants for firmware (atomic write)
    norm_path = Path(args.data_dir) / 'normalization.json'
    norm_data = {
        'input_mean': X_mean.squeeze().tolist(),
        'input_std': X_std.squeeze().tolist(),
    }
    tmp_norm = norm_path.with_suffix('.json.tmp')
    with open(tmp_norm, 'w') as f:
        json.dump(norm_data, f, indent=2)
    tmp_norm.replace(norm_path)
    print(f"Normalization constants saved: {norm_path}")


if __name__ == '__main__':
    main()
