"""
System identification for the espresso pressure/flow plant.

Generates chirp excitation signals, logs the response, fits a
first-order-plus-dead-time (FOPDT) model, and produces Bode plots.

Usage:
    python system_identification.py --data ../data/chirp_response.csv
    python system_identification.py --generate-chirp --duration 30 --f0 0.1 --f1 10
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, optimize
from pathlib import Path


def generate_chirp(duration: float, fs: float, f0: float, f1: float,
                   amplitude: float = 0.3, offset: float = 0.5) -> tuple:
    """Generate a chirp signal for pump excitation."""
    t = np.arange(0, duration, 1.0 / fs)
    chirp_signal = offset + amplitude * signal.chirp(t, f0, duration, f1, method='logarithmic')
    chirp_signal = np.clip(chirp_signal, 0.0, 1.0)
    return t, chirp_signal


def fopdt_model(t, K, tau, theta):
    """First-order-plus-dead-time step response."""
    y = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti > theta:
            y[i] = K * (1.0 - np.exp(-(ti - theta) / tau))
    return y


def fit_fopdt(t, input_signal, output_signal) -> dict:
    """Fit FOPDT model to step response data using least squares."""
    # Find the step: use the mean of the first and last quarters
    u_before = np.mean(input_signal[:len(input_signal) // 4])
    u_after = np.mean(input_signal[-len(input_signal) // 4:])
    y_before = np.mean(output_signal[:len(output_signal) // 4])
    y_after = np.mean(output_signal[-len(output_signal) // 4:])

    delta_u = u_after - u_before
    delta_y = y_after - y_before

    if abs(delta_u) < 1e-6:
        raise ValueError("Input signal has no significant step change")

    K_guess = delta_y / delta_u
    tau_guess = (t[-1] - t[0]) * 0.2
    theta_guess = 0.5

    # Normalize output for fitting
    y_norm = (output_signal - y_before) / (delta_u if abs(delta_u) > 1e-6 else 1.0)
    t_norm = t - t[0]

    try:
        popt, pcov = optimize.curve_fit(
            fopdt_model, t_norm, y_norm,
            p0=[K_guess, tau_guess, theta_guess],
            bounds=([0, 0.01, 0], [100, 30, 5]),
            maxfev=10000
        )
    except RuntimeError:
        return {'K': K_guess, 'tau': tau_guess, 'theta': theta_guess, 'fit_success': False}

    return {
        'K': popt[0],
        'tau': popt[1],
        'theta': popt[2],
        'fit_success': True,
        'pcov': pcov,
    }


def compute_bode(K, tau, theta, f_range=None):
    """Compute Bode plot data for the FOPDT model."""
    if f_range is None:
        f_range = np.logspace(-2, 2, 500)

    omega = 2 * np.pi * f_range
    s = 1j * omega

    # G(s) = K * exp(-theta*s) / (tau*s + 1)
    G = K * np.exp(-theta * s) / (tau * s + 1)

    magnitude_db = 20 * np.log10(np.abs(G))
    phase_deg = np.degrees(np.angle(G))

    return f_range, magnitude_db, phase_deg


def empirical_bode(t, input_signal, output_signal, fs):
    """Compute empirical frequency response using FFT."""
    n = len(t)
    U = np.fft.rfft(input_signal - np.mean(input_signal))
    Y = np.fft.rfft(output_signal - np.mean(output_signal))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)

    # Avoid division by zero
    mask = np.abs(U) > 1e-10
    H = np.zeros_like(U, dtype=complex)
    H[mask] = Y[mask] / U[mask]

    magnitude_db = 20 * np.log10(np.abs(H) + 1e-20)
    phase_deg = np.degrees(np.angle(H))

    return freqs, magnitude_db, phase_deg


def plot_bode(freqs_model, mag_model, phase_model,
              freqs_emp=None, mag_emp=None, phase_emp=None,
              title='Bode Plot — Pressure Plant', save_path=None):
    """Generate Bode plot with model and optional empirical data."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(title)

    ax1.semilogx(freqs_model, mag_model, 'b-', linewidth=2, label='FOPDT Model')
    if freqs_emp is not None:
        ax1.semilogx(freqs_emp, mag_emp, 'r.', alpha=0.3, markersize=2, label='Empirical')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.grid(True, which='both', alpha=0.3)
    ax1.legend()

    ax2.semilogx(freqs_model, phase_model, 'b-', linewidth=2, label='FOPDT Model')
    if freqs_emp is not None:
        ax2.semilogx(freqs_emp, phase_emp, 'r.', alpha=0.3, markersize=2, label='Empirical')
    ax2.set_ylabel('Phase (deg)')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.grid(True, which='both', alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    plt.show()


def plot_step_response(t, input_signal, output_signal, model_params, save_path=None):
    """Plot step response with fitted model overlay."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Step Response — Pressure Plant')

    ax1.plot(t, input_signal, 'b-', label='Pump PWM')
    ax1.set_ylabel('Input (duty)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, output_signal, 'b-', alpha=0.5, label='Measured')

    t_model = t - t[0]
    u_step = np.mean(input_signal[-len(input_signal) // 4:]) - np.mean(input_signal[:len(input_signal) // 4])
    y_base = np.mean(output_signal[:len(output_signal) // 4])
    y_model = y_base + u_step * fopdt_model(t_model, model_params['K'], model_params['tau'], model_params['theta'])
    ax2.plot(t, y_model, 'r--', linewidth=2, label=f"FOPDT: K={model_params['K']:.2f}, τ={model_params['tau']:.2f}s, θ={model_params['theta']:.2f}s")

    ax2.set_ylabel('Pressure (bar)')
    ax2.set_xlabel('Time (s)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Espresso plant system identification')
    parser.add_argument('--data', type=str, help='Path to CSV with columns: time,pump_duty,pressure')
    parser.add_argument('--generate-chirp', action='store_true', help='Generate chirp excitation CSV')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--fs', type=float, default=100.0, help='Sample rate (Hz)')
    parser.add_argument('--f0', type=float, default=0.1, help='Chirp start frequency')
    parser.add_argument('--f1', type=float, default=10.0, help='Chirp end frequency')
    parser.add_argument('--output-dir', type=str, default='../../docs')
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.generate_chirp:
        t, chirp = generate_chirp(args.duration, args.fs, args.f0, args.f1)
        out_file = Path('../../data/chirp_excitation.csv')
        out_file.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(out_file, np.column_stack([t, chirp]),
                   header='time,pump_duty', delimiter=',', comments='')
        print(f"Chirp saved to {out_file}")
        print(f"Duration: {args.duration}s, {args.f0}-{args.f1} Hz, {int(args.fs)} Hz sample rate")
        return

    if args.data:
        data = np.genfromtxt(args.data, delimiter=',', skip_header=1)
        t = data[:, 0]
        pump_duty = data[:, 1]
        pressure = data[:, 2]

        print("Fitting FOPDT model...")
        params = fit_fopdt(t, pump_duty, pressure)
        print(f"  K     = {params['K']:.4f} (bar per unit duty)")
        print(f"  tau   = {params['tau']:.4f} s")
        print(f"  theta = {params['theta']:.4f} s (transport delay)")
        print(f"  Fit success: {params.get('fit_success', 'N/A')}")

        plot_step_response(t, pump_duty, pressure, params,
                           save_path=output_dir / 'step_response.png')

        f_model, mag_model, phase_model = compute_bode(params['K'], params['tau'], params['theta'])
        f_emp, mag_emp, phase_emp = empirical_bode(t, pump_duty, pressure, args.fs)

        plot_bode(f_model, mag_model, phase_model,
                  f_emp[1:], mag_emp[1:], phase_emp[1:],
                  save_path=output_dir / 'bode.png')
    else:
        # Demo with synthetic data
        print("No data file provided. Running with synthetic demo data.")
        fs = 100.0
        t = np.arange(0, 20, 1.0 / fs)

        # Synthetic plant: K=10, tau=0.8, theta=1.2
        pump_duty = np.zeros_like(t)
        pump_duty[int(2 * fs):] = 0.5  # step at t=2s

        K_true, tau_true, theta_true = 10.0, 0.8, 1.2
        pressure = np.zeros_like(t)
        for i in range(1, len(t)):
            dt = t[i] - t[i - 1]
            delayed_input = pump_duty[max(0, i - int(theta_true * fs))]
            pressure[i] = pressure[i - 1] + dt / tau_true * (K_true * delayed_input - pressure[i - 1])
        pressure += np.random.normal(0, 0.1, len(t))

        params = fit_fopdt(t, pump_duty, pressure)
        print(f"True:  K={K_true}, tau={tau_true}, theta={theta_true}")
        print(f"Fit:   K={params['K']:.3f}, tau={params['tau']:.3f}, theta={params['theta']:.3f}")

        plot_step_response(t, pump_duty, pressure, params,
                           save_path=output_dir / 'step_response.png')

        f_model, mag_model, phase_model = compute_bode(params['K'], params['tau'], params['theta'])
        plot_bode(f_model, mag_model, phase_model,
                  title='Bode Plot — Synthetic Plant',
                  save_path=output_dir / 'bode.png')


if __name__ == '__main__':
    main()
