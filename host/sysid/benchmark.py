"""
Benchmark analysis: compare shot-to-shot consistency across controller modes.

Reads logged shot CSVs from data/ and produces comparison plots and statistics
for the results page.

Usage:
    python benchmark.py --data-dir ../../data --output-dir ../../docs
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_shots(data_dir: str) -> dict:
    """Load shots grouped by controller mode from filename convention."""
    data_path = Path(data_dir)
    groups = {}

    for csv_file in sorted(data_path.glob('shot_*.csv')):
        data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        if data.shape[0] < 100:
            continue

        # Determine controller mode from filename or first column metadata
        name = csv_file.stem
        if 'baseline' in name:
            mode = 'Baseline PID'
        elif 'adaptive_ml' in name:
            mode = 'Adaptive + ML'
        elif 'adaptive' in name:
            mode = 'Adaptive'
        else:
            mode = 'Unknown'

        groups.setdefault(mode, []).append(data)

    return groups


def compute_extraction_time(shot_data: np.ndarray, flow_threshold: float = 0.3) -> float:
    """Estimate extraction time as duration where flow > threshold."""
    if shot_data.shape[1] < 3:
        return 0.0

    times = shot_data[:, 0] / 1000.0  # ms to s
    flow = shot_data[:, 2]

    flowing = flow > flow_threshold
    if not np.any(flowing):
        return 0.0

    first = np.argmax(flowing)
    last = len(flowing) - 1 - np.argmax(flowing[::-1])

    return times[last] - times[first]


def compute_pressure_rmse(shot_data: np.ndarray, target: float = 9.0) -> float:
    """RMSE of pressure vs target during extraction phase."""
    if shot_data.shape[1] < 7:
        return 0.0

    pressure = shot_data[:, 1]
    state = shot_data[:, -1] if shot_data.shape[1] > 9 else np.ones(len(pressure)) * 3

    extracting = state == 3  # STATE_EXTRACT
    if not np.any(extracting):
        # Fall back: use middle 60% of data
        n = len(pressure)
        extracting = np.zeros(n, dtype=bool)
        extracting[int(n * 0.2):int(n * 0.8)] = True

    return np.sqrt(np.mean((pressure[extracting] - target) ** 2))


def generate_synthetic_benchmark() -> dict:
    """Generate synthetic data for demo when no real shots exist."""
    rng = np.random.RandomState(42)
    groups = {}

    for mode, time_mean, time_std, p_noise in [
        ('Baseline PID', 27.0, 2.4, 0.8),
        ('Adaptive', 26.5, 1.4, 0.5),
        ('Adaptive + ML', 26.0, 0.9, 0.3),
    ]:
        shots = []
        for _ in range(20):
            n = int((time_mean + rng.normal(0, time_std)) * 100)
            n = max(500, min(4000, n))
            t = np.arange(n) * 10  # 100 Hz, in ms

            # Pressure profile
            t_s = t / 1000.0
            pressure = np.where(t_s < 5, t_s * 1.8,
                       np.where(t_s < 8, 3 + (t_s - 5) * 2, 9.0))
            pressure += rng.normal(0, p_noise, n)

            flow = pressure / (8 + rng.normal(0, 0.5, n))
            flow = np.maximum(flow, 0)

            data = np.column_stack([
                t, pressure, flow,
                93 + rng.normal(0, 0.3, n),  # temp
                np.clip(pressure / 12, 0, 1),  # pump duty
                np.full(n, 0.3),  # heater duty
                np.full(n, 9.0),  # setpoint
                8 + t_s * 0.2,  # resistance
                np.where(t_s < 5, 0, np.where(t_s < 8, 1, 2)),  # phase
                np.where(t_s < 5, 2, 3),  # state
            ])
            shots.append(data)
        groups[mode] = shots

    return groups


def plot_comparison(groups: dict, output_dir: Path):
    """Generate comparison plots."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Controller Comparison — Espresso Extraction', fontsize=14)

    colors = {'Baseline PID': '#e74c3c', 'Adaptive': '#3498db', 'Adaptive + ML': '#2ecc71'}

    # 1. Extraction time distribution
    ax = axes[0, 0]
    times_by_mode = {}
    for mode, shots in groups.items():
        times = [compute_extraction_time(s) for s in shots]
        times_by_mode[mode] = times

    positions = list(range(len(times_by_mode)))
    labels = list(times_by_mode.keys())
    bp = ax.boxplot(list(times_by_mode.values()), positions=positions,
                    patch_artist=True, widths=0.6)
    for patch, mode in zip(bp['boxes'], labels):
        patch.set_facecolor(colors.get(mode, '#888'))
        patch.set_alpha(0.7)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel('Extraction Time (s)')
    ax.set_title('Shot-to-Shot Consistency')
    ax.grid(True, alpha=0.3)

    # 2. Pressure traces overlay
    ax = axes[0, 1]
    for mode, shots in groups.items():
        for i, s in enumerate(shots[:5]):
            t = s[:, 0] / 1000.0
            p = s[:, 1]
            ax.plot(t, p, color=colors.get(mode, '#888'), alpha=0.3,
                    label=mode if i == 0 else None)
    ax.axhline(y=9.0, color='k', linestyle='--', alpha=0.5, label='Target')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pressure (bar)')
    ax.set_title('Pressure Traces (5 shots each)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3. Pressure RMSE
    ax = axes[1, 0]
    rmse_by_mode = {}
    for mode, shots in groups.items():
        rmse = [compute_pressure_rmse(s) for s in shots]
        rmse_by_mode[mode] = rmse

    positions = list(range(len(rmse_by_mode)))
    bp = ax.boxplot(list(rmse_by_mode.values()), positions=positions,
                    patch_artist=True, widths=0.6)
    for patch, mode in zip(bp['boxes'], list(rmse_by_mode.keys())):
        patch.set_facecolor(colors.get(mode, '#888'))
        patch.set_alpha(0.7)
    ax.set_xticks(positions)
    ax.set_xticklabels(list(rmse_by_mode.keys()), fontsize=9)
    ax.set_ylabel('Pressure RMSE (bar)')
    ax.set_title('Tracking Accuracy')
    ax.grid(True, alpha=0.3)

    # 4. Summary statistics table
    ax = axes[1, 1]
    ax.axis('off')
    table_data = []
    for mode in groups:
        times = times_by_mode.get(mode, [])
        rmse = rmse_by_mode.get(mode, [])
        if times and rmse:
            table_data.append([
                mode,
                f"{np.mean(times):.1f} +/- {np.std(times):.1f}",
                f"{np.mean(rmse):.2f}",
                str(len(times))
            ])

    if table_data:
        table = ax.table(cellText=table_data,
                         colLabels=['Controller', 'Time (s)', 'RMSE (bar)', 'N'],
                         loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1.5)

    plt.tight_layout()
    save_path = output_dir / 'comparison.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Saved: {save_path}")
    plt.show()


def print_summary(groups: dict):
    """Print summary statistics to console."""
    print("\n" + "=" * 60)
    print("BENCHMARK RESULTS")
    print("=" * 60)

    baseline_std = None
    for mode, shots in groups.items():
        times = [compute_extraction_time(s) for s in shots]
        rmse = [compute_pressure_rmse(s) for s in shots]

        t_mean, t_std = np.mean(times), np.std(times)
        r_mean = np.mean(rmse)

        if mode == 'Baseline PID':
            baseline_std = t_std

        improvement = ''
        if baseline_std and t_std < baseline_std:
            pct = (1 - t_std / baseline_std) * 100
            improvement = f' ({pct:.0f}% reduction)'

        print(f"\n{mode} (N={len(shots)}):")
        print(f"  Extraction time: {t_mean:.1f} +/- {t_std:.1f} s{improvement}")
        print(f"  Pressure RMSE:   {r_mean:.2f} bar")

    print("\n" + "=" * 60)


def main():
    parser = argparse.ArgumentParser(description='Benchmark controller variants')
    parser.add_argument('--data-dir', type=str, default='../../data')
    parser.add_argument('--output-dir', type=str, default='../../docs')
    parser.add_argument('--synthetic', action='store_true',
                        help='Use synthetic data for demo')
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    groups = load_shots(args.data_dir)

    if not groups or args.synthetic:
        print("Using synthetic data for demonstration.")
        groups = generate_synthetic_benchmark()

    print_summary(groups)
    plot_comparison(groups, output_dir)


if __name__ == '__main__':
    main()
