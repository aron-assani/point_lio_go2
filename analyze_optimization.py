#!/usr/bin/env python3
"""
Trajectory Analysis & Visualization Tool

Analyze and visualize optimization results, comparing different parameter sets
and their corresponding trajectories.
"""

import json
import argparse
import sys
from pathlib import Path
from typing import List, Dict, Any
import numpy as np
from tabulate import tabulate

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    print("⚠️  matplotlib not installed. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib", "tabulate", "-q"])
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from tabulate import tabulate


def load_optimization_results(results_file: Path) -> List[Dict[str, Any]]:
    """Load optimization results from JSON."""
    if not results_file.exists():
        print(f"❌ Results file not found: {results_file}")
        sys.exit(1)

    with open(results_file, 'r') as f:
        return json.load(f)


def print_results_table(results: List[Dict[str, Any]], top_n: int = 10):
    """Print results as formatted table."""
    print(f"\n📊 Top {min(top_n, len(results))} Best Results:")
    print("=" * 80)

    # Sort by ATE
    sorted_results = sorted(results, key=lambda x: x['ate'])

    headers = ['Rank', 'ATE (m)', 'Poses', 'Parameters']
    table_data = []

    for i, result in enumerate(sorted_results[:top_n]):
        params_str = '\n'.join(
            f"  {k}: {v:.6f}" for k, v in result['params'].items()
        )
        table_data.append([
            i + 1,
            f"{result['ate']:.4f}",
            result.get('aligned_count', 'N/A'),
            params_str
        ])

    print(tabulate(table_data, headers=headers, tablefmt='grid'))


def plot_convergence(results: List[Dict[str, Any]], output_file: Path = None):
    """Plot optimization convergence."""
    ate_values = [r['ate'] for r in results]
    best_ate = [min(ate_values[:i+1]) for i in range(len(ate_values))]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Individual trial results
    ax1.plot(ate_values, 'o-', alpha=0.6, label='Trial ATE')
    ax1.plot(best_ate, 'r-', linewidth=2, label='Best ATE')
    ax1.set_xlabel('Trial Number')
    ax1.set_ylabel('ATE (meters)')
    ax1.set_title('Optimization Convergence')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Distribution of results
    ax2.hist(ate_values, bins=20, alpha=0.7, edgecolor='black')
    ax2.axvline(min(ate_values), color='r', linestyle='--', label=f'Min: {min(ate_values):.4f}m')
    ax2.set_xlabel('ATE (meters)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('ATE Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150)
        print(f"✓ Convergence plot saved: {output_file}")
    else:
        plt.show()

    plt.close()


def plot_parameter_sensitivity(results: List[Dict[str, Any]], output_file: Path = None):
    """Plot parameter sensitivity (ATE vs each parameter)."""
    if not results or not results[0].get('params'):
        print("⚠️  No parameter data available")
        return

    param_names = list(results[0]['params'].keys())
    ate_values = np.array([r['ate'] for r in results])

    # Create subplots for each parameter
    n_params = len(param_names)
    n_cols = 3
    n_rows = (n_params + n_cols - 1) // n_cols

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 5 * n_rows))
    axes = axes.flatten() if n_params > 1 else [axes]

    for idx, param_name in enumerate(param_names):
        ax = axes[idx]
        param_values = np.array([r['params'][param_name] for r in results])

        # Sort by parameter value for clearer visualization
        sort_idx = np.argsort(param_values)
        sorted_params = param_values[sort_idx]
        sorted_ate = ate_values[sort_idx]

        ax.scatter(sorted_params, sorted_ate, alpha=0.6, s=50)
        ax.plot(sorted_params, sorted_ate, 'b-', alpha=0.3)
        ax.set_xlabel(param_name)
        ax.set_ylabel('ATE (meters)')
        ax.set_title(f'Sensitivity: {param_name}')
        ax.grid(True, alpha=0.3)

    # Hide unused subplots
    for idx in range(len(param_names), len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150)
        print(f"✓ Sensitivity plot saved: {output_file}")
    else:
        plt.show()

    plt.close()


def analyze_parameter_impact(results: List[Dict[str, Any]]):
    """Analyze which parameters have the most impact on ATE."""
    if not results or not results[0].get('params'):
        print("⚠️  No parameter data available")
        return

    param_names = list(results[0]['params'].keys())
    ate_values = np.array([r['ate'] for r in results])

    print(f"\n🔍 Parameter Impact Analysis:")
    print("=" * 60)

    correlations = {}
    for param_name in param_names:
        param_values = np.array([r['params'][param_name] for r in results])
        # Compute Pearson correlation
        if np.std(param_values) > 0 and np.std(ate_values) > 0:
            correlation = np.corrcoef(param_values, ate_values)[0, 1]
        else:
            correlation = 0.0

        correlations[param_name] = abs(correlation)

    # Sort by impact
    sorted_params = sorted(correlations.items(), key=lambda x: x[1], reverse=True)

    print(f"{'Parameter':<25} {'Correlation (|r|)':<20}")
    print("-" * 60)
    for param_name, corr in sorted_params:
        print(f"{param_name:<25} {corr:.4f}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze and visualize parameter optimization results'
    )

    parser.add_argument(
        '--results',
        type=Path,
        default=Path('tester_rosbag/optimization_results/optimization_log.json'),
        help='Path to optimization_log.json results file'
    )

    parser.add_argument(
        '--top',
        type=int,
        default=10,
        help='Number of top results to display'
    )

    parser.add_argument(
        '--plot-convergence',
        action='store_true',
        help='Plot optimization convergence'
    )

    parser.add_argument(
        '--plot-sensitivity',
        action='store_true',
        help='Plot parameter sensitivity'
    )

    parser.add_argument(
        '--analyze',
        action='store_true',
        help='Analyze parameter impact on ATE'
    )

    parser.add_argument(
        '--all',
        action='store_true',
        help='Run all analyses'
    )

    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='Directory to save plots'
    )

    args = parser.parse_args()

    # Load results
    results = load_optimization_results(args.results)
    print(f"✓ Loaded {len(results)} trial results")

    # Set output directory
    output_dir = args.output_dir or args.results.parent
    output_dir.mkdir(exist_ok=True)

    # Print summary
    print_results_table(results, args.top)

    # Run analyses
    if args.all or args.analyze:
        analyze_parameter_impact(results)

    if args.all or args.plot_convergence:
        output_file = output_dir / 'convergence.png' if args.output_dir else None
        plot_convergence(results, output_file)

    if args.all or args.plot_sensitivity:
        output_file = output_dir / 'sensitivity.png' if args.output_dir else None
        plot_parameter_sensitivity(results, output_file)

    print(f"\n✓ Analysis complete!")


if __name__ == '__main__':
    main()
