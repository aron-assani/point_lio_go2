#!/usr/bin/env python3
"""
Trajectory Analysis and Visualization Tool
Analyze Point-LIO optimization results and compare trajectories.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys


def load_trajectory(filepath):
    """Load trajectory from text file (timestamp x y z qx qy qz qw)."""
    try:
        data = np.loadtxt(filepath, skiprows=1)
        if len(data.shape) == 1:
            data = data.reshape(1, -1)
        return data
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None


def compute_ae(slam_traj, mocap_traj):
    """Compute absolute errors at synchronized timestamps."""
    if slam_traj is None or mocap_traj is None:
        return None
    
    slam_ts = slam_traj[:, 0]
    mocap_ts = mocap_traj[:, 0]
    
    # Find common time range
    min_ts = max(slam_ts.min(), mocap_ts.min())
    max_ts = min(slam_ts.max(), mocap_ts.max())
    
    if max_ts <= min_ts:
        print("No overlapping timestamps")
        return None
    
    # Use mocap timestamps as reference and interpolate SLAM
    mocap_mask = (mocap_ts >= min_ts) & (mocap_ts <= max_ts)
    mocap_ts_subset = mocap_ts[mocap_mask]
    mocap_pos = mocap_traj[mocap_mask, 1:4]
    
    # Interpolate SLAM positions
    slam_pos_interp = np.zeros((len(mocap_ts_subset), 3))
    for i in range(3):
        slam_pos_interp[:, i] = np.interp(mocap_ts_subset, slam_ts, slam_traj[:, i+1])
    
    # Compute errors
    errors = np.linalg.norm(slam_pos_interp - mocap_pos, axis=1)
    
    return {
        'timestamps': mocap_ts_subset,
        'positions_slam': slam_pos_interp,
        'positions_mocap': mocap_pos,
        'errors': errors,
        'ate': np.sqrt(np.mean(errors**2)),
        'rpe_trans': np.mean(np.linalg.norm(np.diff(slam_pos_interp - mocap_pos, axis=0), axis=1))
    }


def plot_trajectories(results, output_path='trajectories.png'):
    """Plot trajectories comparison."""
    if results is None:
        print("No results to plot")
        return
    
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectories
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(results['positions_slam'][:, 0], results['positions_slam'][:, 1], 
             results['positions_slam'][:, 2], 'b-', label='SLAM', linewidth=2)
    ax1.plot(results['positions_mocap'][:, 0], results['positions_mocap'][:, 1], 
             results['positions_mocap'][:, 2], 'r--', label='MoCap', linewidth=2)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectories')
    ax1.legend()
    
    # XY projection
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(results['positions_slam'][:, 0], results['positions_slam'][:, 1], 'b-', label='SLAM', linewidth=2)
    ax2.plot(results['positions_mocap'][:, 0], results['positions_mocap'][:, 1], 'r--', label='MoCap', linewidth=2)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_aspect('equal')
    ax2.set_title('XY Projection')
    ax2.legend()
    ax2.grid(True)
    
    # Error over time
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(results['timestamps'] - results['timestamps'][0], results['errors'], 'g-', linewidth=2)
    ax3.axhline(y=results['ate'], color='k', linestyle='--', label=f"ATE: {results['ate']:.3f} m")
    ax3.set_xlabel('Time from start (s)')
    ax3.set_ylabel('Position Error (m)')
    ax3.set_title('Absolute Error Over Time')
    ax3.legend()
    ax3.grid(True)
    
    # Individual axis errors
    ax4 = fig.add_subplot(2, 3, 4)
    diff = results['positions_slam'] - results['positions_mocap']
    ax4.plot(results['timestamps'] - results['timestamps'][0], diff[:, 0], 'r-', label='X error', linewidth=1.5)
    ax4.plot(results['timestamps'] - results['timestamps'][0], diff[:, 1], 'g-', label='Y error', linewidth=1.5)
    ax4.plot(results['timestamps'] - results['timestamps'][0], diff[:, 2], 'b-', label='Z error', linewidth=1.5)
    ax4.set_xlabel('Time from start (s)')
    ax4.set_ylabel('Error (m)')
    ax4.set_title('Error by Axis')
    ax4.legend()
    ax4.grid(True)
    
    # Error distribution
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.hist(results['errors'], bins=30, edgecolor='black', alpha=0.7)
    ax5.axvline(x=results['ate'], color='r', linestyle='--', linewidth=2, label=f'Mean: {results["ate"]:.3f} m')
    ax5.set_xlabel('Error (m)')
    ax5.set_ylabel('Frequency')
    ax5.set_title('Error Distribution')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Stats text
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    stats_text = f"""
    Trajectory Statistics
    
    Duration: {results['timestamps'][-1] - results['timestamps'][0]:.1f} s
    Points: {len(results['errors'])}
    
    ATE (RMSE): {results['ate']:.6f} m
    RPE (Translation): {results['rpe_trans']:.6f} m/m
    
    Error Min: {results['errors'].min():.6f} m
    Error Max: {results['errors'].max():.6f} m
    Error Std: {np.std(results['errors']):.6f} m
    Error Median: {np.median(results['errors']):.6f} m
    """
    ax6.text(0.1, 0.5, stats_text, fontsize=11, verticalalignment='center',
             family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved to {output_path}")
    plt.show()


def main():
    if len(sys.argv) < 3:
        print("Usage: trajectory_analyzer.py <slam_trajectory.txt> <mocap_trajectory.txt> [output.png]")
        sys.exit(1)
    
    slam_file = sys.argv[1]
    mocap_file = sys.argv[2]
    output_file = sys.argv[3] if len(sys.argv) > 3 else 'trajectory_comparison.png'
    
    print(f"Loading trajectories...")
    slam_traj = load_trajectory(slam_file)
    mocap_traj = load_trajectory(mocap_file)
    
    if slam_traj is None or mocap_traj is None:
        sys.exit(1)
    
    print(f"SLAM trajectory: {len(slam_traj)} points")
    print(f"MoCap trajectory: {len(mocap_traj)} points")
    
    print(f"Computing trajectory errors...")
    results = compute_ae(slam_traj, mocap_traj)
    
    if results:
        print(f"\n{'='*50}")
        print(f"Trajectory Analysis Results")
        print(f"{'='*50}")
        print(f"Absolute Trajectory Error (ATE): {results['ate']:.6f} m")
        print(f"Relative Pose Error (RPE):       {results['rpe_trans']:.6f} m/m")
        print(f"Max Error: {results['errors'].max():.6f} m")
        print(f"Min Error: {results['errors'].min():.6f} m")
        print(f"Mean Error: {results['errors'].mean():.6f} m")
        print(f"Std Dev: {np.std(results['errors']):.6f} m")
        print(f"Median Error: {np.median(results['errors']):.6f} m")
        print(f"{'='*50}\n")
        
        plot_trajectories(results, output_file)
    else:
        print("Failed to compute trajectory errors")
        sys.exit(1)


if __name__ == '__main__':
    main()
