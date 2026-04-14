#!/usr/bin/env python3
"""
Parameter Optimization Framework for Point-LIO UTLiDAR Configuration

This script optimizes the utlidar.yaml parameters using Bayesian optimization
with early stopping. It plays a rosbag file, runs Point-LIO with different
parameter combinations, and minimizes ATE (Absolute Trajectory Error) between
estimated and mocap ground truth trajectories.

Usage:
    python3 optimize_utlidar.py --rosbag /path/to/rosbag [--trials N] [--timeout S]
"""

import argparse
import os
import sys
import time
import yaml
import subprocess
import json
import threading
from pathlib import Path as FilePath
from collections import deque
from typing import Dict, Tuple, Any, Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    from scipy.spatial.transform import Rotation as R
except ImportError:
    print("⚠️  ROS 2 Python packages not available. This script must be run inside the Docker environment.")
    print("   Expected to import: rclpy, sensor_msgs, geometry_msgs, scipy")
    sys.exit(1)

try:
    import optuna
    from optuna.pruners import MedianPruner
except ImportError:
    print("⚠️  Optuna not installed. Installing...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "optuna", "-q"])
    import optuna
    from optuna.pruners import MedianPruner


# ============================================================================
# Configuration
# ============================================================================

# Parameters to optimize and their bounds
PARAM_BOUNDS = {
    # LiDAR measurement covariance (higher = trust LiDAR less)
    'lidar_meas_cov': (0.001, 0.1),

    # IMU acceleration measurement covariance (higher = trust IMU accel less)
    'imu_meas_acc_cov': (1.0, 50.0),

    # IMU angular velocity measurement covariance (higher = trust IMU gyro less)
    'imu_meas_omg_cov': (1.0, 20.0),

    # Acceleration bias covariance
    'b_acc_cov': (0.001, 0.1),

    # Gyro bias covariance
    'b_gyr_cov': (0.001, 0.1),

    # Plane flatness threshold (lower = stricter plane detection)
    'plane_thr': (0.05, 0.5),

    # Surface downsampling filter size
    'filter_size_surf': (0.1, 0.5),

    # Map downsampling filter size
    'filter_size_map': (0.1, 0.5),
}

# Fixed parameters (usually shouldn't change)
FIXED_PARAMS = {
    'use_sim_time': True,
    'use_imu_as_input': True,
    'runtime_pos_log_enable': False,
}

# Optimization settings
OPTIMIZATION_CONFIG = {
    'target_ate_threshold': 0.5,      # Stop if ATE < 0.5 meters
    'max_drift_threshold': 1.5,       # Stop iteration if current ATE exceeds this (meters)
    'patience': 5,                     # Early stopping: iterations without improvement
    'initial_trials': 20,              # Number of random trials before optimization
    'max_trials': 100,                 # Maximum optimization iterations
    'trajectory_collection_timeout': 60.0,  # Max time to collect trajectory (seconds)
}


# ============================================================================
# Trajectory Collection & Processing
# ============================================================================

class TrajectoryCollector(Node):
    """ROS 2 node that collects odometry and ground truth trajectories."""

    def __init__(self):
        super().__init__('trajectory_collector')

        self.odometry_poses = deque(maxlen=5000)
        self.mocap_poses = deque(maxlen=5000)
        self.collection_lock = threading.Lock()
        self.collection_complete = threading.Event()

        # Subscribe to Point-LIO body-center path (from trajectory_node)
        self.body_path_sub = self.create_subscription(
            Path,
            '/body_path',
            self.body_path_callback,
            10
        )

        # Subscribe to mocap ground truth
        self.mocap_sub = self.create_subscription(
            Path,
            '/mocap_path',
            self.mocap_callback,
            10
        )

        self.get_logger().info("TrajectoryCollector initialized")

    def body_path_callback(self, msg: Path):
        """Collect Point-LIO body-center path poses."""
        with self.collection_lock:
            self.odometry_poses.clear()
            for pose_stamped in msg.poses:
                pose = {
                    'timestamp': pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9,
                    'position': np.array([
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z
                    ]),
                    'orientation': R.from_quat([
                        pose_stamped.pose.orientation.x,
                        pose_stamped.pose.orientation.y,
                        pose_stamped.pose.orientation.z,
                        pose_stamped.pose.orientation.w
                    ])
                }
                self.odometry_poses.append(pose)

    def mocap_callback(self, msg: Path):
        """Collect mocap ground truth poses."""
        with self.collection_lock:
            self.mocap_poses.clear()
            for pose_stamped in msg.poses:
                pose = {
                    'timestamp': pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9,
                    'position': np.array([
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z
                    ]),
                    'orientation': R.from_quat([
                        pose_stamped.pose.orientation.x,
                        pose_stamped.pose.orientation.y,
                        pose_stamped.pose.orientation.z,
                        pose_stamped.pose.orientation.w
                    ])
                }
                self.mocap_poses.append(pose)

    def get_trajectories(self) -> Tuple[list, list]:
        """Return collected trajectories."""
        with self.collection_lock:
            return list(self.odometry_poses), list(self.mocap_poses)

    def clear(self):
        """Clear collected trajectories."""
        with self.collection_lock:
            self.odometry_poses.clear()
            self.mocap_poses.clear()


# Note: Post-hoc alignment removed. trajectory_node.py already applies frame alignment.


def compute_ate(est_traj: list, gt_traj: list) -> Tuple[float, int]:
    """
    Compute Absolute Trajectory Error (ATE) between trajectories.

    Uses trajectories as-is from trajectory_node (frame alignment already applied).
    No post-hoc alignment.

    ATE = sqrt(mean(||p_i - p_i_gt||^2))

    Args:
        est_traj: Estimated trajectory (from /body_path)
        gt_traj: Ground truth trajectory (from /mocap_path, already frame-aligned)

    Returns:
        ATE error, number of aligned poses
    """
    if len(est_traj) < 2 or len(gt_traj) < 2:
        return float('inf'), 0

    # Compute ATE by temporal alignment (no post-hoc alignment)
    errors = []
    aligned_count = 0

    g_idx = 0
    for e_pose in est_traj:
        # Find nearest gt pose by timestamp
        while g_idx < len(gt_traj) - 1 and \
              abs(gt_traj[g_idx + 1]['timestamp'] - e_pose['timestamp']) < \
              abs(gt_traj[g_idx]['timestamp'] - e_pose['timestamp']):
            g_idx += 1

        if g_idx < len(gt_traj):
            error = np.linalg.norm(e_pose['position'] - gt_traj[g_idx]['position'])
            errors.append(error)
            aligned_count += 1

    ate = np.sqrt(np.mean(np.array(errors) ** 2)) if errors else float('inf')

    return ate, aligned_count


# ============================================================================
# Configuration Management
# ============================================================================

class ConfigManager:
    """Manages yaml configuration file modifications."""

    def __init__(self, config_path: FilePath):
        self.config_path = config_path
        self.backup_path = config_path.with_suffix('.bak')
        self.original_config = self._load_config()

        # Backup original
        self._load_config()
        with open(self.backup_path, 'w') as f:
            yaml.dump(self.original_config, f)

    def _load_config(self) -> dict:
        """Load yaml configuration."""
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)

    def update_parameters(self, params: Dict[str, float]) -> bool:
        """
        Update specific parameters in yaml.

        Args:
            params: Dictionary of parameter names and values

        Returns:
            True if successful
        """
        try:
            config = self._load_config()

            # Update parameters (they're nested under /**:/ros__parameters:/mapping:)
            mapping = config['/**']['ros__parameters'].get('mapping', {})
            for key, value in params.items():
                if key in mapping:
                    mapping[key] = value

            with open(self.config_path, 'w') as f:
                yaml.dump(config, f)

            return True
        except Exception as e:
            print(f"❌ Failed to update config: {e}")
            return False

    def restore_original(self):
        """Restore original configuration."""
        with open(self.backup_path, 'r') as f:
            config = yaml.safe_load(f)
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f)


# ============================================================================
# Optimization
# ============================================================================

class ParameterOptimizer:
    """Bayesian optimization framework using Optuna."""

    def __init__(self,
                 config_path: FilePath,
                 rosbag_path: FilePath,
                 param_bounds: Dict[str, Tuple[float, float]],
                 output_dir: FilePath = None):
        """
        Initialize optimizer.

        Args:
            config_path: Path to utlidar.yaml
            rosbag_path: Path to rosbag directory
            param_bounds: Parameter bounds for optimization
            output_dir: Directory to save results
        """
        self.config_path = FilePath(config_path)
        self.rosbag_path = FilePath(rosbag_path)
        self.param_bounds = param_bounds
        self.output_dir = FilePath(output_dir or self.rosbag_path.parent / 'optimization_results')
        self.output_dir.mkdir(exist_ok=True)

        # Validate paths exist
        if not self.rosbag_path.exists():
            raise FileNotFoundError(f"❌ Rosbag not found: {self.rosbag_path}")
        if not self.config_path.exists():
            raise FileNotFoundError(f"❌ Config not found: {self.config_path}")

        self.config_manager = ConfigManager(self.config_path)
        self.best_ate = float('inf')
        self.best_params = None
        self.trial_results = []
        self.consecutive_no_improve = 0

    def run_trial(self, params: Dict[str, float]) -> Tuple[float, int]:
        """
        Execute single optimization trial.

        Args:
            params: Parameter dictionary

        Returns:
            ATE error, number of aligned poses
        """
        print(f"\n{'='*70}")
        print(f"Testing parameters: {params}")
        print(f"{'='*70}")

        # Update configuration
        if not self.config_manager.update_parameters(params):
            return float('inf'), 0

        # Run ROS nodes and collect trajectory
        ate, aligned_count = self._run_ros_system()

        # Track results
        trial_data = {
            'params': params,
            'ate': ate,
            'aligned_count': aligned_count,
            'timestamp': time.time()
        }
        self.trial_results.append(trial_data)

        # Save results incrementally
        self._save_results()

        print(f"✓ Trial result: ATE={ate:.4f}m, aligned_poses={aligned_count}")

        # Update best
        if ate < self.best_ate:
            self.best_ate = ate
            self.best_params = params.copy()
            self.consecutive_no_improve = 0
            print(f"🎯 New best! ATE={self.best_ate:.4f}m")
        else:
            self.consecutive_no_improve += 1
            print(f"⏱️  No improvement ({self.consecutive_no_improve}/{OPTIMIZATION_CONFIG['patience']})")

        return ate, aligned_count

    def _run_ros_system(self) -> Tuple[float, int]:
        """
        Run Point-LIO with rosbag and collect trajectory.

        This function:
        1. Launches point_lio mapping_utlidar.launch
        2. Launches trajectory_node (mocap connection & alignment)
        3. Launches rosbag playback
        4. Collects body_path and mocap trajectories
        5. Computes ATE
        """
        try:
            # Initialize ROS
            rclpy.init(args=None)
            collector = TrajectoryCollector()

            # Step 1: Launch Point-LIO
            pointlio_proc = subprocess.Popen(
                ['ros2', 'launch', 'point_lio', 'mapping_utlidar.launch', 'rviz:=false'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)

            # Step 2: Launch trajectory_node (requires OptiTrack connection)
            trajectory_proc = subprocess.Popen(
                ['ros2', 'run', 'mocap_odometry', 'trajectory_node'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)

            # Step 3: Run rosbag in background
            rosbag_proc = subprocess.Popen(
                ['ros2', 'bag', 'play', str(self.rosbag_path), '-l'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Collect trajectories for specified timeout
            timeout_start = time.time()
            min_poses = 50
            drift_check_counter = 0
            early_termination = False

            while (time.time() - timeout_start) < OPTIMIZATION_CONFIG['trajectory_collection_timeout']:
                est_traj, gt_traj = collector.get_trajectories()

                # Check if we have enough poses
                if len(est_traj) >= min_poses and len(gt_traj) >= min_poses:
                    break

                # Monitor drift every 10 iterations
                drift_check_counter += 1
                if drift_check_counter >= 10 and len(est_traj) >= 20 and len(gt_traj) >= 20:
                    drift_check_counter = 0
                    current_ate, _ = compute_ate(est_traj, gt_traj)
                    
                    # Early termination if drift exceeds threshold
                    if current_ate > OPTIMIZATION_CONFIG['max_drift_threshold']:
                        print(f"⚠️  Current ATE ({current_ate:.3f}m) exceeds drift threshold ({OPTIMIZATION_CONFIG['max_drift_threshold']:.3f}m). Terminating early.")
                        early_termination = True
                        break

                # Spin ROS for short duration to collect messages
                rclpy.spin_once(collector, timeout_sec=0.1)
                time.sleep(0.1)

            # Get final trajectories
            est_traj, gt_traj = collector.get_trajectories()

            print(f"📊 Collected: {len(est_traj)} estimated poses, {len(gt_traj)} mocap poses")

            # Compute ATE
            ate, aligned_count = compute_ate(est_traj, gt_traj)

            # Cleanup
            pointlio_proc.terminate()
            trajectory_proc.terminate()
            rosbag_proc.terminate()
            time.sleep(1)
            pointlio_proc.kill()
            trajectory_proc.kill()
            rosbag_proc.kill()

            collector.destroy_node()
            rclpy.shutdown()

            # Return infinity if terminated early due to excessive drift
            if early_termination:
                return float('inf'), aligned_count
            
            return ate, aligned_count

        except Exception as e:
            print(f"❌ Error running ROS system: {e}")
            try:
                rclpy.shutdown()
            except:
                pass
            return float('inf'), 0

    def objective(self, trial: optuna.Trial) -> float:
        """Optuna objective function."""
        # Suggest parameters
        params = {}
        for param_name, (low, high) in self.param_bounds.items():
            params[param_name] = trial.suggest_float(param_name, low, high)

        # Run trial
        ate, _ = self.run_trial(params)

        # Report for pruning
        trial.report(ate, step=len(self.trial_results))

        # Check if we should prune
        if trial.should_prune():
            raise optuna.TrialPruned()

        # Early stopping by patience
        if self.consecutive_no_improve >= OPTIMIZATION_CONFIG['patience']:
            raise optuna.TrialPruned()

        return ate

    def optimize(self, n_trials: int = None, timeout: float = None):
        """
        Run optimization study.

        Args:
            n_trials: Number of trials
            timeout: Timeout in seconds
        """
        n_trials = n_trials or OPTIMIZATION_CONFIG['max_trials']

        sampler = optuna.samplers.TPESampler(seed=42)
        pruner = MedianPruner(n_startup_trials=OPTIMIZATION_CONFIG['initial_trials'])

        study = optuna.create_study(
            sampler=sampler,
            pruner=pruner,
            direction='minimize'
        )

        print(f"\n{'='*70}")
        print(f"🚀 Starting Parameter Optimization")
        print(f"   Config: {self.config_path}")
        print(f"   Rosbag: {self.rosbag_path}")
        print(f"   Trials: {n_trials}, Timeout: {timeout}s")
        print(f"   Target ATE: {OPTIMIZATION_CONFIG['target_ate_threshold']}m")
        print(f"{'='*70}\n")

        study.optimize(
            self.objective,
            n_trials=n_trials,
            timeout=timeout,
            catch=(Exception,)
        )

        self._print_summary(study)
        self.config_manager.restore_original()

    def _save_results(self):
        """Save optimization results to JSON."""
        results_file = self.output_dir / 'optimization_log.json'
        with open(results_file, 'w') as f:
            json.dump(self.trial_results, f, indent=2, default=str)

    def _print_summary(self, study):
        """Print optimization summary."""
        print(f"\n{'='*70}")
        print(f"📈 Optimization Complete!")
        print(f"{'='*70}")
        print(f"Best ATE: {self.best_ate:.4f}m")
        print(f"Best parameters:")
        for param, value in self.best_params.items():
            print(f"  - {param}: {value:.6f}")
        print(f"Total trials: {len(self.trial_results)}")
        print(f"Results saved to: {self.output_dir}")
        print(f"{'='*70}\n")

        # Save best config
        self.config_manager.update_parameters(self.best_params)
        print(f"✓ Best configuration saved to: {self.config_path}")


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Optimize Point-LIO UTLiDAR parameters using trajectory error minimization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic optimization with default rosbag
  python3 optimize_utlidar.py

  # Specify rosbag and number of trials
  python3 optimize_utlidar.py --rosbag /path/to/rosbag --trials 50

  # Set target error threshold
  python3 optimize_utlidar.py --target-ate 0.3
        """
    )

    parser.add_argument(
        '--rosbag',
        default='/data/rosbag',
        help='Path to rosbag directory (default: /data/rosbag when running in Docker)'
    )

    parser.add_argument(
        '--config',
        default='point_lio/config/utlidar.yaml',
        help='Path to utlidar.yaml config file (default: point_lio/config/utlidar.yaml)'
    )

    parser.add_argument(
        '--trials',
        type=int,
        default=OPTIMIZATION_CONFIG['max_trials'],
        help=f"Maximum number of trials (default: {OPTIMIZATION_CONFIG['max_trials']})"
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=None,
        help='Optimization timeout in seconds (default: None)'
    )

    parser.add_argument(
        '--target-ate',
        type=float,
        default=OPTIMIZATION_CONFIG['target_ate_threshold'],
        help=f"Target ATE error threshold in meters (default: {OPTIMIZATION_CONFIG['target_ate_threshold']})"
    )

    parser.add_argument(
        '--max-drift',
        type=float,
        default=OPTIMIZATION_CONFIG['max_drift_threshold'],
        help=f"Max drift threshold - terminate iteration if ATE exceeds this (default: {OPTIMIZATION_CONFIG['max_drift_threshold']})"
    )

    parser.add_argument(
        '--output-dir',
        default=None,
        help='Directory to save optimization results'
    )

    args = parser.parse_args()

    # Update global config
    OPTIMIZATION_CONFIG['target_ate_threshold'] = args.target_ate
    OPTIMIZATION_CONFIG['max_drift_threshold'] = args.max_drift

    # Run optimization (ParameterOptimizer converts strings to Path internally)
    optimizer = ParameterOptimizer(
        config_path=args.config,
        rosbag_path=args.rosbag,
        param_bounds=PARAM_BOUNDS,
        output_dir=args.output_dir
    )

    optimizer.optimize(n_trials=args.trials, timeout=args.timeout)


if __name__ == '__main__':
    main()
