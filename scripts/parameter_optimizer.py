#!/usr/bin/env python3
"""
Point-LIO Parameter Optimizer using Rosbag and Ground Truth Mocap Data
Optimizes YAML parameters to minimize trajectory error vs mocap ground truth.
"""

import os
import sys
import subprocess
import time
import shutil
import tempfile
import yaml
import numpy as np
import signal
from pathlib import Path
from typing import Dict, List, Tuple
from scipy.optimize import differential_evolution, minimize
from scipy.spatial.transform import Rotation as R


class ParameterOptimizer:
    def __init__(self, rosbag_path: str, config_path: str, workspace: str = '/root/ros2_ws'):
        self.rosbag_path = rosbag_path
        self.config_path = config_path
        self.workspace = workspace
        self.processes = []
        self.iteration = 0
        
        # Default parameter ranges and values 
        self.param_bounds = {
            'imu_meas_acc_cov': (1.0, 50.0),
            'imu_meas_omg_cov': (0.5, 20.0),
            'lidar_meas_cov': (0.001, 0.1),
            'acc_cov_input': (0.5, 10.0),
            'gyr_cov_input': (0.05, 2.0),
            'plane_thr': (0.05, 0.5),
            'b_acc_cov': (0.001, 0.1),
            'b_gyr_cov': (0.001, 0.1),
        }
        
        self.param_defaults = {
            'imu_meas_acc_cov': 10.0,
            'imu_meas_omg_cov': 5.0,
            'lidar_meas_cov': 0.01,
            'acc_cov_input': 2.0,
            'gyr_cov_input': 0.1,
            'plane_thr': 0.1,
            'b_acc_cov': 0.01,
            'b_gyr_cov': 0.01,
        }
        
        self.param_names = list(self.param_bounds.keys())
        print(f"✓ Optimizer initialized. Optimizing {len(self.param_names)} parameters.")
        
    def cleanup_processes(self):
        """Terminate all spawned processes and kill rviz windows."""
        # Kill rviz windows explicitly
        subprocess.run('pkill -9 rviz2', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
        
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self.processes.clear()
        
    def __del__(self):
        self.cleanup_processes()
    
    def create_temp_config(self, param_dict: Dict[str, float]) -> str:
        """Create temporary YAML config with modified parameters."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Update parameters in the mapping section
        for param_name, value in param_dict.items():
            if param_name in config['/**']['ros__parameters']['mapping']:
                config['/**']['ros__parameters']['mapping'][param_name] = float(value)
        
        # Write to temp file
        temp_dir = Path(self.workspace) / 'temp_configs'
        temp_dir.mkdir(exist_ok=True)
        temp_path = temp_dir / f'config_{self.iteration}.yaml'
        
        with open(temp_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        return str(temp_path)
    
    def compute_trajectory_error(self, slam_file: str, mocap_file: str) -> float:
        """
        Compute ATE (Absolute Trajectory Error) between SLAM and mocap trajectories.
        Returns ATE in meters (lower is better).
        """
        try:
            slam_data = np.loadtxt(slam_file, skiprows=1)
            mocap_data = np.loadtxt(mocap_file, skiprows=1)
        except Exception as e:
            print(f"  ✗ Error reading trajectory files: {e}")
            return float('inf')
        
        if slam_data.size == 0 or mocap_data.size == 0:
            print("  ✗ Empty trajectory files")
            return float('inf')
        
        # Extract positions (columns 1-3)
        slam_pos = slam_data[:, 1:4] if len(slam_data.shape) > 1 else slam_data[1:4].reshape(1, -1)
        mocap_pos = mocap_data[:, 1:4] if len(mocap_data.shape) > 1 else mocap_data[1:4].reshape(1, -1)
        
        # Align trajectories by timestamps (column 0)
        slam_ts = slam_data[:, 0] if len(slam_data.shape) > 1 else slam_data[0:1]
        mocap_ts = mocap_data[:, 0] if len(mocap_data.shape) > 1 else mocap_data[0:1]
        
        min_ts = max(slam_ts.min(), mocap_ts.min())
        max_ts = min(slam_ts.max(), mocap_ts.max())
        
        # Interpolate to common timestamps
        slam_interp = np.interp(mocap_ts[(mocap_ts >= min_ts) & (mocap_ts <= max_ts)], 
                                slam_ts, slam_pos[:, 0], left=np.nan, right=np.nan)
        mocap_aligned = mocap_pos[(mocap_ts >= min_ts) & (mocap_ts <= max_ts), :]
        
        if len(slam_interp) < 10:
            print(f"  ✗ Not enough overlapping trajectory points: {len(slam_interp)}")
            return float('inf')
        
        # Compute ATE (RMS of position differences)
        try:
            errors = np.linalg.norm(slam_pos[:len(slam_interp)] - mocap_aligned, axis=1)
            ate = np.sqrt(np.mean(errors ** 2))
            return float(ate)
        except Exception as e:
            print(f"  ✗ Error computing ATE: {e}")
            return float('inf')
    
    def run_optimization_iteration(self, params: np.ndarray) -> float:
        """
        Run a single optimization iteration with given parameters.
        Returns ATE error value (lower is better).
        """
        self.iteration += 1
        param_dict = {name: float(val) for name, val in zip(self.param_names, params)}
        
        print(f"\n[Iteration {self.iteration}] Testing parameters:")
        for name, val in sorted(param_dict.items()):
            print(f"  {name}: {val:.6f}")
        
        # Create temp config
        temp_config = self.create_temp_config(param_dict)
        
        # Cleanup previous run (kill rviz and all processes)
        self.cleanup_processes()
        time.sleep(1)
        
        # Clean trajectory log files
        slam_log = Path(self.workspace) / 'slam_trajectory.txt'
        mocap_log = Path(self.workspace) / 'mocap_trajectory.txt'
        slam_log.unlink(missing_ok=True)
        mocap_log.unlink(missing_ok=True)
        
        try:
            # Start trajectory listener node first
            print("  Starting trajectory listener...")
            listener_proc = subprocess.Popen(
                f'source /opt/ros/humble/setup.bash && '
                f'source {self.workspace}/install/setup.bash && '
                f'python3 {Path(__file__).parent}/trajectory_listener.py',
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(listener_proc)
            time.sleep(2)
            
            # Start rosbag playback
            print("  Starting rosbag playback...")
            bag_proc = subprocess.Popen(
                f'source /opt/ros/humble/setup.bash && '
                f'source {self.workspace}/install/setup.bash && '
                f'ros2 bag play "{self.rosbag_path}" --loop',
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(bag_proc)
            time.sleep(2)
            
            # Start point_lio with temp config (launches rviz)
            print("  Starting Point-LIO...")
            lio_proc = subprocess.Popen(
                f'source /opt/ros/humble/setup.bash && '
                f'source {self.workspace}/install/setup.bash && '
                f'timeout 120 ros2 launch point_lio mapping_utlidar.launch '
                f'config_file:={temp_config}',
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(lio_proc)
            time.sleep(3)
            
            # Start mocap odometry
            print("  Starting mocap odometry...")
            mocap_proc = subprocess.Popen(
                f'source /opt/ros/humble/setup.bash && '
                f'source {self.workspace}/install/setup.bash && '
                f'timeout 120 ros2 run mocap_odometry trajectory_node',
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(mocap_proc)
            
            # Wait for trajectories to be logged
            print("  Waiting for trajectory data...")
            start_wait = time.time()
            timeout = 120  # 2 minutes
            trajectory_ready = False
            
            while time.time() - start_wait < timeout:
                if slam_log.exists() and mocap_log.exists():
                    slam_size = slam_log.stat().st_size
                    mocap_size = mocap_log.stat().st_size
                    # Need at least 2 lines of data (header + 1 entry)
                    if slam_size > 100 and mocap_size > 100:
                        print(f"  ✓ Trajectories recorded ({slam_size} + {mocap_size} bytes)")
                        trajectory_ready = True
                        break
                time.sleep(1)
            
            if not trajectory_ready:
                print(f"  ✗ Timeout waiting for trajectory files")
                # Debug: check what topics are being published
                subprocess.run('ros2 topic list', shell=True, executable='/bin/bash')
                return float('inf')
            
            # Give extra time for final writes
            time.sleep(3)
            
            # Compute error
            if slam_log.exists() and mocap_log.exists():
                ate = self.compute_trajectory_error(str(slam_log), str(mocap_log))
                print(f"  ✓ ATE (Absolute Trajectory Error): {ate:.6f} m")
                return ate
            else:
                print(f"  ✗ Trajectory files not created")
                return float('inf')
                
        except Exception as e:
            print(f"  ✗ Iteration error: {e}")
            import traceback
            traceback.print_exc()
            return float('inf')
        finally:
            self.cleanup_processes()
            time.sleep(1)
    
    def optimize(self, method: str = 'differential_evolution', max_eval: int = 50):
        """
        Optimize parameters using specified optimization algorithm.
        
        Args:
            method: 'differential_evolution' (global, recommended) or 'nelder-mead' (local)
            max_eval: Maximum number of function evaluations
        """
        bounds = [self.param_bounds[name] for name in self.param_names]
        defaults = np.array([self.param_defaults[name] for name in self.param_names])
        
        print(f"\n{'='*60}")
        print(f"Starting Parameter Optimization ({method})")
        print(f"Parameters to optimize: {len(self.param_names)}")
        print(f"Max evaluations: {max_eval}")
        print(f"{'='*60}")
        
        best_result = None
        best_ate = float('inf')
        
        def objective(params):
            nonlocal best_result, best_ate
            ate = self.run_optimization_iteration(params)
            if ate < best_ate:
                best_ate = ate
                best_result = params.copy()
            return ate
        
        try:
            if method == 'differential_evolution':
                result = differential_evolution(
                    objective,
                    bounds,
                    seed=42,
                    maxiter=max_eval // 10,
                    workers=1,
                    updating='deferred',
                    atol=0.01,
                    tol=0.01
                )
            else:  # nelder-mead
                result = minimize(
                    objective,
                    defaults,
                    method='Nelder-Mead',
                    options={'maxiter': max_eval, 'xatol': 0.01, 'fatol': 0.01}
                )
            
            print(f"\n{'='*60}")
            print(f"Optimization Complete!")
            print(f"Best ATE: {best_ate:.6f} m")
            print(f"Best Parameters:")
            for name, val in zip(self.param_names, best_result):
                print(f"  {name}: {val:.6f}")
            print(f"{'='*60}")
            
            return best_result, best_ate
            
        except KeyboardInterrupt:
            print("\n\nOptimization interrupted by user.")
            if best_result is not None:
                print(f"Best result so far - ATE: {best_ate:.6f} m")
            return best_result, best_ate


def main():
    if len(sys.argv) < 2:
        print("Usage: parameter_optimizer.py <rosbag_path> [config_path]")
        print("Example: parameter_optimizer.py ~/rosbags/go2_data.db3")
        sys.exit(1)
    
    rosbag_path = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 else '/root/ros2_ws/src/point_lio/config/utlidar.yaml'
    
    if not os.path.exists(rosbag_path):
        print(f"✗ Rosbag not found: {rosbag_path}")
        sys.exit(1)
    
    if not os.path.exists(config_path):
        print(f"✗ Config not found: {config_path}")
        sys.exit(1)
    
    optimizer = ParameterOptimizer(rosbag_path, config_path)
    
    # Run optimization with Differential Evolution (global optimizer)
    best_params, best_ate = optimizer.optimize(method='differential_evolution', max_eval=50)
    
    if best_params is not None:
        # Save best parameters to YAML
        param_dict = {name: float(val) for name, val in zip(optimizer.param_names, best_params)}
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        for param_name, value in param_dict.items():
            config['/**']['ros__parameters']['mapping'][param_name] = value
        
        backup_path = config_path + '.backup'
        shutil.copy(config_path, backup_path)
        print(f"  Backup saved to: {backup_path}")
        
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        print(f"  ✓ Best parameters saved to: {config_path}")


if __name__ == '__main__':
    main()
