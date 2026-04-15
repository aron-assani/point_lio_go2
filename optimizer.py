#!/usr/bin/env python3
import os
import sys
import yaml
import subprocess
import time
import numpy as np
from pathlib import Path
from threading import Thread, Event
import rclpy
from nav_msgs.msg import Path as PathMsg
import optuna
from optuna.pruners import MedianPruner

# Configuration
ROSBAG_PATH = os.path.expanduser("~/ros2_ws/tester_rosbag/rosbag2_1")
CONFIG_PATH = os.path.expanduser("~/ros2_ws/install/point_lio/share/point_lio/config/utlidar.yaml")
ROS_DOMAIN_ID = 42
MAX_DRIFT = 2.0  # meters, early stop threshold
ATE_ERROR_VALUE = 100.0  # penalty for failed runs

# Parameter search space
PARAM_SPACE = {
    "lidar_meas_cov": (0.001, 0.1),
    "imu_meas_acc_cov": (1.0, 50.0),
    "imu_meas_omg_cov": (0.5, 10.0),
    "plane_thr": (0.01, 0.5),
    "det_range": (50.0, 150.0),
}

class TrajectoryMonitor:
    def __init__(self):
        self.body_poses = []
        self.mocap_poses = []
        self.ready = Event()
        self.stop = Event()

    def body_callback(self, msg: PathMsg):
        if msg.poses:
            self.body_poses.append(msg.poses[-1].pose.position)

    def mocap_callback(self, msg: PathMsg):
        if msg.poses:
            self.mocap_poses.append(msg.poses[-1].pose.position)
            if len(self.mocap_poses) >= 10 and len(self.body_poses) >= 10:
                self.ready.set()

    def calculate_ate(self):
        if len(self.body_poses) < 2 or len(self.mocap_poses) < 2:
            return ATE_ERROR_VALUE
        
        min_len = min(len(self.body_poses), len(self.mocap_poses))
        body_arr = np.array([[p.x, p.y, p.z] for p in self.body_poses[:min_len]])
        mocap_arr = np.array([[p.x, p.y, p.z] for p in self.mocap_poses[:min_len]])
        
        return np.sqrt(np.mean(np.sum((body_arr - mocap_arr) ** 2, axis=1)))

    def check_drift(self):
        if len(self.body_poses) > 0 and len(self.mocap_poses) > 0:
            body_pos = np.array([self.body_poses[-1].x, self.body_poses[-1].y, self.body_poses[-1].z])
            mocap_pos = np.array([self.mocap_poses[-1].x, self.mocap_poses[-1].y, self.mocap_poses[-1].z])
            drift = np.linalg.norm(body_pos - mocap_pos)
            return drift > MAX_DRIFT
        return False

def update_config(params):
    """Update utlidar.yaml with new parameters"""
    with open(CONFIG_PATH, 'r') as f:
        config = yaml.safe_load(f)
    
    for key, value in params.items():
        config['/**']['ros__parameters']['mapping'][key] = value
    
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(config, f)

def run_trial(params, trial_num):
    """Run single optimization trial"""
    print(f"\n=== Trial {trial_num} ===")
    print(f"Parameters: {params}")
    
    update_config(params)
    
    # Initialize ROS and monitor
    rclpy.init(allow_reused_context=True)
    node = rclpy.create_node('optimizer_node')
    monitor = TrajectoryMonitor()
    
    node.create_subscription(PathMsg, '/body_path', monitor.body_callback, 10)
    node.create_subscription(PathMsg, '/mocap_path', monitor.mocap_callback, 10)
    
    processes = []
    try:
        # Kill previous processes
        os.system("pkill -f pointlio_mapping || true")
        os.system("pkill -f rviz2 || true")
        os.system("pkill -f trajectory_evaluator || true")
        time.sleep(1)
        
        # Start nodes
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(ROS_DOMAIN_ID)
        
        # Launch trajectory node
        p_traj = subprocess.Popen(
            ["bash", "-l", "-c", "ros2 run mocap_odometry trajectory_node"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        processes.append(p_traj)
        time.sleep(1)
        
        # Launch Point LIO with RViz
        p_lio = subprocess.Popen(
            ["bash", "-l", "-c", "ros2 launch point_lio mapping_utlidar.launch rviz:=true"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        processes.append(p_lio)
        time.sleep(3)
        
        # Wait for trajectories to initialize
        monitor.ready.wait(timeout=10)
        
        if not monitor.ready.is_set():
            print(f"Trial {trial_num}: Timeout waiting for trajectory data")
            return ATE_ERROR_VALUE
        
        # Play rosbag
        p_bag = subprocess.Popen(
            ["bash", "-l", "-c", f"ros2 bag play {ROSBAG_PATH} --clock"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        processes.append(p_bag)
        
        # Monitor for early stopping
        start_time = time.time()
        max_duration = 120  # seconds
        
        while True:
            # Spin to receive messages
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Check early stopping condition
            if monitor.check_drift():
                print(f"Trial {trial_num}: Early stopping - drift exceeded {MAX_DRIFT}m")
                break
            
            # Check timeout
            if time.time() - start_time > max_duration:
                print(f"Trial {trial_num}: Timeout reached")
                break
            
            time.sleep(0.1)
        
        # Calculate ATE
        ate = monitor.calculate_ate()
        print(f"Trial {trial_num}: ATE = {ate:.4f}")
        return ate
        
    except Exception as e:
        print(f"Trial {trial_num}: Error - {e}")
        return ATE_ERROR_VALUE
    
    finally:
        # Cleanup
        for p in processes:
            try:
                p.terminate()
                p.wait(timeout=5)
            except:
                p.kill()
        
        os.system("pkill -f rviz2 || true")
        time.sleep(1)
        
        try:
            rclpy.shutdown()
        except:
            pass

def objective(trial):
    """Optuna objective function"""
    params = {
        key: trial.suggest_float(key, bounds[0], bounds[1])
        for key, bounds in PARAM_SPACE.items()
    }
    
    ate = run_trial(params, trial.number)
    
    # Report intermediate value for pruning
    trial.report(ate, step=1)
    
    return ate

if __name__ == "__main__":
    study = optuna.create_study(
        direction="minimize",
        pruner=MedianPruner(n_startup_trials=5, n_warmup_steps=0)
    )
    
    study.optimize(objective, n_trials=20, n_jobs=1)
    
    print("\n=== Optimization Complete ===")
    print(f"Best ATE: {study.best_value:.4f}")
    print(f"Best Parameters: {study.best_params}")
