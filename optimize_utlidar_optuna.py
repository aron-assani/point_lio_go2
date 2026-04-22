#!/usr/bin/env python3
"""
SLAM Trajectory Optimizer for UTLIDAR Configuration (Optuna version)
Starts the mapping_utlidar_optimize.launch and monitors trajectory topics.
"""

import subprocess
import time
import threading
import math
import os
import signal
import json
import re
from pathlib import Path as FSPath

import optuna
import rclpy
from nav_msgs.msg import Path as PathMsg
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


UTLIDAR_INSTALL_CONFIG_PATH = FSPath('/root/ros2_ws/install/point_lio/share/point_lio/config/utlidar.yaml')
UTLIDAR_SOURCE_CONFIG_PATH = FSPath(__file__).resolve().parent / 'point_lio' / 'config' / 'utlidar.yaml'
UTLIDAR_CONFIG_PATH = UTLIDAR_INSTALL_CONFIG_PATH if UTLIDAR_INSTALL_CONFIG_PATH.exists() else UTLIDAR_SOURCE_CONFIG_PATH

PARAMETER_BOUNDS = {
    'lidar_meas_cov': (0.001, 0.05),
    'acc_cov_output': (50.0, 2000.0),
    'gyr_cov_output': (100.0, 3000.0),
    'b_acc_cov': (0.0001, 0.1),
    'b_gyr_cov': (0.0001, 0.1),
    'imu_meas_acc_cov': (0.5, 30.0),
    'imu_meas_omg_cov': (0.5, 30.0),
    'gyr_cov_input': (0.01, 5.0),
    'acc_cov_input': (0.1, 20.0),
}

INITIAL_PARAMETER_VALUES = {
    'lidar_meas_cov': 0.01,
    'acc_cov_output': 500.0,
    'gyr_cov_output': 1000.0,
    'b_acc_cov': 0.01,
    'b_gyr_cov': 0.01,
    'imu_meas_acc_cov': 10.0,
    'imu_meas_omg_cov': 5.0,
    'gyr_cov_input': 0.1,
    'acc_cov_input': 2.0,
}

EARLY_EXIT_PENALTY_COST = 1e9
OPTUNA_STORAGE_PATH = FSPath('optuna_utlidar_study.db')

# Multi-term objective: accuracy + smoothness + stability.
OBJECTIVE_MODE = 'accuracy_smoothness_stability'
OBJECTIVE_TAIL_FRACTION = 0.25
OBJECTIVE_BOUNDS = {
    'time_in_bound_threshold_m': 0.25,
    'slope_deadband': 0.0,
}
OBJECTIVE_WEIGHTS = {
    'accuracy_mean': 0.45,
    'accuracy_p95': 0.25,
    'jitter_accel': 0.20,
    'stability_positive_slope': 0.05,
    'stability_out_of_bound_ratio': 0.05,
}
OPTUNA_STUDY_NAME = f'utlidar_optimization_{OBJECTIVE_MODE}_v4'


class TrajectoryMonitor(Node):
    def __init__(self, on_failure=None, drift_threshold_m=1.0, max_consecutive_violations=3):
        super().__init__('utlidar_optimizer')
        self.on_failure, self.drift_threshold_m, self.max_consecutive_violations = on_failure, drift_threshold_m, max_consecutive_violations
        self.mocap_path_count = self.body_path_count = 0
        self.last_mocap_msg_time = self.last_body_msg_time = None
        self.latest_mocap_position = self.latest_body_position = self.latest_path_drift = None
        self.last_mocap_wall_time = self.last_body_wall_time = time.monotonic()
        self.consecutive_drift_violations = 0
        self.termination_requested = False
        self.drift_samples = []

        for topic, msg_type, cb, depth in (
            ('/mocap_path', PathMsg, self.mocap_path_callback, 10),
            ('/body_path', PathMsg, self.body_path_callback, 10),
        ):
            self.create_subscription(msg_type, topic, cb, depth)

        self.get_logger().info('Trajectory Monitor initialized. Waiting for messages...')
        self.stats_timer = self.create_timer(2.0, self.print_stats)
        self.drift_timer = self.create_timer(0.2, self.check_path_drift)

    @staticmethod
    def _stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _handle_failure(self, reason: str):
        if self.termination_requested:
            return
        self.termination_requested = True
        self.get_logger().error(reason)
        if self.on_failure is not None:
            self.on_failure(reason)

    def mocap_path_callback(self, msg: PathMsg):
        self.mocap_path_count += 1
        self.last_mocap_msg_time = self.get_clock().now()
        self.last_mocap_wall_time = time.monotonic()
        if msg.poses:
            p = msg.poses[-1].pose.position
            self.latest_mocap_position = (p.x, p.y, p.z)
        self.get_logger().debug(f'Mocap path message #{self.mocap_path_count}: {len(msg.poses)} poses')

    def body_path_callback(self, msg: PathMsg):
        self.body_path_count += 1
        self.last_body_msg_time = self.get_clock().now()
        self.last_body_wall_time = time.monotonic()
        if msg.poses:
            p = msg.poses[-1].pose.position
            self.latest_body_position = (p.x, p.y, p.z)
        self.get_logger().debug(f'Body path message #{self.body_path_count}: {len(msg.poses)} poses')

    def check_path_drift(self):
        if self.termination_requested or self.latest_mocap_position is None or self.latest_body_position is None:
            return
        self.latest_path_drift = math.dist(self.latest_mocap_position, self.latest_body_position)
        self.drift_samples.append(self.latest_path_drift)
        self.consecutive_drift_violations = self.consecutive_drift_violations + 1 if self.latest_path_drift > self.drift_threshold_m else 0
        if self.consecutive_drift_violations >= self.max_consecutive_violations:
            self._handle_failure(
                f'Path drift exceeded threshold: {self.latest_path_drift:.3f} m > '
                f'{self.drift_threshold_m:.3f} m for {self.consecutive_drift_violations} checks.'
            )

    def print_stats(self):
        drift_text = 'N/A' if self.latest_path_drift is None else f'{self.latest_path_drift:.3f} m'
        self.get_logger().info(
            f'Mocap: {self.mocap_path_count} | Body: {self.body_path_count} | Drift: {drift_text}'
        )


def kill_rviz_processes():
    subprocess.run(['pkill', '-f', 'rviz2'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_all_playbacks():
    patterns = ('ros2 bag play', 'rosbag2_transport', 'rosbag2_cpp', 'rosbag2_player', '/rosbag2_')
    for sig in ('', '-9'):
        for pattern in patterns:
            subprocess.run(['pkill', sig, '-f', pattern] if sig else ['pkill', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.2)


def cleanup_stale_processes():
    kill_rviz_processes()
    kill_all_playbacks()
    for name in ('pointlio_mapping', 'transform_everything', 'trajectory_node', 'mapping_utlidar_optimize.launch'):
        for sig in ('', '-9'):
            subprocess.run(['pkill', sig, '-f', name] if sig else ['pkill', '-f', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_launch_process(process, timeout_sec=5):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def run_launch_file():
    cmd = ['ros2', 'launch', 'point_lio', 'mapping_utlidar_optimize.launch']
    print(f"Starting launch file: {' '.join(cmd)}")
    process = subprocess.Popen(cmd, start_new_session=True)
    return process


def monitor_launch_output(process, on_exit):
    while process.poll() is None:
        time.sleep(0.1)
    on_exit('rosbag playback ended' if process.returncode == 0 else f'Launch process exited with code {process.returncode}')


def find_rosbag_pid(launch_process, timeout_sec=30.0, poll_sec=0.2):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and launch_process.poll() is None:
        try:
            pid = subprocess.check_output(['pgrep', '-n', '-f', 'ros2 bag play'], text=True).strip()
            if pid:
                return int(pid)
        except subprocess.CalledProcessError:
            pass
        time.sleep(poll_sec)
    return None


def monitor_rosbag_pid(launch_process, on_exit):
    pid = find_rosbag_pid(launch_process)
    if pid is None:
        if launch_process.poll() is not None:
            on_exit(f'Launch process exited with code {launch_process.returncode}')
        return
    while launch_process.poll() is None and os.path.exists(f'/proc/{pid}'):
        time.sleep(0.2)
    if launch_process.poll() is None:
        on_exit('rosbag playback ended')


def write_json_log(path, buffer):
    path.write_text(json.dumps(buffer, indent=2), encoding='utf-8')


def clamp_parameter_value(key, value):
    lo, hi = PARAMETER_BOUNDS[key]
    return float(min(max(value, lo), hi))


def to_yaml_scalar(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, str):
        return value
    return json.dumps(value)


def set_yaml_parameter(file_path, key, value):
    lines = file_path.read_text(encoding='utf-8').splitlines(keepends=True)
    target_index = None
    key_re = re.compile(rf'^(\s*)({re.escape(key)})\s*:\s*(.*?)\s*(#.*)?$')

    for i, raw in enumerate(lines):
        line = raw.rstrip('\n')
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        m = key_re.match(line)
        if m and m.group(3).strip() != '':
            target_index = i
            break

    if target_index is None:
        raise ValueError(f'Could not find scalar parameter key: {key}')

    old_line = lines[target_index].rstrip('\n')
    comment_split = old_line.split('#', 1)
    before_comment = comment_split[0]
    comment = '' if len(comment_split) == 1 else '#' + comment_split[1]
    prefix, _ = before_comment.split(':', 1)
    new_line = f"{prefix}: {to_yaml_scalar(value)}"
    if comment:
        new_line = f"{new_line} {comment.strip()}"
    lines[target_index] = new_line + '\n'
    file_path.write_text(''.join(lines), encoding='utf-8')


def compute_smoothness_jitter(drift_series):
    if len(drift_series) < 3:
        return 0.0
    accel = [
        drift_series[i] - 2.0 * drift_series[i - 1] + drift_series[i - 2]
        for i in range(2, len(drift_series))
    ]
    accel_energy = sum(a * a for a in accel) / len(accel)
    return math.sqrt(accel_energy)


def compute_positive_slope(drift_series):
    if len(drift_series) < 2:
        return 0.0

    n = len(drift_series)
    x_mean = 0.5 * (n - 1)
    y_mean = sum(drift_series) / n
    num = sum((i - x_mean) * (y - y_mean) for i, y in enumerate(drift_series))
    den = sum((i - x_mean) ** 2 for i in range(n))
    slope = num / den if den > 0.0 else 0.0
    return max(0.0, slope - OBJECTIVE_BOUNDS['slope_deadband'])


def summarize_iteration(iteration_index, reason, monitor, parameter_values, proposal_mode):
    samples = monitor.drift_samples
    natural_completion = reason == 'rosbag playback ended'
    if natural_completion and samples:
        sample_count = len(samples)
        cost_sum = float(sum(samples))
        cost_mean = cost_sum / sample_count
        sorted_samples = sorted(samples)
        p95_index = max(0, min(sample_count - 1, int(0.95 * (sample_count - 1))))
        p95_drift = float(sorted_samples[p95_index])
        max_drift = float(sorted_samples[-1])

        tail_start_index = max(0, int(sample_count * (1.0 - OBJECTIVE_TAIL_FRACTION)))
        objective_samples = samples[tail_start_index:]
        objective_sample_count = len(objective_samples)
        objective_sum = float(sum(objective_samples))
        objective_mean = objective_sum / objective_sample_count
        objective_sorted_samples = sorted(objective_samples)
        objective_p95_index = max(0, min(objective_sample_count - 1, int(0.95 * (objective_sample_count - 1))))
        objective_p95_drift = float(objective_sorted_samples[objective_p95_index])

        jitter_accel = compute_smoothness_jitter(objective_samples)
        positive_slope = compute_positive_slope(objective_samples)
        threshold = OBJECTIVE_BOUNDS['time_in_bound_threshold_m']
        in_bound_count = sum(1 for d in objective_samples if d <= threshold)
        time_in_bound_ratio = in_bound_count / objective_sample_count
        out_of_bound_ratio = 1.0 - time_in_bound_ratio

        objective_components = {
            'accuracy_mean': objective_mean,
            'accuracy_p95': objective_p95_drift,
            'jitter_accel': jitter_accel,
            'stability_positive_slope': positive_slope,
            'stability_out_of_bound_ratio': out_of_bound_ratio,
        }
        objective_cost = sum(
            OBJECTIVE_WEIGHTS[name] * objective_components[name]
            for name in OBJECTIVE_WEIGHTS
        )
    else:
        sample_count = len(samples)
        cost_sum = float(sum(samples)) if samples else None
        cost_mean = None
        p95_drift = None
        max_drift = None
        objective_sample_count = None
        objective_sum = None
        objective_mean = None
        objective_p95_drift = None
        jitter_accel = None
        positive_slope = None
        threshold = OBJECTIVE_BOUNDS['time_in_bound_threshold_m']
        time_in_bound_ratio = None
        out_of_bound_ratio = None
        objective_components = None
        objective_cost = EARLY_EXIT_PENALTY_COST

    return {
        'iteration': iteration_index,
        'reason': reason,
        'natural_completion': natural_completion,
        'parameter_values': parameter_values,
        'proposal_mode': proposal_mode,
        'samples': samples,
        'sample_count': sample_count,
        'cost': objective_cost,
        'cost_mode': OBJECTIVE_MODE,
        'cost_sum': cost_sum,
        'cost_mean': cost_mean,
        'p95_drift': p95_drift,
        'max_drift': max_drift,
        'objective_sample_count': objective_sample_count,
        'objective_sum': objective_sum,
        'objective_mean': objective_mean,
        'objective_p95_drift': objective_p95_drift,
        'objective_jitter_accel': jitter_accel,
        'objective_positive_slope': positive_slope,
        'objective_time_in_bound_ratio': time_in_bound_ratio,
        'objective_out_of_bound_ratio': out_of_bound_ratio,
        'objective_time_in_bound_threshold_m': threshold,
        'objective_weights': OBJECTIVE_WEIGHTS,
        'objective_components': objective_components,
        'final_drift': monitor.latest_path_drift,
    }


def create_optuna_study():
    storage_url = f"sqlite:///{OPTUNA_STORAGE_PATH}"
    sampler = optuna.samplers.TPESampler(multivariate=True, seed=42)
    return optuna.create_study(
        direction='minimize',
        sampler=sampler,
        study_name=OPTUNA_STUDY_NAME,
        storage=storage_url,
        load_if_exists=True,
    )


def ensure_initial_trial_enqueued(study):
    if len(study.trials) == 0:
        study.enqueue_trial(INITIAL_PARAMETER_VALUES)


def propose_next_parameters_optuna(study):
    trial = study.ask()
    params = {}
    for key, (lo, hi) in PARAMETER_BOUNDS.items():
        use_log = lo > 0.0 and (hi / lo) >= 50.0
        params[key] = clamp_parameter_value(key, trial.suggest_float(key, lo, hi, log=use_log))
    mode = 'optuna_enqueued' if trial.number == 0 else 'optuna_tpe'
    return trial, params, mode


def main():
    print('=' * 60)
    print('UTLIDAR Configuration Optimizer (Optuna)')
    print('=' * 60)

    drift_buffer = []
    json_log_path = FSPath('drift_iterations_optuna.json')
    iteration = 0

    study = create_optuna_study()
    ensure_initial_trial_enqueued(study)

    try:
        while True:
            iteration += 1
            trial, current_parameters, current_mode = propose_next_parameters_optuna(study)
            print(f"\n{'=' * 60}\nStarting iteration {iteration} (trial {trial.number})\n{'=' * 60}")

            for key, value in current_parameters.items():
                set_yaml_parameter(UTLIDAR_CONFIG_PATH, key, value)
            print(f"Set {len(current_parameters)} parameters in {UTLIDAR_CONFIG_PATH} ({current_mode})")

            print('Cleaning up stale processes before launch...')
            cleanup_stale_processes()
            time.sleep(0.5)

            launch_process = run_launch_file()
            stop_event = threading.Event()
            result = {'reason': 'unknown'}

            def terminate_on_failure(reason: str):
                if stop_event.is_set():
                    return
                stop_event.set()
                result['reason'] = reason
                print(f"\n{'=' * 60}\nStopping optimization: {reason}\n{'=' * 60}")
                stop_launch_process(launch_process)
                cleanup_stale_processes()
                if rclpy.ok():
                    rclpy.shutdown()

            rclpy.init()
            monitor = TrajectoryMonitor(on_failure=terminate_on_failure, drift_threshold_m=1.0, max_consecutive_violations=3)
            executor = MultiThreadedExecutor()
            executor.add_node(monitor)
            monitor_thread = threading.Thread(target=monitor_launch_output, args=(launch_process, terminate_on_failure), daemon=True)
            rosbag_thread = threading.Thread(target=monitor_rosbag_pid, args=(launch_process, terminate_on_failure), daemon=True)
            monitor_thread.start()
            rosbag_thread.start()

            print('\nWaiting for ROS 2 system to initialize...')
            time.sleep(5)
            if launch_process.poll() is not None:
                terminate_on_failure(result['reason'])

            try:
                print(
                    f"\n{'=' * 60}\nMonitoring SLAM system (Ctrl+C to stop)...\n"
                    f"Watching for:\n  - Path messages from mocap and SLAM\n"
                    f"  - Drift between mocap and body paths\n{'=' * 60}\n"
                )
                executor.spin()
            except Exception as e:
                print(f'\n\nERROR: {e}')
                import traceback
                traceback.print_exc()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
                stop_launch_process(launch_process)
                cleanup_stale_processes()

            summary = summarize_iteration(iteration, result['reason'], monitor, dict(current_parameters), current_mode)
            drift_buffer.append(summary)
            write_json_log(json_log_path, drift_buffer)

            study.tell(trial, summary['cost'])
            best = study.best_trial if len(study.trials) else None

            max_drift = 'N/A' if summary['max_drift'] is None else f"{summary['max_drift']:.3f} m"
            print(
                f"Stored iteration {iteration}: reason={summary['reason']} | "
                f"trial={trial.number} | samples={summary['sample_count']} | "
                f"cost={summary['cost']:.4f} | max_drift={max_drift}"
            )
            print(f"Wrote JSON log: {json_log_path}")
            if best is not None:
                print(f"Best trial so far: #{best.number} cost={best.value:.4f}")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print('\n\nShutting down...')
    finally:
        cleanup_stale_processes()
        print('\nOptimizer stopped.')


if __name__ == '__main__':
    main()
