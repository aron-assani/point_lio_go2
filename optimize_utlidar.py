#!/usr/bin/env python3
"""
SLAM Trajectory Optimizer for UTLIDAR Configuration
Starts the mapping_utlidar_optimize.launch and monitors trajectory topics.
"""

import os
import sys
import subprocess
import time
import threading
import signal
from pathlib import Path
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray

import numpy as np


class TrajectoryMonitor(Node):
    """Monitor mocap and body trajectories from SLAM optimization."""
    
    def __init__(self):
        super().__init__('utlidar_optimizer')
        
        self.mocap_path_count = 0
        self.body_path_count = 0
        self.last_mocap_msg_time = None
        self.last_body_msg_time = None
        
        # Diagnostics
        self.cloud_sizes = deque(maxlen=10)  # Track last 10 clouds
        self.cloud_count = 0
        self.registered_cloud_count = 0
        self.crash_detected = False
        
        # Subscriptions
        self.mocap_path_sub = self.create_subscription(
            PathMsg,
            '/mocap_path',
            self.mocap_path_callback,
            10
        )
        
        self.body_path_sub = self.create_subscription(
            PathMsg,
            '/body_path',
            self.body_path_callback,
            10
        )
        
        # Monitor input and output clouds
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/utlidar/transformed_cloud',
            self.cloud_callback,
            1
        )
        
        self.registered_cloud_sub = self.create_subscription(
            PointCloud2,
            '/registered_scan',
            self.registered_cloud_callback,
            1
        )
        
        # Monitor diagnostics
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        
        self.get_logger().info("Trajectory Monitor initialized. Waiting for messages...")
        
        # Print stats periodically
        self.stats_timer = self.create_timer(2.0, self.print_stats)
        self.diagnostics_timer = self.create_timer(5.0, self.print_diagnostics)
    
    def cloud_callback(self, msg: PointCloud2):
        """Callback for input point cloud."""
        self.cloud_count += 1
        num_points = msg.width * msg.height
        self.cloud_sizes.append(num_points)
        self.get_logger().debug(f"Input cloud #{self.cloud_count}: {num_points} points")
    
    def registered_cloud_callback(self, msg: PointCloud2):
        """Callback for registered output cloud."""
        self.registered_cloud_count += 1
        num_points = msg.width * msg.height
        self.get_logger().debug(f"Registered cloud #{self.registered_cloud_count}: {num_points} points")
    
    def diagnostics_callback(self, msg: DiagnosticArray):
        """Monitor node diagnostics for crashes."""
        for status in msg.status:
            if 'laserMapping' in status.name and status.level > 1:  # WARN or ERROR
                self.get_logger().error(f"SLAM Node Issue: {status.name} - {status.message}")
                self.crash_detected = True
    
    def mocap_path_callback(self, msg: PathMsg):
        """Callback for mocap_path topic."""
        self.mocap_path_count += 1
        self.last_mocap_msg_time = self.get_clock().now()
        self.get_logger().debug(f"Mocap path message #{self.mocap_path_count}: {len(msg.poses)} poses")
    
    def body_path_callback(self, msg: PathMsg):
        """Callback for body_path topic."""
        self.body_path_count += 1
        self.last_body_msg_time = self.get_clock().now()
        self.get_logger().debug(f"Body path message #{self.body_path_count}: {len(msg.poses)} poses")
    
    def print_stats(self):
        """Print current statistics."""
        self.get_logger().info(
            f"Messages received - Mocap: {self.mocap_path_count} | Body: {self.body_path_count} | "
            f"Input Clouds: {self.cloud_count} | Registered Clouds: {self.registered_cloud_count}"
        )
    
    def print_diagnostics(self):
        """Print diagnostic information."""
        if not self.cloud_sizes:
            self.get_logger().warn("No point clouds received yet!")
            return
        
        avg_cloud_size = np.mean(list(self.cloud_sizes))
        max_cloud_size = np.max(list(self.cloud_sizes))
        min_cloud_size = np.min(list(self.cloud_sizes))
        
        self.get_logger().info(
            f"Cloud Statistics - Avg: {avg_cloud_size:.0f} | Max: {max_cloud_size} | Min: {min_cloud_size} points"
        )
        
        if self.crash_detected:
            self.get_logger().error("CRASH DETECTED! Check SLAM node diagnostics.")


def run_launch_file():
    """Start the mapping_utlidar_optimize.launch file."""
    # Use ros2 launch which automatically finds the package in install/
    cmd = [
        'ros2', 'launch',
        'point_lio',
        'mapping_utlidar_optimize.launch'
    ]
    
    print(f"Starting launch file: {' '.join(cmd)}")
    # DON'T pipe output - let it stream to console for immediate error visibility
    process = subprocess.Popen(cmd)
    return process


def monitor_launch_output(process):
    """Monitor launch file process for crashes."""
    while process.poll() is None:
        time.sleep(0.1)
    
    exit_code = process.returncode
    if exit_code != 0:
        print(f"\n{'='*60}")
        print(f"ERROR: Launch process exited with code {exit_code}")
        print(f"{'='*60}")


def main():
    """Main entry point for the optimizer."""
    print("=" * 60)
    print("UTLIDAR Configuration Optimizer - Diagnostic Mode")
    print("=" * 60)
    
    # Start the launch file
    launch_process = run_launch_file()
    
    # Monitor launch output in background thread
    monitor_thread = threading.Thread(target=monitor_launch_output, args=(launch_process,), daemon=True)
    monitor_thread.start()
    
    # Give the system time to initialize
    print("\nWaiting for ROS 2 system to initialize...")
    time.sleep(5)
    
    # Initialize ROS 2
    rclpy.init()
    
    try:
        # Create and spin the monitor node
        monitor = TrajectoryMonitor()
        executor = MultiThreadedExecutor()
        executor.add_node(monitor)
        
        print("\n" + "=" * 60)
        print("Monitoring SLAM system (Ctrl+C to stop)...")
        print("Watching for:")
        print("  - Point cloud sizes and frequencies")
        print("  - Path messages from mocap and SLAM")
        print("  - Node crashes and diagnostics")
        print("=" * 60 + "\n")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        rclpy.shutdown()
        launch_process.terminate()
        try:
            launch_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            launch_process.kill()
            launch_process.wait()
        
        print("\nOptimizer stopped.")


if __name__ == '__main__':
    main()
