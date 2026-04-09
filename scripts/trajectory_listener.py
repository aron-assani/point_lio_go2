#!/usr/bin/env python3
"""
Trajectory Listener - Captures Point-LIO and MoCap trajectories from ROS topics.
Saves trajectories to text files in format: timestamp x y z qx qy qz qw
Automatically discovers and subscribes to relevant odometry topics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pathlib import Path
import numpy as np
from collections import deque


class TrajectoryListener(Node):
    def __init__(self, output_dir: str = '/root/ros2_ws'):
        super().__init__('trajectory_listener')
        
        self.output_dir = Path(output_dir)
        self.slam_file = self.output_dir / 'slam_trajectory.txt'
        self.mocap_file = self.output_dir / 'mocap_trajectory.txt'
        
        # Clear previous files
        self.slam_file.unlink(missing_ok=True)
        self.mocap_file.unlink(missing_ok=True)
        
        # Write headers
        header = "timestamp x y z qx qy qz qw\n"
        self.slam_file.write_text(header)
        self.mocap_file.write_text(header)
        
        # Buffers to avoid duplicate writes
        self.slam_buffer = deque(maxlen=100)
        self.mocap_buffer = deque(maxlen=100)
        self.min_time_diff = 0.01  # 10ms between writes
        
        # Topic subscription tracking
        self.slam_subs = []
        self.mocap_subs = []
        
        # Subscribe to common SLAM topics (Point-LIO)
        slam_topic_names = ['/Odometry', '/odometry', '/utlidar/odometry']
        for topic in slam_topic_names:
            try:
                sub = self.create_subscription(
                    Odometry,
                    topic,
                    self.slam_callback,
                    10
                )
                self.slam_subs.append(sub)
                self.get_logger().info(f"✓ Subscribed to SLAM: {topic}")
            except:
                pass
        
        # Subscribe to common MoCap topics
        mocap_topic_names = ['/mocap_path', '/mocap_odometry', '/vicon/odometry', '/ground_truth/odometry', '/mocap']
        for topic in mocap_topic_names:
            try:
                sub = self.create_subscription(
                    Odometry,
                    topic,
                    self.mocap_callback,
                    10
                )
                self.mocap_subs.append(sub)
                self.get_logger().info(f"✓ Subscribed to MoCap: {topic}")
            except:
                pass
        
        if not self.slam_subs:
            self.get_logger().warn("Could not subscribe to any SLAM topics")
        if not self.mocap_subs:
            self.get_logger().warn("Could not subscribe to any MoCap topics")
        
        self.get_logger().info(f"Trajectory listener ready. Saving to {self.output_dir}")
    
    def slam_callback(self, msg: Odometry):
        """Save Point-LIO trajectory."""
        try:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            
            # Check for duplicates
            if len(self.slam_buffer) > 0 and abs(ts - self.slam_buffer[-1]) < self.min_time_diff:
                return
            
            self.slam_buffer.append(ts)
            
            line = f"{ts:.6f} {pos.x:.8f} {pos.y:.8f} {pos.z:.8f} " \
                   f"{quat.x:.8f} {quat.y:.8f} {quat.z:.8f} {quat.w:.8f}\n"
            
            with open(self.slam_file, 'a') as f:
                f.write(line)
            
        except Exception as e:
            self.get_logger().error(f"SLAM callback error: {e}")
    
    def mocap_callback(self, msg: Odometry):
        """Save MoCap trajectory."""
        try:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            
            # Check for duplicates
            if len(self.mocap_buffer) > 0 and abs(ts - self.mocap_buffer[-1]) < self.min_time_diff:
                return
            
            self.mocap_buffer.append(ts)
            
            line = f"{ts:.6f} {pos.x:.8f} {pos.y:.8f} {pos.z:.8f} " \
                   f"{quat.x:.8f} {quat.y:.8f} {quat.z:.8f} {quat.w:.8f}\n"
            
            with open(self.mocap_file, 'a') as f:
                f.write(line)
            
        except Exception as e:
            self.get_logger().error(f"MoCap callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Get output directory from environment or use default
    import os
    output_dir = os.environ.get('ROS_WS', '/root/ros2_ws')
    
    listener = TrajectoryListener(output_dir)
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
