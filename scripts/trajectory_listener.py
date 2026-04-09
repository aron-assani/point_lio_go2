#!/usr/bin/env python3
"""
Trajectory Listener - Captures Point-LIO and MoCap trajectories from ROS topics.
Subscribes directly to /Odometry (Point-LIO sensor frame) and applies the same 
sensor-to-body-center transformation as trajectory_node.py.
Also subscribes to /mocap_path (MoCap ground truth).
Saves to text files in format: timestamp x y z qx qy qz qw
Works with rosbag playback without needing OptiTrack hardware connection.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from pathlib import Path as FilePath
import numpy as np


class TrajectoryListener(Node):
    def __init__(self, output_dir: str = '/root/ros2_ws'):
        super().__init__('trajectory_listener')
        
        self.output_dir = FilePath(output_dir)
        self.slam_file = self.output_dir / 'slam_trajectory.txt'
        self.mocap_file = self.output_dir / 'mocap_trajectory.txt'
        
        # Initialize files with headers
        self.slam_file.write_text("# timestamp x y z qx qy qz qw\n")
        self.mocap_file.write_text("# timestamp x y z qx qy qz qw\n")
        
        self.get_logger().info(f"✓ Trajectory files initialized at {self.output_dir}")
        
        # Sensor to Body Center Offset (same as trajectory_node.py)
        self.local_offset = np.array([-0.2894, 0.0, 0.0468])
        
        # Subscribe to Point-LIO /Odometry (sensor frame)
        self.slam_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.slam_callback,
            10
        )
        self.get_logger().info("✓ Subscribed to /Odometry (Point-LIO sensor frame)")
        
        # Subscribe to mocap Path output (ground truth)
        self.mocap_sub = self.create_subscription(
            Path,
            '/mocap_path',
            self.mocap_path_callback,
            10
        )
        self.get_logger().info("✓ Subscribed to /mocap_path (MoCap ground truth)")
        
        # Track last written timestamps to avoid duplicates
        self.last_slam_ts = 0.0
        self.last_mocap_ts = 0.0
    
    def slam_callback(self, msg: Odometry):
        """
        Save Point-LIO SLAM trajectory.
        Applies sensor-to-body-center transformation (same as trajectory_node.py).
        """
        try:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            
            # Skip duplicates
            if abs(ts - self.last_slam_ts) < 0.001:
                return
            
            self.last_slam_ts = ts
            
            # Extract sensor frame position and orientation
            pos_sensor = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            quat_sensor = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            
            # Transform to body-center frame (same as trajectory_node.py odom_callback)
            rot_sensor = R.from_quat(quat_sensor)
            global_offset = rot_sensor.apply(self.local_offset)
            pos_body = pos_sensor + global_offset
            
            # Write body-center pose to file
            line = f"{ts:.6f} {pos_body[0]:.8f} {pos_body[1]:.8f} {pos_body[2]:.8f} " \
                   f"{quat_sensor[0]:.8f} {quat_sensor[1]:.8f} {quat_sensor[2]:.8f} {quat_sensor[3]:.8f}\n"
            
            with open(self.slam_file, 'a') as f:
                f.write(line)
            
        except Exception as e:
            self.get_logger().error(f"SLAM callback error: {e}", throttle_duration_sec=5)
    
    def mocap_path_callback(self, msg: Path):
        """Save mocap trajectory from Path messages."""
        try:
            if not msg.poses:
                return
            
            # Write only the latest pose to avoid duplicates
            if msg.poses:
                pose_stamped = msg.poses[-1]  # Get latest pose
                ts = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec / 1e9
                
                # Skip if this is a duplicate
                if abs(ts - self.last_mocap_ts) < 0.001:
                    return
                
                self.last_mocap_ts = ts
                
                pos = pose_stamped.pose.position
                quat = pose_stamped.pose.orientation
                
                line = f"{ts:.6f} {pos.x:.8f} {pos.y:.8f} {pos.z:.8f} " \
                       f"{quat.x:.8f} {quat.y:.8f} {quat.z:.8f} {quat.w:.8f}\n"
                
                with open(self.mocap_file, 'a') as f:
                    f.write(line)
            
        except Exception as e:
            self.get_logger().error(f"MoCap callback error: {e}", throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    
    listener = TrajectoryListener('/root/ros2_ws')
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
