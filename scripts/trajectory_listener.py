#!/usr/bin/env python3
"""
Trajectory Listener - Captures Point-LIO and MoCap trajectories from ROS topics.
Subscribes to /Odometry (Point-LIO) and /mocap_path (MoCap trajectory).
Saves to text files in format: timestamp x y z qx qy qz qw
Works with rosbag playback.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from pathlib import Path as FilePath


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
        
        # Subscribe to Point-LIO /Odometry output
        self.slam_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.slam_callback,
            10
        )
        self.get_logger().info("✓ Subscribed to /Odometry (Point-LIO)")
        
        # Subscribe to mocap Path output
        self.mocap_sub = self.create_subscription(
            Path,
            '/mocap_path',
            self.mocap_path_callback,
            10
        )
        self.get_logger().info("✓ Subscribed to /mocap_path (MoCap)")
        
        # Track last written timestamps to avoid duplicates from Path messages
        self.last_mocap_ts = 0.0
    
    def slam_callback(self, msg: Odometry):
        """Save Point-LIO SLAM trajectory from Odometry messages."""
        try:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            
            line = f"{ts:.6f} {pos.x:.8f} {pos.y:.8f} {pos.z:.8f} " \
                   f"{quat.x:.8f} {quat.y:.8f} {quat.z:.8f} {quat.w:.8f}\n"
            
            with open(self.slam_file, 'a') as f:
                f.write(line)
            
        except Exception as e:
            self.get_logger().error(f"SLAM callback error: {e}")
    
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
            self.get_logger().error(f"MoCap callback error: {e}")


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
