import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import motioncapture
import sys
import os

class TrajectoryLoggerNode(Node):
    def __init__(self, target_body_name=None):
        super().__init__('trajectory_logger_node')

        self.target_body_name = target_body_name

        # Open log files in the workspace root
        log_dir = os.path.expanduser('~/ros2_ws')
        self.slam_file = open(os.path.join(log_dir, 'slam_trajectory.txt'), 'w')
        self.mocap_file = open(os.path.join(log_dir, 'mocap_trajectory.txt'), 'w')

        # TUM Format headers (ignored by EVO, helpful for humans)
        self.slam_file.write("# timestamp x y z qx qy qz qw\n")
        self.mocap_file.write("# timestamp x y z qx qy qz qw\n")

        # 1. SLAM Subscriber
        # Ensure '/Odometry' matches the exact topic point_lio is publishing to
        self.subscription = self.create_subscription(
            Odometry,
            '/body_odom',
            self.odom_callback,
            10)

        # 2. Mocap UDP Connection
        try:
            self.mocap = motioncapture.connect("optitrack", {'hostname': '192.168.2.141'})
            self.get_logger().info("Connected to OptiTrack.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect to OptiTrack: {e}")
            sys.exit(1)

        # 3. Mocap Polling Loop (75Hz)
        self.timer = self.create_timer(1.0/75.0, self.mocap_timer_callback)
        self.get_logger().info("Logging started. Press Ctrl+C to stop and save files.")

    def odom_callback(self, msg):
        # Extract ROS timestamp and convert to float seconds
        t = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        line = f"{t:.6f} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f} {ori.x:.6f} {ori.y:.6f} {ori.z:.6f} {ori.w:.6f}\n"
        self.slam_file.write(line)

    def mocap_timer_callback(self):
        self.mocap.waitForNextFrame()

        # Use ROS clock for Mocap to perfectly sync timestamps with SLAM data
        t = self.get_clock().now().nanoseconds * 1e-9

        for name, body in self.mocap.rigidBodies.items():
            if self.target_body_name and name != self.target_body_name:
                continue

            line = f"{t:.6f} {body.position[0]:.6f} {body.position[1]:.6f} {body.position[2]:.6f} " \
                   f"{body.rotation.x:.6f} {body.rotation.y:.6f} {body.rotation.z:.6f} {body.rotation.w:.6f}\n"
            self.mocap_file.write(line)

    def destroy_node(self):
        # Ensure files are saved and closed properly when shutting down
        self.slam_file.close()
        self.mocap_file.close()
        self.get_logger().info("Log files saved to ~/ros2_ws/")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLoggerNode(target_body_name=None)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

