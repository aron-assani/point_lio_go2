import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import motioncapture
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

class MocapPublisherNode(Node):
    def __init__(self, target_body_name=None):
        super().__init__('publisher_node')
        
        self.target_body_name = target_body_name
        self.max_poses = 5000
        
        # Publishers & Subscribers
        self.path_publisher = self.create_publisher(Path, '/mocap_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/body_odom', self.odom_callback, 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'camera_init' 
        
        # SLAM Body Initial State
        self.slam_body_initial_pos = None
        self.slam_body_initial_rot = None
        
        # MoCap Initial State
        self.mocap_initial_pos = None
        self.mocap_initial_rot_inv = None
        
        try:
            self.mocap = motioncapture.connect("optitrack", {'hostname': '192.168.2.141'})
            self.get_logger().info("Connected to OptiTrack. Waiting for SLAM body odometry to initialize...")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect to OptiTrack: {e}")
            sys.exit(1)

        self.timer = self.create_timer(1.0/75.0, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        # Capture the very first SLAM body pose and lock it
        if self.slam_body_initial_pos is None:
            self.slam_body_initial_pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            self.slam_body_initial_rot = R.from_quat([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            self.get_logger().info("SLAM body origin locked. Ready to align MoCap data.")

    def timer_callback(self):
        # Don't process MoCap until SLAM has initialized the body position
        if self.slam_body_initial_pos is None:
            return

        self.mocap.waitForNextFrame()
        now = self.get_clock().now().to_msg()

        for name, body in self.mocap.rigidBodies.items():
            if self.target_body_name and name != self.target_body_name:
                continue

            pos_raw = np.array([body.position[0], body.position[1], body.position[2]])
            rot_raw = R.from_quat([body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w])

            # 1. Capture the first MoCap frame
            if self.mocap_initial_pos is None:
                self.mocap_initial_pos = pos_raw
                self.mocap_initial_rot_inv = rot_raw.inv()
                self.get_logger().info("Initial MoCap pose captured. Aligning trajectory to SLAM body origin.")

            # 2. Move MoCap to origin (0,0,0) mathematically
            pos_zero = self.mocap_initial_rot_inv.apply(pos_raw - self.mocap_initial_pos)
            rot_zero = self.mocap_initial_rot_inv * rot_raw

            # 3. Translate and rotate from origin to the SLAM body start position
            pos_aligned = self.slam_body_initial_rot.apply(pos_zero) + self.slam_body_initial_pos
            rot_aligned = self.slam_body_initial_rot * rot_zero
            quat_aligned = rot_aligned.as_quat()

            # 4. Construct the Pose
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.path_msg.header.frame_id

            pose.pose.position.x = float(pos_aligned[0])
            pose.pose.position.y = float(pos_aligned[1])
            pose.pose.position.z = float(pos_aligned[2])

            pose.pose.orientation.x = float(quat_aligned[0])
            pose.pose.orientation.y = float(quat_aligned[1])
            pose.pose.orientation.z = float(quat_aligned[2])
            pose.pose.orientation.w = float(quat_aligned[3])

            # 5. Update Path header and append
            self.path_msg.header.stamp = now
            self.path_msg.poses.append(pose)

            if len(self.path_msg.poses) > self.max_poses:
                self.path_msg.poses.pop(0)

            self.path_publisher.publish(self.path_msg)
            break

def main(args=None):
    rclpy.init(args=args)
    node = MocapPublisherNode(target_body_name=None) 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
