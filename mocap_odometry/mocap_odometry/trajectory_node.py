import os
import sys
import numpy as np
import motioncapture
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TrajectoryEvaluator(Node):
    def __init__(self, target_body_name='go2'):
        super().__init__('trajectory_evaluator_node')

        # --- CONFIGURATION ---
        self.point_lio_odom_topic = '/Odometry'
        self.body_odom_topic = '/body_odom'
        self.body_path_topic = '/body_path'
        self.mocap_path_topic = '/mocap_path'
        self.fixed_frame = 'camera_init'
        self.target_body_name = target_body_name
        self.max_poses = 5000
        
        # Sensor to Body Center Offset
        self.local_offset = np.array([-0.2894, 0.0, 0.0468])

        # State Machine Variables
        self.start_time = None
        self.slam_ready = False
        self.mocap_aligned = False
        self.use_mocap = False
        
        self.slam_align_pos = None
        self.slam_align_rot = None
        self.mocap_initial_pos = None
        self.mocap_initial_rot_inv = None

        # File Pointers
        self.slam_file = None
        self.mocap_file = None

        # Callbacks Groups
        self.slam_cb_group = MutuallyExclusiveCallbackGroup()
        self.mocap_cb_group = MutuallyExclusiveCallbackGroup()

        # Publishers & Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 
            self.point_lio_odom_topic, 
            self.odom_callback, 
            10,
            callback_group=self.slam_cb_group
        )
        self.odom_pub = self.create_publisher(Odometry, self.body_odom_topic, 10)
        self.body_path_pub = self.create_publisher(Path, self.body_path_topic, 10)
        self.mocap_path_pub = self.create_publisher(Path, self.mocap_path_topic, 10)

        self.body_path_msg = Path()
        self.body_path_msg.header.frame_id = self.fixed_frame
        self.mocap_path_msg = Path()
        self.mocap_path_msg.header.frame_id = self.fixed_frame

        # MoCap Connection
        try:
            self.mocap = motioncapture.connect("optitrack", {'hostname': '192.168.2.141'})
            self.get_logger().info("Connected to OptiTrack. Waiting for first SLAM message...")
            self.use_mocap = True
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to OptiTrack: {e}. Running in SLAM-only mode.")
            self.use_mocap = False

        # MoCap Polling Timer (75Hz) - Only initialize if MoCap is connected
        if self.use_mocap:
            self.timer = self.create_timer(
                1.0 / 75.0, 
                self.mocap_timer_callback, 
                callback_group=self.mocap_cb_group
            )

    def init_logging_files(self):
        log_dir = os.path.expanduser('~/ros2_ws')
        self.slam_file = open(os.path.join(log_dir, 'slam_trajectory.txt'), 'w')
        self.slam_file.write("# timestamp x y z qx qy qz qw\n")
        
        if self.use_mocap:
            self.mocap_file = open(os.path.join(log_dir, 'mocap_trajectory.txt'), 'w')
            self.mocap_file.write("# timestamp x y z qx qy qz qw\n")
            
        self.get_logger().info(f"Log files created in {log_dir}. Logging started.")

    def odom_callback(self, msg: Odometry):
        current_ros_time = self.get_clock().now().nanoseconds * 1e-9

        # Set T=0 on first message
        if self.start_time is None:
            self.start_time = current_ros_time
            self.get_logger().info("First SLAM message received. 10-second timer started.")

        # 1. Transform SLAM to Body Center
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

        rot_sensor = R.from_quat(quat_sensor)
        global_offset = rot_sensor.apply(self.local_offset)
        pos_body = pos_sensor + global_offset

        # 2. Publish Body Odom & Path
        body_odom = Odometry()
        body_odom.header.stamp = msg.header.stamp
        body_odom.header.frame_id = self.fixed_frame
        body_odom.child_frame_id = 'body_center'
        body_odom.pose.pose.position.x = float(pos_body[0])
        body_odom.pose.pose.position.y = float(pos_body[1])
        body_odom.pose.pose.position.z = float(pos_body[2])
        body_odom.pose.pose.orientation = msg.pose.pose.orientation

        self.odom_pub.publish(body_odom)

        pose_stamped = PoseStamped()
        pose_stamped.header = body_odom.header
        pose_stamped.pose = body_odom.pose.pose

        self.body_path_msg.header.stamp = msg.header.stamp
        self.body_path_msg.poses.append(pose_stamped)
        if len(self.body_path_msg.poses) > self.max_poses:
            self.body_path_msg.poses.pop(0)
        self.body_path_pub.publish(self.body_path_msg)

        # 3. Handle 10-second alignment threshold
        elapsed = current_ros_time - self.start_time
        if elapsed >= 10.0 and not self.slam_ready:
            self.slam_align_pos = pos_body
            self.slam_align_rot = rot_sensor
            self.slam_ready = True
            
            if self.use_mocap:
                self.get_logger().info("10 seconds elapsed. SLAM pose locked. Awaiting next MoCap frame for alignment...")
            else:
                self.init_logging_files()
                self.get_logger().info("10 seconds elapsed. SLAM pose locked. Logging active in SLAM-only mode. Press Ctrl+C to stop.")

        # 4. Log SLAM data if fully aligned (or if running without MoCap)
        if self.mocap_aligned or (not self.use_mocap and self.slam_ready):
            t_slam = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
            ori = msg.pose.pose.orientation
            line = f"{t_slam:.6f} {pos_body[0]:.6f} {pos_body[1]:.6f} {pos_body[2]:.6f} {ori.x:.6f} {ori.y:.6f} {ori.z:.6f} {ori.w:.6f}\n"
            self.slam_file.write(line)
            self.slam_file.flush()

    def mocap_timer_callback(self):
        # Blocking call - safe here due to MultiThreadedExecutor
        self.mocap.waitForNextFrame()
        
        if not self.slam_ready:
            return
        
        body = self.mocap.rigidBodies.get(self.target_body_name)
        
        if body is None:
            return

        t_mocap = self.get_clock().now().nanoseconds * 1e-9
        now_msg = self.get_clock().now().to_msg()

        for name, body in self.mocap.rigidBodies.items():
            if self.target_body_name and name != self.target_body_name:
                continue

            pos_raw = np.array([body.position[0], body.position[1], body.position[2]])
            rot_raw = R.from_quat([body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w])

            # 1. Capture alignment frame
            if self.mocap_initial_pos is None:
                self.mocap_initial_pos = pos_raw
                self.mocap_initial_rot_inv = rot_raw.inv()
                self.init_logging_files()
                self.mocap_aligned = True
                self.get_logger().info("MoCap successfully aligned to SLAM origin. Trajectory logging active. Press Ctrl+C to stop and save files.")

            # 2. Align Data
            pos_zero = self.mocap_initial_rot_inv.apply(pos_raw - self.mocap_initial_pos)
            rot_zero = self.mocap_initial_rot_inv * rot_raw
            
            pos_aligned = self.slam_align_rot.apply(pos_zero) + self.slam_align_pos
            rot_aligned = self.slam_align_rot * rot_zero
            quat_aligned = rot_aligned.as_quat()

            # 3. Publish Mocap Path
            pose = PoseStamped()
            pose.header.stamp = now_msg
            pose.header.frame_id = self.fixed_frame
            pose.pose.position.x = float(pos_aligned[0])
            pose.pose.position.y = float(pos_aligned[1])
            pose.pose.position.z = float(pos_aligned[2])
            pose.pose.orientation.x = float(quat_aligned[0])
            pose.pose.orientation.y = float(quat_aligned[1])
            pose.pose.orientation.z = float(quat_aligned[2])
            pose.pose.orientation.w = float(quat_aligned[3])

            self.mocap_path_msg.header.stamp = now_msg
            self.mocap_path_msg.poses.append(pose)
            if len(self.mocap_path_msg.poses) > self.max_poses:
                self.mocap_path_msg.poses.pop(0)
            self.mocap_path_pub.publish(self.mocap_path_msg)

            # 4. Log Mocap Data
            if self.mocap_aligned:
                line = f"{t_mocap:.6f} {pos_aligned[0]:.6f} {pos_aligned[1]:.6f} {pos_aligned[2]:.6f} {quat_aligned[0]:.6f} {quat_aligned[1]:.6f} {quat_aligned[2]:.6f} {quat_aligned[3]:.6f}\n"
                self.mocap_file.write(line)
                self.mocap_file.flush()
            
            break 

    def shutdown_routine(self):
        self.get_logger().info("Shutting down node... ensuring data is saved.")
        if self.slam_file and not self.slam_file.closed:
            self.slam_file.flush()
            self.slam_file.close()
        if self.mocap_file and not self.mocap_file.closed:
            self.mocap_file.flush()
            self.mocap_file.close()
            
        if self.mocap_aligned or (not self.use_mocap and self.slam_ready):
            self.get_logger().info("Log files successfully saved and closed.")
        else:
            self.get_logger().info("Program exited before alignment. No log files were created.")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryEvaluator(target_body_name='go2')
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_routine()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()