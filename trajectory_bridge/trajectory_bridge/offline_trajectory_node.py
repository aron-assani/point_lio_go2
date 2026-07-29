import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OfflineTrajectoryEvaluator(Node):
    def __init__(self):
        super().__init__('offline_trajectory_evaluator_node')

        # Topic configuration
        self.slam_odom_topic = '/slam/odometry'
        self.body_pose_topic = '/robot/pose_estimate'
        self.slam_path_topic = '/robot/path_slam'
        
        self.mocap_ref_topic = '/mocap_path'        # raw data from rosbag
        self.mocap_path_topic = '/robot/path_mocap' # aligned data for RViz
        
        self.fixed_frame = 'camera_init'
        self.max_poses = 5000

        # Offline behavior
        self.offline_ref_timeout_sec = 2.0
        self.allow_body_fallback_without_reference = True

        # Sensor to body-center offset
        self.local_offset = np.array([-0.2894, 0.0, 0.0468])

        # State
        self.start_time = None
        self.slam_ready = False
        self.mocap_aligned = False

        self.slam_align_pos = None
        self.slam_align_rot = None
        self.offline_ref_pos = None
        self.offline_ref_rot = None
        self.offline_alignment_ready = False
        self.offline_mocap_to_slam_rot = R.identity()
        self.offline_mocap_to_slam_trans = np.zeros(3)

        self.offline_ref_count = 0
        self.last_offline_ref_wall_time = None
        self.last_body_msg_wall_time = None
        self.offline_fallback_active = False

        # File handles
        self.slam_file = None
        self.mocap_file = None

        # Callback groups
        self.slam_cb_group = MutuallyExclusiveCallbackGroup()
        self.ref_cb_group = MutuallyExclusiveCallbackGroup()

        # I/O
        self.odom_sub = self.create_subscription(
            Odometry,
            self.slam_odom_topic,
            self.odom_callback,
            10,
            callback_group=self.slam_cb_group,
        )
        self.offline_mocap_ref_sub = self.create_subscription(
            Path,
            self.mocap_ref_topic,
            self.offline_mocap_ref_callback,
            10,
            callback_group=self.ref_cb_group,
        )

        self.pose_pub = self.create_publisher(Odometry, self.body_pose_topic, 10)
        self.slam_path_pub = self.create_publisher(Path, self.slam_path_topic, 10)
        self.mocap_path_pub = self.create_publisher(Path, self.mocap_path_topic, 10)

        self.body_path_msg = Path()
        self.body_path_msg.header.frame_id = self.fixed_frame
        self.mocap_path_msg = Path()
        self.mocap_path_msg.header.frame_id = self.fixed_frame
        self.get_logger().info('Offline trajectory node started. OptiTrack connection is disabled.')

    def init_logging_files(self):
        log_dir = os.path.expanduser('~/ros2_ws')
        self.slam_file = open(os.path.join(log_dir, 'slam_trajectory.txt'), 'w')
        self.slam_file.write('# timestamp x y z qx qy qz qw\n')

        self.mocap_file = open(os.path.join(log_dir, 'mocap_trajectory.txt'), 'w')
        self.mocap_file.write('# timestamp x y z qx qy qz qw\n')

        self.get_logger().info(f'Log files created in {log_dir}. Logging started.')

    def offline_mocap_ref_callback(self, msg: Path):
        if not msg.poses:
            return

        self.offline_ref_count += 1
        self.last_offline_ref_wall_time = time.monotonic()

        if not self.slam_ready:
            return

        pose = msg.poses[-1].pose
        pos_raw = np.array([pose.position.x, pose.position.y, pose.position.z])
        rot_raw = R.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])

        self.offline_ref_pos = pos_raw
        self.offline_ref_rot = rot_raw

        if not self.offline_alignment_ready and self.slam_align_pos is not None:
            self.offline_mocap_to_slam_rot = self.slam_align_rot * self.offline_ref_rot.inv()
            self.offline_mocap_to_slam_trans = self.slam_align_pos - self.offline_mocap_to_slam_rot.apply(self.offline_ref_pos)
            self.offline_alignment_ready = True
            self.mocap_aligned = True
            self.offline_fallback_active = False
            self.get_logger().info('Offline alignment initialized. Publishing aligned /mocap_path.')

        if not self.offline_alignment_ready:
            return

        pos_aligned = self.offline_mocap_to_slam_rot.apply(pos_raw) + self.offline_mocap_to_slam_trans
        rot_aligned = self.offline_mocap_to_slam_rot * rot_raw
        quat_aligned = rot_aligned.as_quat()

        pose_out = PoseStamped()
        pose_out.header.stamp = msg.poses[-1].header.stamp
        pose_out.header.frame_id = self.fixed_frame
        pose_out.pose.position.x = float(pos_aligned[0])
        pose_out.pose.position.y = float(pos_aligned[1])
        pose_out.pose.position.z = float(pos_aligned[2])
        pose_out.pose.orientation.x = float(quat_aligned[0])
        pose_out.pose.orientation.y = float(quat_aligned[1])
        pose_out.pose.orientation.z = float(quat_aligned[2])
        pose_out.pose.orientation.w = float(quat_aligned[3])

        self.mocap_path_msg.header.stamp = pose_out.header.stamp
        self.mocap_path_msg.poses.append(pose_out)
        if len(self.mocap_path_msg.poses) > self.max_poses:
            self.mocap_path_msg.poses.pop(0)
        self.mocap_path_pub.publish(self.mocap_path_msg)

        if self.mocap_file and not self.mocap_file.closed:
            t_mocap = pose_out.header.stamp.sec + (pose_out.header.stamp.nanosec * 1e-9)
            line = (
                f'{t_mocap:.6f} {pos_aligned[0]:.6f} {pos_aligned[1]:.6f} {pos_aligned[2]:.6f} '
                f'{quat_aligned[0]:.6f} {quat_aligned[1]:.6f} {quat_aligned[2]:.6f} {quat_aligned[3]:.6f}\n'
            )
            self.mocap_file.write(line)
            self.mocap_file.flush()

    def odom_callback(self, msg: Odometry):
        current_ros_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_body_msg_wall_time = time.monotonic()

        if self.start_time is None:
            self.start_time = current_ros_time
            self.get_logger().info('First SLAM message received. Warmup timer started.')

        elapsed = current_ros_time - self.start_time

        pos_sensor = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        quat_sensor = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

        rot_sensor = R.from_quat(quat_sensor)
        global_offset = rot_sensor.apply(self.local_offset)
        pos_body = pos_sensor + global_offset

        body_pose = Odometry()
        body_pose.header.stamp = msg.header.stamp
        body_pose.header.frame_id = self.fixed_frame
        body_pose.child_frame_id = 'body_center'
        body_pose.pose.pose.position.x = float(pos_body[0])
        body_pose.pose.pose.position.y = float(pos_body[1])
        body_pose.pose.pose.position.z = float(pos_body[2])
        body_pose.pose.pose.orientation.x = float(quat_sensor[0])
        body_pose.pose.pose.orientation.y = float(quat_sensor[1])
        body_pose.pose.pose.orientation.z = float(quat_sensor[2])
        body_pose.pose.pose.orientation.w = float(quat_sensor[3])
        self.pose_pub.publish(body_pose)

        pose_stamped = PoseStamped()
        pose_stamped.header = body_pose.header
        pose_stamped.pose = body_pose.pose.pose

        self.slam_path_msg.header.stamp = msg.header.stamp
        self.slam_path_msg.poses.append(pose_stamped)
        if len(self.slam_path_msg.poses) > self.max_poses:
            self.slam_path_msg.poses.pop(0)
        self.slam_path_pub.publish(self.slam_path_msg)