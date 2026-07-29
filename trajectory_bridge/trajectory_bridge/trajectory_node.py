import os
import time
import threading
import numpy as np
try:
    import motioncapture
except ImportError:
    motioncapture = None
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TrajectoryEvaluator(Node):
    def __init__(self, target_body_name='go2'):
        super().__init__('trajectory_evaluator_node')

        # Config
        self.optitrack_enabled = bool(self.declare_parameter('enable_optitrack', True).value)
        self.slam_odom_topic = '/slam/odometry'
        self.body_pose_topic = '/robot/pose_estimate'
        self.slam_path_topic = '/robot/path_slam'
        self.mocap_path_topic = '/robot/path_mocap'
        self.fixed_frame = 'camera_init'
        self.target_body_name = target_body_name
        self.max_poses = 5000
        self.optitrack_connect_timeout_sec = 3.0
        
        # Sensor to Body Center Offset
        self.local_offset = np.array([-0.2894, 0.0, 0.0468])

        # State
        self.start_time = None
        self.slam_ready = False
        self.mocap_aligned = False
        self.use_mocap = False
        
        self.slam_align_pos = None
        self.slam_align_rot = None
        self.mocap_initial_pos = None
        self.mocap_initial_rot_inv = None

        # File handles
        self.slam_file = None
        self.mocap_file = None

        self.odom_sub = self.create_subscription(
            Odometry, 
            self.slam_odom_topic, 
            self.odom_callback, 
            10
        )
        self.pose_pub = self.create_publisher(Odometry, self.body_pose_topic, 10)
        self.slam_path_pub = self.create_publisher(Path, self.slam_path_topic, 10)
        self.mocap_path_pub = self.create_publisher(Path, self.mocap_path_topic, 10)

        self.slam_path_msg = Path()
        self.slam_path_msg.header.frame_id = self.fixed_frame
        self.mocap_path_msg = Path()
        self.mocap_path_msg.header.frame_id = self.fixed_frame
        
        # Thread handles
        self.mocap_thread = None
        self.is_running = True

        # MoCap connection
        if not self.optitrack_enabled:
            self.get_logger().info('OptiTrack is disabled by launch argument. Running in SLAM-only mode.')
        elif motioncapture is None:
            self.get_logger().warn('motioncapture Python package is not available. Running in SLAM-only mode.')
            self.use_mocap = False
        else:
            connected = self.try_connect_optitrack_with_timeout(self.optitrack_connect_timeout_sec)
            if connected:
                self.get_logger().info('Connected to OptiTrack.')
                self.use_mocap = True
                
                # Start dedicated background thread for MoCap polling
                self.mocap_thread = threading.Thread(target=self.mocap_worker_loop, daemon=True)
                self.mocap_thread.start()
            else:
                self.use_mocap = False

    def try_connect_optitrack_with_timeout(self, timeout_sec):
        result = {'client': None, 'error': None}

        def connect_worker():
            try:
                result['client'] = motioncapture.connect("optitrack", {'hostname': '192.168.2.141'})
            except Exception as exc:
                result['error'] = exc

        thread = threading.Thread(target=connect_worker, daemon=True)
        thread.start()
        thread.join(timeout=timeout_sec)

        if thread.is_alive():
            self.get_logger().warn(
                f"Timed out after {timeout_sec:.1f}s while connecting to OptiTrack. Running in SLAM-only mode."
            )
            return False

        if result['error'] is not None:
            self.get_logger().warn(f"Failed to connect to OptiTrack: {result['error']}. Running in SLAM-only mode.")
            return False

        self.mocap = result['client']
        return self.mocap is not None

    def init_logging_files(self):
        log_dir = os.path.expanduser('~/ros2_ws')
        self.slam_file = open(os.path.join(log_dir, 'slam_trajectory.txt'), 'w')
        self.slam_file.write("# timestamp x y z qx qy qz qw\n")
        
        self.mocap_file = open(os.path.join(log_dir, 'mocap_trajectory.txt'), 'w')
        self.mocap_file.write("# timestamp x y z qx qy qz qw\n")
            
        self.get_logger().info(f"Log files created in {log_dir}. Logging started.")

    def odom_callback(self, msg: Odometry):
        current_ros_time = self.get_clock().now().nanoseconds * 1e-9

        # Set T=0 on first message
        if self.start_time is None:
            self.start_time = current_ros_time
            self.get_logger().info('First SLAM message received.')

        elapsed = current_ros_time - self.start_time

        # 1. Transform SLAM to body center
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

        # 2. Publish body pose in SLAM frame
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

        # 3. Handle 3-second alignment threshold
        if elapsed >= 3.0 and not self.slam_ready:
            self.slam_align_pos = pos_body
            self.slam_align_rot = rot_sensor
            self.slam_ready = True
            
            if self.use_mocap:
                self.get_logger().info('SLAM pose locked. Awaiting first OptiTrack frame.')
            else:
                self.init_logging_files()
                self.get_logger().info('SLAM pose locked. SLAM-only mode active.')

        # 4. Log SLAM data after SLAM is ready
        if self.slam_ready and self.slam_file and not self.slam_file.closed:
            t_slam = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
            line = f"{t_slam:.6f} {pos_body[0]:.6f} {pos_body[1]:.6f} {pos_body[2]:.6f} {quat_sensor[0]:.6f} {quat_sensor[1]:.6f} {quat_sensor[2]:.6f} {quat_sensor[3]:.6f}\n"
            self.slam_file.write(line)
            self.slam_file.flush()

    def mocap_worker_loop(self):
        """Dedicated thread to safely block on OptiTrack frames without halting ROS."""
        while rclpy.ok() and self.is_running:
            try:
                self.mocap.waitForNextFrame()
                
                if not self.slam_ready:
                    continue
                
                body = self.mocap.rigidBodies.get(self.target_body_name)
                if body is None:
                    continue

                t_mocap = self.get_clock().now().nanoseconds * 1e-9
                now_msg = self.get_clock().now().to_msg()

                pos_raw = np.array([body.position[0], body.position[1], body.position[2]])
                rot_raw = R.from_quat([body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w])

                # 1. Capture alignment frame
                if self.mocap_initial_pos is None:
                    self.mocap_initial_pos = pos_raw
                    self.mocap_initial_rot_inv = rot_raw.inv()
                    self.init_logging_files()
                    self.mocap_aligned = True
                    self.get_logger().info('OptiTrack aligned to SLAM frame. Publishing mocap trajectory.')

                # 2. Align data
                pos_zero = self.mocap_initial_rot_inv.apply(pos_raw - self.mocap_initial_pos)
                rot_zero = self.mocap_initial_rot_inv * rot_raw
                
                pos_aligned = self.slam_align_rot.apply(pos_zero) + self.slam_align_pos
                rot_aligned = self.slam_align_rot * rot_zero
                quat_aligned = rot_aligned.as_quat()

                # 3. Publish MoCap path
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

                # 4. Log
                if self.mocap_aligned and self.mocap_file and not self.mocap_file.closed:
                    line = f"{t_mocap:.6f} {pos_aligned[0]:.6f} {pos_aligned[1]:.6f} {pos_aligned[2]:.6f} {quat_aligned[0]:.6f} {quat_aligned[1]:.6f} {quat_aligned[2]:.6f} {quat_aligned[3]:.6f}\n"
                    self.mocap_file.write(line)
                    self.mocap_file.flush()

            except Exception as e:
                self.get_logger().warn(f'OptiTrack read error: {e}')
                time.sleep(0.01)

    def shutdown_routine(self):
        self.is_running = False
        if self.mocap_thread and self.mocap_thread.is_alive():
            self.mocap_thread.join(timeout=1.0)
            
        if self.slam_file and not self.slam_file.closed:
            self.slam_file.flush()
            self.slam_file.close()
        if self.mocap_file and not self.mocap_file.closed:
            self.mocap_file.flush()
            self.mocap_file.close()


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