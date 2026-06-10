import math
import sys
import os
import threading
import select
import termios
import tty
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False


class PathExecutor(Node):
    def __init__(self, is_offline=False):
        super().__init__('path_executor')
        
        self.is_offline = is_offline

        self.declare_parameter('path_topic', '/robot/path_straight_line')
        self.declare_parameter('slam_topic', '/robot/path_slam')
        self.declare_parameter('vx_gain', 1.0)
        self.declare_parameter('vy_gain', 1.0)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.15)
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_yaw_rate', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        
        # Ramp params
        self.declare_parameter('max_linear_accel', 0.4)  # m/s^2
        self.declare_parameter('max_yaw_accel', 0.8)     # rad/s^2

        self.path_topic = self.get_parameter('path_topic').value
        self.slam_topic = self.get_parameter('slam_topic').value
        self.vx_gain = float(self.get_parameter('vx_gain').value)
        self.vy_gain = float(self.get_parameter('vy_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.control_period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        
        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_yaw_accel = float(self.get_parameter('max_yaw_accel').value)

        self.path_lock = threading.Lock()
        self.active_target = None
        self.active_yaw = 0.0
        self.last_reached_target = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_yaw_rate = 0.0

        if not self.is_offline and SDK_AVAILABLE:
            # Init sport client API
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

            # Init obstacle avoidance client
            self.avoid_client = ObstaclesAvoidClient()
            self.avoid_client.SetTimeout(10.0)
            self.avoid_client.Init()
            
            self.get_logger().info("Activating hardware obstacle avoidance subsystem...")
            self.avoid_client.SwitchSet(True)
            self.avoid_client.UseRemoteCommandFromApi(True)
        else:
            self.get_logger().warn("OFFLINE MODE: Hardware clients bypassed. Simulating velocity commands.")

        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.slam_sub = self.create_subscription(Path, self.slam_topic, self.slam_callback, 10)
        self.control_timer = self.create_timer(self.control_period, self.control_callback)

        # Keyboard listener
        self.keyboard_active = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and self.keyboard_active:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'p':
                        self.get_logger().error("Emergency Stop Key 'P' pressed! Halting execution node.")
                        self.shutdown_clients()
                        time.sleep(0.2)
                        os._exit(0)
        except Exception as e:
            self.get_logger().error(f"Keyboard listener failed: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def path_callback(self, msg):
        if not msg.poses:
            return

        target_pose = msg.poses[-1].pose
        new_target = (
            float(target_pose.position.x),
            float(target_pose.position.y),
            float(target_pose.position.z),
        )

        with self.path_lock:
            if self.last_reached_target == new_target:
                return

            self.active_target = new_target
            self.active_yaw = self.quaternion_to_yaw(
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            )

    def slam_callback(self, msg):
        if not msg.poses:
            return

        pose = msg.poses[-1].pose
        self.current_x = float(pose.position.x)
        self.current_y = float(pose.position.y)
        self.current_z = float(pose.position.z)
        self.current_yaw = self.quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def control_callback(self):
        with self.path_lock:
            target = self.active_target
            target_yaw = self.active_yaw

        if target is None:
            self.apply_ramp_and_send(0.0, 0.0, 0.0)
            return

        error_x = target[0] - self.current_x
        error_y = target[1] - self.current_y
        distance = math.hypot(error_x, error_y)

        if distance <= self.position_tolerance:
            if not self.is_offline:
                self.avoid_client.Move(0.0, 0.0, 0.0)
            else:
                self.get_logger().info("Target reached in offline mode.", throttle_duration_sec=2.0)
                
            self.prev_vx, self.prev_vy, self.prev_yaw_rate = 0.0, 0.0, 0.0
            with self.path_lock:
                self.last_reached_target = self.active_target
                self.active_target = None
            return

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        body_x = cos_yaw * error_x + sin_yaw * error_y
        body_y = -sin_yaw * error_x + cos_yaw * error_y

        target_vx = max(-self.max_vx, min(self.max_vx, self.vx_gain * body_x))
        target_vy = max(-self.max_vy, min(self.max_vy, self.vy_gain * body_y))

        yaw_error = self.wrap_angle(target_yaw - self.current_yaw)
        if abs(yaw_error) <= self.yaw_tolerance:
            target_yaw_rate = 0.0
        else:
            target_yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.yaw_gain * yaw_error))

        self.apply_ramp_and_send(target_vx, target_vy, target_yaw_rate)

    def apply_ramp_and_send(self, target_vx, target_vy, target_yaw_rate):
        """Saturates step changes in velocity and executes via Avoidance Client."""
        max_dv_linear = self.max_linear_accel * self.control_period
        max_dv_yaw = self.max_yaw_accel * self.control_period

        # Apply limits
        vx = max(self.prev_vx - max_dv_linear, min(self.prev_vx + max_dv_linear, target_vx))
        vy = max(self.prev_vy - max_dv_linear, min(self.prev_vy + max_dv_linear, target_vy))
        yaw_rate = max(self.prev_yaw_rate - max_dv_yaw, min(self.prev_yaw_rate + max_dv_yaw, target_yaw_rate))

        # Update tracking state
        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_yaw_rate = yaw_rate

        if not self.is_offline:
            self.avoid_client.Move(vx, vy, yaw_rate)
        else:
            pass

    def shutdown_clients(self):
        """Safely releases control APIs and halts the robot."""
        if self.is_offline:
            self.get_logger().info("Offline mode: Skipping hardware shutdown sequence.")
            return
            
        self.get_logger().info("Releasing remote API control and halting...")
        if SDK_AVAILABLE:
            self.avoid_client.Move(0.0, 0.0, 0.0)
            self.avoid_client.UseRemoteCommandFromApi(False)
            self.avoid_client.SwitchSet(False)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    network_interface = os.environ.get('NETWORK_INTERFACE', 'offline')
    is_offline = (network_interface.lower() == 'offline')

    if not is_offline:
        if SDK_AVAILABLE:
            print(f"[PathExecutor] Initializing Unitree SDK on interface: {network_interface}")
            ChannelFactoryInitialize(0, network_interface)
        else:
            print("[PathExecutor] FATAL: Not in offline mode, but Unitree SDK is not installed. Forcing offline mode.")
            is_offline = True
    else:
        print("[PathExecutor] NETWORK_INTERFACE set to 'offline'. Running in simulation mode.")

    rclpy.init(args=args)
    node = PathExecutor(is_offline=is_offline)

    try:
        rclpy.spin(node)
    finally:
        node.keyboard_active = False
        node.shutdown_clients()
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()