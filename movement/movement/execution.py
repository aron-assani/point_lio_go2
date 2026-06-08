import math
import sys
import threading

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


class PathExecutor(Node):
    def __init__(self):
        super().__init__('path_executor')

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

        self.path_lock = threading.Lock()
        self.active_target = None
        self.active_yaw = 0.0
        self.last_reached_target = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.slam_sub = self.create_subscription(Path, self.slam_topic, self.slam_callback, 10)
        self.control_timer = self.create_timer(self.control_period, self.control_callback)

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
            return

        error_x = target[0] - self.current_x
        error_y = target[1] - self.current_y
        distance = math.hypot(error_x, error_y)

        if distance <= self.position_tolerance:
            self.sport_client.StopMove()
            with self.path_lock:
                self.last_reached_target = self.active_target
                self.active_target = None
            return

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        body_x = cos_yaw * error_x + sin_yaw * error_y
        body_y = -sin_yaw * error_x + cos_yaw * error_y

        vx = max(-self.max_vx, min(self.max_vx, self.vx_gain * body_x))
        vy = max(-self.max_vy, min(self.max_vy, self.vy_gain * body_y))

        yaw_error = self.wrap_angle(target_yaw - self.current_yaw)
        if abs(yaw_error) <= self.yaw_tolerance:
            yaw_rate = 0.0
        else:
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.yaw_gain * yaw_error))

        self.sport_client.Move(vx, vy, yaw_rate)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0, 'enx00133b9a06ef')

    rclpy.init(args=args)
    node = PathExecutor()

    try:
        rclpy.spin(node)
    finally:
        node.sport_client.StopMove()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()