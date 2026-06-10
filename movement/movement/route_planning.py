import math
import sys
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty


class StraightLineMovement(Node):
    def __init__(self, target_xyz=None):
        super().__init__('straight_line_movement')

        self.declare_parameter('slam_topic', '/robot/path_slam')
        self.declare_parameter('plan_topic', '/robot/path_straight_line')
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.0)

        self.slam_topic = self.get_parameter('slam_topic').value
        self.plan_topic = self.get_parameter('plan_topic').value

        if target_xyz is None:
            self.target_x = float(self.get_parameter('target_x').value)
            self.target_y = float(self.get_parameter('target_y').value)
            self.target_z = float(self.get_parameter('target_z').value)
        else:
            self.target_x = float(target_xyz[0])
            self.target_y = float(target_xyz[1])
            self.target_z = float(target_xyz[2])

        self.path_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.path_sub = self.create_subscription(Path, self.slam_topic, self.slam_callback, 10)
        
        # E-stop subscription
        self.estop_sub = self.create_subscription(Empty, '/emergency_stop', self.estop_callback, 10)

    def estop_callback(self, msg):
        self.get_logger().error("Emergency Stop received via ROS topic! Halting planning node.")
        os._exit(0)

    def set_target(self, x, y, z):
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = float(z)

    def plan_straight_line(self, current_pose, header):
        planned_path = Path()
        planned_path.header = header

        start_pose = PoseStamped()
        start_pose.header = header
        start_pose.pose = current_pose

        goal_pose = PoseStamped()
        goal_pose.header = header
        goal_pose.pose.position.x = self.target_x
        goal_pose.pose.position.y = self.target_y
        goal_pose.pose.position.z = self.target_z

        dx = self.target_x - current_pose.position.x
        dy = self.target_y - current_pose.position.y
        
        if math.hypot(dx, dy) > 0.05:
            yaw = math.atan2(dy, dx)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        else:
            goal_pose.pose.orientation = current_pose.orientation

        planned_path.poses = [start_pose, goal_pose]
        return planned_path

    def slam_callback(self, msg):
        current_pose = msg.poses[-1].pose
        self.path_pub.publish(self.plan_straight_line(current_pose, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineMovement()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()