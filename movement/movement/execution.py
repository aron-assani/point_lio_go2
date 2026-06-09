import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class StraightLineMovement(Node):
    def __init__(self):
        super().__init__('straight_line_movement')

        self.declare_parameter('slam_topic', '/robot/path_slam')
        self.declare_parameter('plan_topic', '/robot/path_straight_line')

        self.slam_topic = self.get_parameter('slam_topic').value
        self.plan_topic = self.get_parameter('plan_topic').value

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.has_active_target = False

        self.path_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.slam_sub = self.create_subscription(Path, self.slam_topic, self.slam_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

    def goal_callback(self, msg):
        self.target_x = float(msg.pose.position.x)
        self.target_y = float(msg.pose.position.y)
        self.target_z = float(msg.pose.position.z)
        self.has_active_target = True
        self.get_logger().info(f"New target received: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")

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
        if not self.has_active_target:
            return

        current_pose = msg.poses[-1].pose
        self.path_pub.publish(self.plan_straight_line(current_pose, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()