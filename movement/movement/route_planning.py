import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import heapq
import os

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    SDK_AVAILABLE = True
except ImportError:
    ChannelFactoryInitialize = None
    ChannelSubscriber = None
    SDK_AVAILABLE = False

# --- GRAPH DEFINITION ---
WAYPOINTS = {
    'A': (0.0, 0.0), 'B': (1.0, 0.0), 'C': (2.0, 0.0),
    'D': (1.0, 1.0), 'E': (2.0, 1.0)
}
EDGES = {
    'A': ['B'], 'B': ['A', 'C', 'D'], 'C': ['B', 'E'],
    'D': ['B', 'E'], 'E': ['C', 'D']
}

class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planning')

        self.declare_parameter('goal_node', 'E')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/robot/pose_estimate')
        self.declare_parameter('buffer_zone', 0.5)
        self.declare_parameter('replan_cooldown_s', 1.0)
        self.declare_parameter('network_interface', os.environ.get('NETWORK_INTERFACE', 'offline'))
        
        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('goal_pose_topic').value,
            self.goal_pose_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.network_interface = self.get_parameter('network_interface').value
        self.dds_topic = 'rt/utlidar/range_info'
        self.range_sub = None
        self.front_distance = float('inf')

        if SDK_AVAILABLE and self.network_interface != 'offline':
            ChannelFactoryInitialize(0, self.network_interface)
            self.range_sub = ChannelSubscriber(self.dds_topic)
            self.range_sub.InitChannel(self.range_callback)
            self.get_logger().info(f'Listening to Unitree DDS topic {self.dds_topic}.')
        else:
            self.get_logger().warn(
                'Unitree SDK unavailable or NETWORK_INTERFACE=offline; obstacle replanning will be disabled.'
            )
        
        # State
        self.current_pose = None
        self.current_yaw = 0.0
        self.path = []
        self.blocked_edges = set()
        self.current_node = None
        self.goal_node = self.get_parameter('goal_node').value
        self.buffer_zone = float(self.get_parameter('buffer_zone').value)
        self.replan_cooldown_s = float(self.get_parameter('replan_cooldown_s').value)
        self.last_replan_time = 0.0
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Route Planner started.")

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        if self.current_node is None:
            self.current_node = self.nearest_waypoint(self.current_pose.position.x, self.current_pose.position.y)
        if not self.path and self.current_node is not None:
            self.replan()

    def nearest_waypoint(self, x, y):
        return min(WAYPOINTS, key=lambda node: math.hypot(WAYPOINTS[node][0] - x, WAYPOINTS[node][1] - y))

    def goal_pose_callback(self, msg: PoseStamped):
        target_node = self.nearest_waypoint(msg.pose.position.x, msg.pose.position.y)
        self.goal_node = target_node
        self.get_logger().info(
            f"Received /goal_pose target ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}); using waypoint {target_node}."
        )

        if self.current_node is not None:
            self.replan()

    def blocked_edge(self, start, end):
        return (start, end) in self.blocked_edges or (end, start) in self.blocked_edges

    def build_path(self, start_node, goal_node):
        distances = {node: float('inf') for node in WAYPOINTS}
        distances[start_node] = 0.0
        previous = {}
        queue = [(0.0, start_node)]

        while queue:
            distance, current = heapq.heappop(queue)
            if distance > distances[current]:
                continue
            if current == goal_node:
                break

            for neighbor in EDGES[current]:
                if self.blocked_edge(current, neighbor):
                    continue

                candidate = distance + math.hypot(
                    WAYPOINTS[current][0] - WAYPOINTS[neighbor][0],
                    WAYPOINTS[current][1] - WAYPOINTS[neighbor][1],
                )
                if candidate < distances[neighbor]:
                    distances[neighbor] = candidate
                    previous[neighbor] = current
                    heapq.heappush(queue, (candidate, neighbor))

        if goal_node not in previous and goal_node != start_node:
            return []

        route = [goal_node]
        while route[0] != start_node:
            route.insert(0, previous[route[0]])
        return route

    def set_next_path(self):
        if self.current_node is None:
            return False

        path = self.build_path(self.current_node, self.goal_node)
        if not path:
            self.path = []
            self.get_logger().error(f"No path found from {self.current_node} to {self.goal_node}.")
            return False

        self.path = path[1:]
        self.get_logger().info(f"Path recalculated: {path}")
        return True

    def trigger_replan(self, blocked_neighbor=None):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < self.replan_cooldown_s:
            return

        if self.current_node is None:
            return

        if blocked_neighbor and self.path:
            self.blocked_edges.add((self.current_node, blocked_neighbor))

        self.last_replan_time = now
        self.replan()

    def _resolve_field(self, obj, field_name):
        value = getattr(obj, field_name, None)
        if value is None:
            return None
        return value() if callable(value) else value

    def range_callback(self, msg):
        """Handle rt/utlidar/range_info from the Unitree SDK."""
        point = self._resolve_field(msg, 'point')
        if point is None:
            return

        front_distance = self._resolve_field(point, 'x')
        try:
            front_distance = float(front_distance)
        except (TypeError, ValueError):
            return

        self.front_distance = front_distance

        if self.current_node is None or not self.path:
            return

        if math.isfinite(front_distance) and 0.0 < front_distance < self.buffer_zone:
            blocked_neighbor = self.path[0]
            self.get_logger().warn(
                f"Obstacle detected on {self.current_node}->{blocked_neighbor}; front range={front_distance:.2f} m. Replanning."
            )
            self.trigger_replan(blocked_neighbor)

    def replan(self):
        self.set_next_path()

    def control_loop(self):
        if not self.current_pose or not self.path: 
            return

        # Target coordinates (next node in path)
        target_node = self.path[0]
        tx, ty = WAYPOINTS[target_node]

        # Calculate errors
        dx = tx - self.current_pose.position.x
        dy = ty - self.current_pose.position.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        
        # Calculate yaw error (normalized to -pi to pi)
        yaw_error = angle_to_target - self.current_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        cmd = Twist()

        # 1. Arrival condition
        if dist < 0.15:
            self.get_logger().info(f"Reached {target_node}")
            self.current_node = self.path.pop(0)
            if not self.path:
                self.replan()
            return

        # 2. Steer towards target (Simple PD Controller)
        # If yaw error is high, spin in place (linear.x = 0)
        if abs(yaw_error) > 0.2:
            cmd.angular.z = max(-0.5, min(0.5, yaw_error * 2.0))
            cmd.linear.x = 0.0 
        else:
            # If facing target, move forward
            cmd.angular.z = yaw_error * 1.0
            cmd.linear.x = min(0.3, dist * 0.5) 

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RoutePlanner()
    rclpy.spin(node)
    rclpy.shutdown()