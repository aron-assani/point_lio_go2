import sys
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False


class Nav2Executor(Node):
    def __init__(self, is_offline=False):
        super().__init__('nav2_executor')
        
        self.is_offline = is_offline
        
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_yaw_rate', 0.5)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        if not self.is_offline and SDK_AVAILABLE:
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            self.get_logger().info("Execution node initialized as a pure velocity bridge. Awaiting /cmd_vel.")
        else:
            self.get_logger().warn("OFFLINE MODE: Hardware clients bypassed.")

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.estop_sub = self.create_subscription(Empty, '/emergency_stop', self.estop_callback, 10)
        
        self.last_cmd_time = time.time()
        self.last_move_time = 0.0
        self.first_message_received = False
        
        # Track whether the robot is currently moving to avoid spamming StopMove
        self.is_moving = False 
        
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)

    def estop_callback(self, msg):
        self.get_logger().error("Emergency Stop received! Halting execution node.")
        self.shutdown_clients()
        time.sleep(0.2)
        os._exit(0)

    def cmd_vel_callback(self, msg: Twist):
        current_time = time.time()
        self.last_cmd_time = current_time

        if not self.first_message_received:
            self.get_logger().info(">>> SUCCESS: First /cmd_vel message received from Nav2! <<<")
            self.first_message_received = True
        
        vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.angular.z))

        # Check if Nav2 is commanding a full stop
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(yaw_rate) < 0.01:
            if self.is_moving:
                if not self.is_offline:
                    self.sport_client.StopMove()
                self.is_moving = False
            return

        # RATE LIMITER: Only send Move() commands every 0.5 seconds (2Hz) max
        if current_time - self.last_move_time >= 0.5:
            if not self.is_offline:
                self.sport_client.Move(vx, vy, yaw_rate)
            self.last_move_time = current_time
            self.is_moving = True

    def watchdog_check(self):
        """Halts the robot if no velocity commands are received for 0.5 seconds."""
        if time.time() - self.last_cmd_time > 0.5:
            if self.is_moving:
                if not self.is_offline:
                    self.sport_client.StopMove()
                self.is_moving = False

    def shutdown_clients(self):
        if self.is_offline:
            return
            
        self.get_logger().info("Execution node shutting down. Sending StopMove()...")
        if SDK_AVAILABLE:
            self.sport_client.StopMove()


def main(args=None):
    network_interface = os.environ.get('NETWORK_INTERFACE', 'offline')
    is_offline = (network_interface.lower() == 'offline')

    if not is_offline:
        if SDK_AVAILABLE:
            try:
                print(f"[Nav2Executor] Initializing Unitree SDK on interface: {network_interface}")
                ChannelFactoryInitialize(0, network_interface)
            except Exception as e:
                print(f"[Nav2Executor] ERROR: Failed to bind to {network_interface}. Network is down.")
                print(f"[Nav2Executor] Exception: {e}")
                print("[Nav2Executor] Falling back to OFFLINE mode to keep ROS node alive.")
                is_offline = True
        else:
            print("FATAL: Not in offline mode, but Unitree SDK is not installed.")
            is_offline = True

    rclpy.init(args=args)
    node = Nav2Executor(is_offline=is_offline)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_clients()
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()