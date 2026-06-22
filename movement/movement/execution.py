import sys
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

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
        self.declare_parameter('max_yaw_rate', 0.8)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        if not self.is_offline and SDK_AVAILABLE:
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

            self.get_logger().info("Initializing robot posture... Standing up.")
            self.sport_client.StandUp()
            time.sleep(2.0)
            
            self.get_logger().info("Engaging FreeWalk mode to unlock velocity tracking...")
            self.sport_client.FreeWalk()
            time.sleep(1.0)
        else:
            self.get_logger().warn("OFFLINE MODE: Hardware clients bypassed.")

        # Subscribe directly to Nav2's velocity output
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.estop_sub = self.create_subscription(Empty, '/emergency_stop', self.estop_callback, 10)
        
        # Watchdog timer: If Nav2 crashes and stops sending cmd_vel, halt the robot.
        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)

    def estop_callback(self, msg):
        self.get_logger().error("Emergency Stop received via ROS topic! Halting execution node.")
        self.shutdown_clients()
        time.sleep(0.2)
        os._exit(0)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        
        # Extract and clamp standard ROS velocities
        vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.angular.z))

        if not self.is_offline:
            self.sport_client.Move(vx, vy, yaw_rate)

    def watchdog_check(self):
        """Halts the robot if no velocity commands are received for 0.5 seconds."""
        if time.time() - self.last_cmd_time > 0.5:
            if not self.is_offline:
                self.sport_client.StopMove()

    def shutdown_clients(self):
        if self.is_offline:
            return
            
        self.get_logger().info("Halting robot and standing down...")
        if SDK_AVAILABLE:
            self.sport_client.StopMove()
            self.sport_client.StandDown()


def main(args=None):
    network_interface = os.environ.get('NETWORK_INTERFACE', 'offline')
    is_offline = (network_interface.lower() == 'offline')

    if not is_offline:
        if SDK_AVAILABLE:
            print(f"[Nav2Executor] Initializing Unitree SDK on interface: {network_interface}")
            ChannelFactoryInitialize(0, network_interface)
        else:
            print("FATAL: Not in offline mode, but Unitree SDK is not installed.")
            is_offline = True

    rclpy.init(args=args)
    node = Nav2Executor(is_offline=is_offline)

    try:
        rclpy.spin(node)
    finally:
        node.shutdown_clients()
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()