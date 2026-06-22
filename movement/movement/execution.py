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

            self.get_logger().info("Initializing robot posture... Standing up.")
            self.sport_client.StandUp()
            time.sleep(2.0)
            
            # BalanceStand is generally safer for Move() tracking on Go2 than FreeWalk
            self.get_logger().info("Engaging Balance mode to unlock velocity tracking...")
            self.sport_client.BalanceStand()
            time.sleep(1.0)
        else:
            self.get_logger().warn("OFFLINE MODE: Hardware clients bypassed.")

        # Ensure compatibility with Nav2's BEST_EFFORT cmd_vel publishers
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.estop_sub = self.create_subscription(Empty, '/emergency_stop', self.estop_callback, 10)
        
        self.last_cmd_time = time.time()
        self.first_message_received = False
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)

    def estop_callback(self, msg):
        self.get_logger().error("Emergency Stop received! Halting execution node.")
        self.shutdown_clients()
        time.sleep(0.2)
        os._exit(0)

    def cmd_vel_callback(self, msg: Twist):
        if not self.first_message_received:
            self.get_logger().info(">>> SUCCESS: First /cmd_vel message received from Nav2! <<<")
            self.first_message_received = True

        self.last_cmd_time = time.time()
        
        vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.angular.z))

        if not self.is_offline:
            # Send continuous velocity limits to the hardware
            self.sport_client.Move(vx, vy, yaw_rate)

    def watchdog_check(self):
        """Halts the robot if no velocity commands are received for 0.5 seconds."""
        if time.time() - self.last_cmd_time > 0.5:
            if not self.is_offline:
                # Do NOT use StopMove() here, it breaks the walking state machine.
                # Instead, command zero velocity to halt safely while ready to resume.
                self.sport_client.Move(0.0, 0.0, 0.0)

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
            try:
                print(f"[Nav2Executor] Initializing Unitree SDK on interface: {network_interface}")
                # Protected against hard crashes if the interface goes down during boot
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