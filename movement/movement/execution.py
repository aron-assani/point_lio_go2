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
        self.sport_client = None
        
        if not self.is_offline and SDK_AVAILABLE:
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            
            self.get_logger().info("Initializing robot posture...")
            self.sport_client.RecoveryStand()
            time.sleep(3.0)
            self.sport_client.FreeWalk()
            time.sleep(1.0)
            self.get_logger().info("Execution Bridge Online. Listening for /cmd_vel.")
        else:
            self.get_logger().warn("OFFLINE MODE: Hardware clients bypassed.")

        # Best effort QoS to match the graph planner
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.estop_sub = self.create_subscription(Empty, '/emergency_stop', self.estop_callback, 10)
        
        self.last_cmd_time = time.time()
        self.last_move_time = 0.0
        self.is_moving = False 
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)

    def estop_callback(self, msg):
        self.get_logger().error("Emergency Stop received!")
        self.shutdown_clients()
        os._exit(0)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        
        # Velocity clamping to ensure robot stability
        vx = max(-0.3, min(0.3, msg.linear.x))
        vy = 0.0 
        yaw_rate = max(-0.5, min(0.5, msg.angular.z))

        # Stop command threshold
        if abs(vx) < 0.01 and abs(yaw_rate) < 0.01:
            if self.is_moving:
                if self.sport_client: self.sport_client.StopMove()
                self.is_moving = False
            return

        # 20Hz update rate matches standard ROS 2 controller frequencies
        if time.time() - self.last_move_time >= 0.05:
            if self.sport_client:
                self.sport_client.Move(vx, vy, yaw_rate)
            self.last_move_time = time.time()
            self.is_moving = True

    def watchdog_check(self):
        # Stop moving if planner stops sending commands (Safety feature)
        if time.time() - self.last_cmd_time > 0.5 and self.is_moving:
            if self.sport_client: self.sport_client.StopMove()
            self.is_moving = False

    def shutdown_clients(self):
        if self.sport_client:
            self.sport_client.StopMove()
            time.sleep(0.5)
            self.sport_client.StandDown()

def main(args=None):
    iface = os.environ.get('NETWORK_INTERFACE', 'offline')
    if iface != 'offline':
        ChannelFactoryInitialize(0, iface)
    
    rclpy.init(args=args)
    node = Nav2Executor(is_offline=(iface == 'offline'))
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_clients()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()