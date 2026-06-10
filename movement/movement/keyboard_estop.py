import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class KeyboardEStop(Node):
    def __init__(self):
        super().__init__('keyboard_estop')
        self.publisher_ = self.create_publisher(Empty, '/emergency_stop', 10)
        
        # Safety check: Prevent running via ros2 launch
        if not sys.stdin.isatty():
            self.get_logger().fatal("This node MUST be run in a dedicated terminal")
            sys.exit(1)

        self.get_logger().info("=====================================")
        self.get_logger().info(" EMERGENCY STOP NODE ACTIVE")
        self.get_logger().info(" Press 'P' at any time to halt robot.")
        self.get_logger().info("=====================================")

    def run_monitor(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'p':
                        self.get_logger().warn(">>> E-STOP TRIGGERED! Broadcasting to network... <<<")
                        msg = Empty()
                        for _ in range(3):
                            self.publisher_.publish(msg)
                            
        except KeyboardInterrupt:
            pass # Handle Ctrl+C cleanly
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardEStop()
    node.run_monitor()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()