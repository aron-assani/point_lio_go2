import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class BodyTransformer(Node):
    def __init__(self):
        super().__init__('body_transformer_node')

        # --- CONFIGURATION ---
        # Update this to match your Point-LIO output topic (e.g., '/state_estimation' or '/aft_mapped_to_init')
        self.point_lio_odom_topic = '/state_estimation' 
        self.body_odom_topic = '/body_odom'
        self.body_path_topic = '/body_path'
        self.fixed_frame = 'camera_init'
        
        # Local offset from Sensor to Body Center
        # Sensor is +0.2894 m X and -0.0468 m Z from the body center.
        # Therefore, the body center is -0.2894 m X and +0.0468m Z from the sensor.
        self.local_offset = np.array([-0.2894, 0.0, 0.0468])

        self.max_poses = 5000

        self.odom_sub = self.create_subscription(Odometry, self.point_lio_odom_topic, self.odom_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.body_odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.body_path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.fixed_frame
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        quat_sensor = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        # 2. Rotate the local offset into the global frame
        rot = R.from_quat(quat_sensor)
        global_offset = rot.apply(self.local_offset)

        # 3. Calculate actual body position
        pos_body = pos_sensor + global_offset

        # 4. Create new Odometry message
        body_odom = Odometry()
        body_odom.header.stamp = msg.header.stamp
        body_odom.header.frame_id = self.fixed_frame
        body_odom.child_frame_id = 'body_center'

        body_odom.pose.pose.position.x = float(pos_body[0])
        body_odom.pose.pose.position.y = float(pos_body[1])
        body_odom.pose.pose.position.z = float(pos_body[2])

        # Orientation remains identical because Point-LIO corrected the sensor tilt
        body_odom.pose.pose.orientation = msg.pose.pose.orientation

        # 5. Publish Odometry
        self.odom_pub.publish(body_odom)

        # 6. Update and Publish Path for RViz
        pose_stamped = PoseStamped()
        pose_stamped.header = body_odom.header
        pose_stamped.pose = body_odom.pose.pose

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose_stamped)

        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BodyTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

