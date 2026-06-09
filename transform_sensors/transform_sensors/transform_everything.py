#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Vector3
import sensor_msgs_py.point_cloud2 as pc2

from scipy.spatial.transform import Rotation as R

import numpy as np
import yaml
import os

class Repuber(Node):
    def __init__(self):
        super().__init__('sensor_transformer')
        self.imu_sub = self.create_subscription(Imu, '/utlidar/imu', self.imu_callback, 50)
        self.cloud_sub = self.create_subscription(PointCloud2, '/utlidar/cloud', self.cloud_callback, 50)
        
        self.imu_pub = self.create_publisher(Imu, '/sensors/utlidar/processed/imu', 50)
        self.cloud_pub = self.create_publisher(PointCloud2, '/sensors/utlidar/processed/lidar_scan', 50)
        
        self.time_stamp_offset = 0
        self.time_stamp_offset_set = False
        
        self.cam_offset = 0.046825

        # Load calibration data
        calib_data = {
                'acc_bias_x': -0.824918,
                'acc_bias_y': 1.82014,
                'acc_bias_z': -0.278397,
                'ang_bias_x': -0.00289323,
                'ang_bias_y': 0.000271719,
                'ang_bias_z': -0.000959372,
                'ang_z2x_proj': 0.135082,
                'ang_z2y_proj': -0.192149
            }
        try:
            home_path = os.path.expanduser('~')
            calib_file_path = os.path.join(home_path, 'Desktop/imu_calib_data.yaml')
            calib_file = open(calib_file_path, 'r')
            calib_data = yaml.load(calib_file, Loader=yaml.FullLoader)
            print("imu_calib.yaml loaded")
            calib_file.close()
        except:
            print("imu_calib.yaml not found, using default values")
            
        self.acc_bias_x = calib_data['acc_bias_x']
        self.acc_bias_y = calib_data['acc_bias_y']
        self.acc_bias_z = calib_data['acc_bias_z']
        self.ang_bias_x = calib_data['ang_bias_x']
        self.ang_bias_y = calib_data['ang_bias_y']
        self.ang_bias_z = calib_data['ang_bias_z']
        self.ang_z2x_proj = calib_data['ang_z2x_proj']
        self.ang_z2y_proj = calib_data['ang_z2y_proj']
                
        self.body2cloud_trans = TransformStamped()
        self.body2cloud_trans.header.stamp = self.get_clock().now().to_msg()
        self.body2cloud_trans.header.frame_id = "body"
        self.body2cloud_trans.child_frame_id = "utlidar_lidar_1"
        self.body2cloud_trans.transform.translation.x = 0.0
        self.body2cloud_trans.transform.translation.y = 0.0
        self.body2cloud_trans.transform.translation.z = 0.0
        quat = R.from_euler('xyz', [0, 2.87820258505555555556, 0]).as_quat()
        self.body2cloud_trans.transform.rotation.x = quat[0]
        self.body2cloud_trans.transform.rotation.y = quat[1]
        self.body2cloud_trans.transform.rotation.z = quat[2]
        self.body2cloud_trans.transform.rotation.w = quat[3]
        
        self.body2imu_trans = TransformStamped()
        self.body2imu_trans.header.stamp = self.get_clock().now().to_msg()
        self.body2imu_trans.header.frame_id = "body"
        self.body2imu_trans.child_frame_id = "utlidar_imu_1"
        self.body2imu_trans.transform.translation.x = 0.0
        self.body2imu_trans.transform.translation.y = 0.0
        self.body2imu_trans.transform.translation.z = 0.0
        quat2 = R.from_euler('xyz', [0, 2.87820258505555555556, 3.14159265358]).as_quat()
        self.body2imu_trans.transform.rotation.x = quat2[0]
        self.body2imu_trans.transform.rotation.y = quat2[1]
        self.body2imu_trans.transform.rotation.z = quat2[2]
        self.body2imu_trans.transform.rotation.w = quat2[3]
        
        self.x_filter_min = -0.7
        self.x_filter_max = -0.1
        self.y_filter_min = -0.3
        self.y_filter_max = 0.3
        self.z_filter_min = -0.6 - self.cam_offset
        self.z_filter_max = 0 - self.cam_offset

    def is_in_filter_box(self, point):
        return (point[0] > self.x_filter_min and point[0] < self.x_filter_max and 
                point[1] > self.y_filter_min and point[1] < self.y_filter_max and 
                point[2] > self.z_filter_min and point[2] < self.z_filter_max)

    def cloud_callback(self, data):
        if not self.time_stamp_offset_set:
            self.time_stamp_offset = self.get_clock().now().nanoseconds - Time.from_msg(data.header.stamp).nanoseconds
            self.time_stamp_offset_set = True
                
        cloud_arr = pc2.read_points_list(data)
        points = np.array(cloud_arr)

        transform = self.body2cloud_trans.transform
        mat = R.from_quat([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]).as_matrix()
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        
        transformed_points = points
        transformed_points[:, 0:3] = points[:, 0:3] @ mat.T + translation
        transformed_points[:, 2] -= self.cam_offset
        
        remove_list = []
        transformed_points = transformed_points.tolist()
        for i in range(len(transformed_points)):
            transformed_points[i][4] = int(transformed_points[i][4])
            if self.is_in_filter_box(transformed_points[i]):
                remove_list.append(i)

        remove_list.sort(reverse=True)
        for id_to_remove in remove_list:
            del transformed_points[id_to_remove]
        
        elevated_cloud = pc2.create_cloud(data.header, data.fields, transformed_points)
        elevated_cloud.header.stamp = Time(nanoseconds=Time.from_msg(elevated_cloud.header.stamp).nanoseconds + self.time_stamp_offset).to_msg()
        elevated_cloud.header.frame_id = "body"
        elevated_cloud.is_dense = data.is_dense

        self.cloud_pub.publish(elevated_cloud)

    def ensure_time_stamp_offset(self, stamp):
        if not self.time_stamp_offset_set:
            self.time_stamp_offset = self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds
            self.time_stamp_offset_set = True

    def imu_callback(self, data):    
        if not self.time_stamp_offset_set:
            self.ensure_time_stamp_offset(data.header.stamp)
        
        rot = [
            self.body2imu_trans.transform.rotation.x,
            self.body2imu_trans.transform.rotation.y,
            self.body2imu_trans.transform.rotation.z,
            self.body2imu_trans.transform.rotation.w
        ]
        
        imu_rot = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        transformed_orientation = (R.from_quat(rot) * R.from_quat(imu_rot)).as_quat()
        
        x = data.angular_velocity.x
        y = -data.angular_velocity.y
        z = -data.angular_velocity.z
        
        theta = 15.1 / 180 * 3.1415926

        x2 = np.cos(theta) * x - np.sin(theta) * z
        y2 = y
        z2 = np.sin(theta) * x + np.cos(theta) * z

        x2 -= self.ang_bias_x
        y2 -= self.ang_bias_y
        z2 -= self.ang_bias_z
        
        x2 += self.ang_z2x_proj * z2
        y2 += self.ang_z2y_proj * z2
        
        transformed_angular_velocity = Vector3()
        transformed_angular_velocity.x = x2
        transformed_angular_velocity.y = y2
        transformed_angular_velocity.z = z2
        
        acc_x = data.linear_acceleration.x
        acc_y = -data.linear_acceleration.y
        acc_z = -data.linear_acceleration.z
        
        acc_x2 = np.cos(theta) * acc_x - np.sin(theta) * acc_z
        acc_y2 = acc_y
        acc_z2 = np.sin(theta) * acc_x + np.cos(theta) * acc_z
        transformed_linear_acceleration = Vector3()
        transformed_linear_acceleration.x = acc_x2 - self.acc_bias_x
        transformed_linear_acceleration.y = acc_y2 - self.acc_bias_y
        transformed_linear_acceleration.z = acc_z2 - self.acc_bias_z
        
        transformed_imu = Imu()
        transformed_imu.header.stamp = data.header.stamp
        transformed_imu.header.frame_id = 'body'
        transformed_imu.orientation.x = transformed_orientation[0]
        transformed_imu.orientation.y = transformed_orientation[1]
        transformed_imu.orientation.z = transformed_orientation[2]
        transformed_imu.orientation.w = transformed_orientation[3]
        transformed_imu.angular_velocity = transformed_angular_velocity
        transformed_imu.linear_acceleration = transformed_linear_acceleration
        
        transformed_imu.header.stamp = Time(nanoseconds=Time.from_msg(transformed_imu.header.stamp).nanoseconds + self.time_stamp_offset).to_msg()
        
        self.imu_pub.publish(transformed_imu)

def main(args=None):
    rclpy.init(args=args)
    transform_node = Repuber()
    rclpy.spin(transform_node)
    transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()