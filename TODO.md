- Search for algorithm, which minimizes difference between optitrack and and Point-LIO
- Record IMU+LiDAR and Mocap into rosbag
    ros2 bag record /utlidar/imu /utlidar/cloud /mocap_path
- Set a running sum for the area between ground-truth and computed path
- Automate the resetting of the parameter, launching the algorithm, playing the rosbag and evaluating the sum