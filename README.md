# point_lio_go2

Lightweight workspace containing LiDAR-inertial mapping and helper packages.

Overview
- `point_lio`: C++ mapping/estimation code.
- `trajectory_bridge`: Python package for mocap-based odometry utilities.
- `transform_sensors`: Python utilities to transform sensor data and frames.
- ``unitree_sdk2_python`: Unitree robot SDK

Quick start
- Build with your ROS workspace (catkin) or preferred build system:

```bash
docker compose build --build-arg GIT_PAT=<GithubPath> --build-arg NETWORK_INTERFACE=<NetworkInterface>
docker compose up -d
docker exec -it point_lio_go2 /bin/bash
```

```bash
ros2 launch point_lio mapping_utlidar.launch
ros2 launch point_lio mapping_utlidar.launch enable_optitrack:=false
```