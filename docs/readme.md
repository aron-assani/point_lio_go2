# Point-LIO Go2

<div align="center">
  <img src="map.png" width="60%">
  <br>
</div>

A ROS 2 workspace integrating LiDAR-inertial mapping, autonomous navigation, and helper packages adapted for the Unitree Go2 robot using its Unilidar sensor. 

This repository heavily adapts the original Point-LIO algorithm and integrates it with a Nav2 stack operating in a rolling-window costmap mode, without a static map.

## Overview of Packages
- **`point_lio`**: C++ SLAM/LIO module based on Iterated Kalman Filters on Manifolds (IKFoM) and incremental kd-trees
- **`transform_sensors`**: Python node for external calibration transformations, IMU bias correction, time-synchronization, and pointcloud filtering
- **`trajectory_bridge`**: Python utilities to bridge SLAM odometry with OptiTrack motion capture systems for ground-truth alignment and evaluation
- **`movement`**: Runtime execution scripts translating Nav2 velocity commands into Unitree SportClient API instructions, with a terminal-based E-STOP
- **`unitree_sdk2_python`**: Official Unitree robot SDK

### Modifications to Original Point-LIO
- reparameterized YAML configurations for Go2
- custom RViz configuration
- rewritten launch files to include the necessary nodes
- wwitched to using the Go2-specific `utlidar/` topic instead of the default `unilidar/` topic

---

## System Architecture

```mermaid
flowchart TB
  subgraph RawSensors[Raw Sensors]
    UL[Unilidar / Livox driver]
  end

  subgraph Preproc[Preprocessing]
    TS[transform_sensors/transform_everything]
  end

  subgraph SLAM[Point-LIO]
    PL[point_lio/pointlio_mapping]
  end

  subgraph Navigation[Navigation Stack]
    NV[Nav2 Planner Controller]
  end

  subgraph Post[Post-processing]
    TR[trajectory_bridge/trajectory_node]
  end

  subgraph Control[Control]
    EX[movement/execution]
    KE[movement/keyboard_estop]
  end

  subgraph Viz[Visualization]
    RV[RViz]
  end

  UL -->|/utlidar/cloud| TS
  UL -->|/utlidar/imu| TS

  TS -->|/sensors/utlidar/processed/lidar_scan| PL
  TS -->|/sensors/utlidar/processed/imu| PL

  PL -->|/slam/odometry| TR
  PL -->|/robot/path_slam| TR

  TR -->|/robot/pose_estimate| NV
  TR -->|/robot/pose_estimate| RV
  TR -->|/robot/path_mocap| RV

  RV -->|/goal_pose| NV
  NV -->|/cmd_vel| EX
  
  KE -->|/emergency_stop| EX

  EX -->|Unitree SportClient API| Hardware[(Go2 robot)]
```

---

## Prerequisites
- **OS:** Linux (Ubuntu), RViz forwarding strictly requires an X11 display server environment on the host
- **Dependencies:** 
  - Git
  - Docker and the Docker Compose plugin

---

## Installation and Build

The entire pipeline runs inside a containerized Docker environment. The container requires host network access to communicate with the LiDAR and robot hardware.

**1. Clone the Repository**
```bash
git clone https://github.com/aron-assani/point_lio_go2.git
cd point_lio_go2
```

**2. Build the Docker Image**
```bash
# Allow Docker to connect to the host's X11 server for RViz
xhost +local:docker 

docker compose build \
  --build-arg NETWORK_INTERFACE=enx00133b9a06ef \
  --no-cache
```
*Note: Adjust the `NETWORK_INTERFACE` to match your hardware (e.g., your USB-Ethernet adapter).*

**3. Start the Workspace**
```bash
docker compose up -d
docker exec -it point_lio_go2 /bin/bash
```

---

## Usage

You can dynamically override the network interface at runtime by prefixing `NETWORK_INTERFACE=<interface>` to your launch commands. 

### Mapping and Navigation Modes

**Online Mode (With OptiTrack Mocap):**
*Runs SLAM, Nav2, and aligns the Mocap trajectory for evaluation.*
```bash
NETWORK_INTERFACE=enx00133b9a06ef ros2 launch point_lio mapping_utlidar.launch enable_navigation:=true enable_optitrack:=true
```

**Online Mode (No Mocap):**
*Standard autonomous navigation.*
```bash
NETWORK_INTERFACE=enx00133b9a06ef ros2 launch point_lio mapping_utlidar.launch enable_navigation:=true enable_optitrack:=false use_sim_time:=false
```

**Offline Mode (Rosbag Replay):**
*Processes pre-recorded rosbags using simulated time.*
```bash
NETWORK_INTERFACE=offline ros2 launch point_lio mapping_utlidar.launch enable_navigation:=true use_sim_time:=true
```

### Controlling the Robot

**Sending a Goal Pose:**
To send the robot to a target 1 meter ahead of the `camera_init` frame via Nav2:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'camera_init'}, 
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0}, 
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Emergency Stop (E-STOP):**
In a new terminal attached to the container, run the keyboard estop. Press `p` to immediately halt movement.
```bash
ros2 run movement keyboard_estop
```

---

## SLAM Odometry vs Mocap Ground Truth

<div align="center">
  <img src="trajectories.png" width="60%">
  <br>
  <img src="performance.png" width="60%">
</div>

---

## References
- **Point-LIO**: [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) | [Paper (Wiley)](https://advanced.onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)
- **IKFoM**: [hku-mars/IKFoM](https://github.com/hku-mars/IKFoM)
- **Nav2**: [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)
- **Point-LIO ROS2 Port**: [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2)
- **Autonomy Stack Go2 (transform_sensors)**: [jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)
