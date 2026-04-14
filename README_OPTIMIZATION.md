# Point-LIO Parameter Optimization

Automatically optimize `utlidar.yaml` parameters using rosbag playback and mocap ground truth.

## Quick Start

### Prerequisites

1. **Rosbag with Ground Truth**: Record data with both sensor topics AND mocap path:
   ```bash
   ros2 bag record /utlidar/imu /utlidar/cloud /mocap_path -o experiment.db3
   ```
   Record ~30-60 seconds of diverse motion (linear + rotational). The `/mocap_path` topic is essential—it provides the ground truth for ATE calculation.

2. **OptiTrack (Optional)**: If connected, `trajectory_node` uses it for live mocap alignment. If not available, it runs in SLAM-only mode (publishes `/body_path` only). Pre-recorded `/mocap_path` in the rosbag is what matters for optimization.

### Run Optimization

```bash
python3 optimize_utlidar.py --rosbag experiment.db3 --trials 50 --timeout 120
```

**Arguments:**
- `--rosbag`: Path to rosbag file (required)
- `--trials`: Maximum optimization iterations (default: 100)
- `--timeout`: Trajectory collection timeout in seconds (default: 60)

## How It Works

Each iteration:
1. **Start Point-LIO**: Launch `mapping_utlidar.launch`
2. **Start trajectory_node**: Publishes `/body_path` (SLAM estimate). If OptiTrack is connected, also attempts to publish `/mocap_path` (live mocap); if not connected, runs in SLAM-only mode.
3. **Play rosbag**: Replays recorded sensor data (`/utlidar/imu`, `/utlidar/cloud`) and `mocap_path` topic
4. **Compare trajectories**: Computes ATE between `/body_path` (estimate) and `/mocap_path` (ground truth, from rosbag)
5. **Optimize**: Uses Optuna to find parameter values that minimize ATE

## Optimized Parameters

| Parameter | Range | Effect |
|-----------|-------|--------|
| `lidar_meas_cov` | 0.001-0.1 | LiDAR measurement trust |
| `imu_meas_acc_cov` | 1.0-50.0 | IMU accelerometer trust |
| `imu_meas_omg_cov` | 1.0-20.0 | IMU gyroscope trust |
| `b_acc_cov` | 0.001-0.1 | Accel bias covariance |
| `b_gyr_cov` | 0.001-0.1 | Gyro bias covariance |
| `plane_thr` | 0.05-0.5 | Plane feature threshold |
| `filter_size_surf` | 0.1-0.5 | Surface filter size |
| `filter_size_map` | 0.1-0.5 | Map filter size |

## Key Features

- **Global Optimization**: Uses Optuna with median pruner for efficient search
- **Early Stopping**: Stops if ATE < 0.5m or no improvement for 5 trials
- **Progress Tracking**: Logs best parameters and ATE after each iteration
- **Automatic Cleanup**: Terminates all processes and ROS nodes

## Output

```
🚀 Starting Parameter Optimization
Trial 1: params={...} ATE=0.456 [0%]
Trial 2: params={...} ATE=0.234 [10%] ✨ New best!
...
✅ Optimization Complete!
Best ATE: 0.123 m
Best Parameters: 
  lidar_meas_cov: 0.025
  ...
```

Best parameters are saved back to `utlidar.yaml`.

## Troubleshooting

**"Failed to connect to OptiTrack"**
- This is normal. `trajectory_node` gracefully falls back to SLAM-only mode.
- Optimization will still work as long as `/mocap_path` is in the rosbag (recorded ground truth).

**"Timeout waiting for trajectories"**
- Increase `--timeout` value
- Verify rosbag plays correctly: `ros2 bag play experiment.db3 --loop`
- Check `/body_path` and `/mocap_path` topics exist during playback: `ros2 topic list` while rosbag is playing
- If `/mocap_path` is missing from the rosbag recording, add it: `ros2 bag record /utlidar/imu /utlidar/cloud /mocap_path -o experiment.db3`

**"Low ATE even after optimization"**
- Verify rosbag has diverse motion (not stationary)
- Check mocap calibration accuracy
- Increase `--trials` for more iterations

## References

- Point-LIO: https://github.com/ZikangYuan/point_lio
- Optuna: https://optuna.org/
- ATE metric: https://vision.in.tum.de/data/datasets/rgbd-dataset/download
