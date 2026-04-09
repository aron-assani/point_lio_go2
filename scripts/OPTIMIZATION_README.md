# Point-LIO Parameter Optimization

Automated optimization tool for tuning Point-LIO parameters against ground-truth mocap data using rosbag recordings.

## Overview

This tool automatically:
1. **Records rosbag** data containing IMU, LiDAR, and mocap trajectory
2. **Modifies parameters** in `utlidar.yaml` 
3. **Replays rosbag** with trajectory listener capturing both SLAM and mocap paths
4. **Runs Point-LIO pipeline** with modified parameters
5. **Computes error** (ATE - Absolute Trajectory Error between SLAM and mocap)
6. **Optimizes parameters** to minimize this error using differential evolution algorithm

## Setup

### Prerequisites

The Docker image is already configured. If running locally, install:

```bash
pip install pyyaml numpy scipy
```

### Rosbag Preparation

Your rosbag **must** contain:
- `/utlidar/imu` - IMU measurements from Go2
- `/utlidar/cloud` - LiDAR point cloud from Go2 (or Unilidar)
- `/mocap_path` - Ground truth trajectory from mocap system (as a `nav_msgs/Path` message)

**Important**: The rosbag must include the mocap trajectory data recorded during the same experiment. This is published by `mocap_odometry/trajectory_node` as `/mocap_path`.

### Recording your rosbag

In one terminal, start the nodes:
```bash
ros2 launch point_lio mapping_utlidar.launch
ros2 run mocap_odometry trajectory_node
```

In another terminal, start recording:
```bash
ros2 bag record /utlidar/imu /utlidar/cloud /mocap_path -o my_experiment.db3
```

Move the robot around to collect diverse motion data (linear + rotational). Stop recording when done (Ctrl+C).

## Usage

### Basic Command

```bash
cd /root/ros2_ws
python3 src/point_lio_go2/scripts/parameter_optimizer.py /path/to/your/rosbag.db3
```

### With Early Termination Threshold

To skip iterations when error is too large (saves time):

```bash
python3 src/point_lio_go2/scripts/parameter_optimizer.py /path/to/rosbag.db3 --threshold 1.5
```

This will reject any parameter set with ATE > 1.5 meters and move to next iteration immediately.

### Custom Config Path

```bash
python3 src/point_lio_go2/scripts/parameter_optimizer.py /path/to/rosbag.db3 /custom/config.yaml
```

## How It Works

1. **Trajectory Listener** subscribes to:
   - `/Odometry` - Point-LIO's pose estimates
   - `/mocap_path` - Ground truth from mocap (recorded in rosbag)

2. **Rosbag playback** provides the sensor data and mocap truth

3. **Point-LIO** runs in real-time on the replayed sensor data

4. **Trajectories** are saved to text files as data arrives

5. **ATE is computed** by comparing aligned trajectories

6. **Optimizer** tests new parameters and selects best ones

### Optimizable Parameters

The tool optimizes these Kalman filter parameters:

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `imu_meas_acc_cov` | 10.0 | 1.0-50.0 | IMU accelerometer measurement variance |
| `imu_meas_omg_cov` | 5.0 | 0.5-20.0 | IMU gyroscope measurement variance |
| `lidar_meas_cov` | 0.01 | 0.001-0.1 | LiDAR measurement variance |
| `acc_cov_input` | 2.0 | 0.5-10.0 | Acceleration input covariance |
| `gyr_cov_input` | 0.1 | 0.05-2.0 | Gyroscope input covariance |
| `plane_thr` | 0.1 | 0.05-0.5 | Plane feature threshold (smaller = flatter) |
| `b_acc_cov` | 0.01 | 0.001-0.1 | Accelerometer bias covariance |
| `b_gyr_cov` | 0.01 | 0.001-0.1 | Gyroscope bias covariance |

**To optimize different parameters**: Edit `param_bounds` in `parameter_optimizer.py`

## Optimization Algorithm

### Differential Evolution Details

- **Type**: Global optimization algorithm (finds global optimum, not local minima)
- **Advantage**: Very robust for high-dimensional non-convex parameter spaces
- **Typical iterations**: ~50 full rosbag replays
- **Iteration time**: ~2.5 minutes each (allows time for Point-LIO startup and data capture)

### What Happens During Each Iteration

1. Temporary config file created with new parameter values
2. RViz windows from previous iteration cleaned up
3. Trajectory listener node started (subscribes to `/Odometry` and `/mocap_path`)
4. Rosbag playback begins (loops continuously)
5. Point-LIO launched with temporary config (launches RViz visualization)
6. Trajectories automatically captured to text files as messages arrive
7. After timeout or sufficient data (>500 bytes each): ATE computed
8. Best parameters tracked across all iterations
9. Processes cleaned up, ready for next iteration

## Output

### Final Results

After optimization completes, you'll see:

```
============================================================
Optimization Complete!
Best ATE: 0.123456 m
Best Parameters:
  imu_meas_acc_cov: 12.345678
  imu_meas_omg_cov: 6.789012
  lidar_meas_cov: 0.025
  ...
============================================================
```

### Trajectory Log Files

Saved in `/root/ros2_ws/`:
- `slam_trajectory.txt` - Point-LIO's trajectory estimates
- `mocap_trajectory.txt` - Ground truth mocap trajectory

Format of each file: `timestamp x y z qx qy qz qw` (space-separated)

The ATE (Absolute Trajectory Error) is the RMS of position differences between these trajectories.

### Iteration Progress

During optimization, you'll see messages for each iteration:
```
[Iteration 1] Testing parameters:
  acc_cov_input: 3.352957
  imu_meas_acc_cov: 6.113449
  ...
  Starting trajectory listener...
  Starting rosbag playback...
  Starting Point-LIO...
  Waiting for trajectory data...
  ✓ Trajectories recorded (1250 + 1180 bytes)
  ✓ ATE: 0.456789 m
  ✨ New best ATE!
```

## Performance Tips

1. **Faster optimization**:
   - Use shorter rosbag (30-60 seconds is usually enough)
   - Reduce `max_eval` parameter in the script
   
2. **Better results**:
   - Use longer rosbag with diverse motion
   - Run optimization multiple times with different rosbags
   
3. **Debugging**:
   - Check trajectory files for obviously wrong data
   - Verify rosbag plays correctly: `ros2 bag play your_rosbag.db3 --loop`
   - Monitor with: `ros2 launch point_lio mapping_utlidar.launch`

## Customization

### Optimize Different Parameters

Edit `parameter_optimizer.py` and modify the `param_bounds` dictionary:

```python
self.param_bounds = {
    'your_param_name': (min_value, max_value),
    # Add more...
}
```

### Use Local Optimization Instead

Replace `differential_evolution` call with:

```python
optimizer.optimize(method='nelder-mead', max_eval=50)
```

This is faster but may get stuck in local minima.

### Manual Trajectory Analysis

```python
from parameter_optimizer import ParameterOptimizer

opt = ParameterOptimizer('rosbag.db3', 'config.yaml')
ate = opt.compute_trajectory_error('slam_trajectory.txt', 'mocap_trajectory.txt')
print(f"ATE: {ate} m")
```

## Troubleshooting

### "Timeout waiting for trajectory files"

**Cause**: Files not being created or not accumulating data fast enough.

**Solutions**:
- Verify rosbag is being played: `ros2 bag play your_rosbag.db3 --loop` (in separate terminal)
- Check if `/Odometry` topic is being published by Point-LIO: `ros2 topic echo /Odometry`
- Check if `/mocap_path` is in your rosbag: `ros2 bag info your_rosbag.db3`
- Increase timeout in `parameter_optimizer.py`: Change `timeout = 150` to `200` or higher

### "Trajectory files are empty or too small"

**Cause**: Nodes not running long enough or not publishing data.

**Solutions**:
- Ensure rosbag has sufficient data (30+ seconds recommended)
- Verify Point-LIO is actually publishing: Check `/Odometry` topic in separate terminal
- Check rosbag contains all necessary topics:
  ```bash
  ros2 bag info your_rosbag.db3 | grep -E "Odometry|cloud|imu|mocap"
  ```

### "Point-LIO not publishing /Odometry"

**Cause**: Point-LIO not finding enough features or configuration issue.

**Solutions**:
- Verify LiDAR cloud topic names match config (`/utlidar/cloud`)
- Check IMU topic matches (`/utlidar/imu`)
- Verify rosbag was recorded with robot in motion (not stationary)
- Check Point-LIO can run normally: `ros2 launch point_lio mapping_utlidar.launch`

### "High ATE even after optimization"

**Cause**: Poor parameter search space or issue with data/calibration.

**Solutions**:
- Expand parameter bounds in `param_bounds` dictionary
- Verify mocap calibration is accurate
- Use rosbag with more diverse motion (linear + rotational + elevation changes)
- Check that mocap_path timestamps align with IMU/LiDAR timestamps
- Manually verify trajectory file contents: `head slam_trajectory.txt`

### "ModuleNotFoundError: No module named 'scipy'"

**Cause**: scipy not installed.

**Solution**:
```bash
pip3 install scipy
```

## Files

- `parameter_optimizer.py` - Main optimization script
- `run_optimizer.sh` - Bash wrapper for convenience  
- `OPTIMIZATION_README.md` - This file

## References

- Point-LIO: https://github.com/ZikangYuan/point_lio
- Differential Evolution: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html
- ATE metric: https://vision.in.tum.de/data/datasets/rgbd-dataset/download
