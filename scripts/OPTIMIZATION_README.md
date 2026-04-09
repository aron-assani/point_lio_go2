# Point-LIO Parameter Optimization

Automated optimization tool for tuning Point-LIO parameters against ground-truth mocap data using rosbag recordings.

## Overview

This tool automatically:
1. **Modifies parameters** in `utlidar.yaml` 
2. **Replays rosbag** data containing IMU, LiDAR, and mocap odometry
3. **Runs Point-LIO pipeline** with modified parameters
4. **Collects trajectories** from both Point-LIO and mocap system
5. **Computes error** (ATE - Absolute Trajectory Error)
6. **Optimizes parameters** to minimize this error using differential evolution

## Setup

### Prerequisites

The Docker image is already configured. If running locally, install:

```bash
pip install pyyaml numpy scipy
```

### Rosbag Preparation

Your rosbag should contain:
- `/utlidar/imu` - IMU measurements  
- `/utlidar/cloud` - LiDAR point cloud
- `/mocap_path` - Ground truth odometry from mocap system (or use the mocap connection directly)

## Usage

### Option 1: Using Docker

```bash
cd /root/ros2_ws/src/point_lio

# Run optimizer
python3 scripts/parameter_optimizer.py /path/to/your/rosbag.db3

# Or with custom config path
python3 scripts/parameter_optimizer.py /path/to/rosbag.db3 /custom/config.yaml
```

### Option 2: Using wrapper script

```bash
bash scripts/run_optimizer.sh /path/to/your/rosbag.db3
```

## Optimization Process

### Algorithm: Differential Evolution

- **Type**: Global optimization algorithm (finds global optimum, not just local)
- **Advantage**: Robust, handles non-convex spaces well, good for 8+ parameters
- **Convergence**: ~50 iterations (50 full rosbag replays) for good results

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

## Output

After optimization completes:

```
============================================================
Optimization Complete!
Best ATE: 0.123456 m
Best Parameters:
  imu_meas_acc_cov: 12.345678
  imu_meas_omg_cov: 6.789012
  ...
============================================================
```

- **Best parameters** are saved to your config file
- **Backup** created as `utlidar.yaml.backup`
- **Trajectory logs** stored in `/root/ros2_ws/`:
  - `slam_trajectory.txt` - Point-LIO estimates
  - `mocap_trajectory.txt` - Ground truth trajectories

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

### "Empty trajectory files"
- Ensure mocap system is properly connected
- Check rosbag contains mocap data
- Verify mocap_odometry node starts correctly

### "Timeout waiting for trajectory files"  
- Increase rosbag playback speed (default: real-time)
- Check timestamps in rosbag aren't too old
- Verify Point-LIO is detecting features

### High ATE even after optimization
- Check rosbag contains diverse motion (linear + rotational)
- Verify ground truth mocap calibration
- Consider optimizing different parameter subset

## Files

- `parameter_optimizer.py` - Main optimization script
- `run_optimizer.sh` - Bash wrapper for convenience  
- `OPTIMIZATION_README.md` - This file

## References

- Point-LIO: https://github.com/ZikangYuan/point_lio
- Differential Evolution: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html
- ATE metric: https://vision.in.tum.de/data/datasets/rgbd-dataset/download
