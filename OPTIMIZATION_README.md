# Parameter Optimization Framework for Point-LIO UTLiDAR

This framework provides **Bayesian optimization with early stopping** to automatically find the best parameters for the `utlidar.yaml` configuration to minimize **Absolute Trajectory Error (ATE)** against ground truth mocap data.

## 📋 Overview

### What This Does

1. **Plays a rosbag file** containing LiDAR, IMU, and MoCap truth data
2. **Runs Point-LIO** with different parameter combinations
3. **Collects trajectories** from both Point-LIO (`/Odometry`) and MoCap (`/mocap_path`)
4. **Aligns trajectories** and computes ATE (Absolute Trajectory Error)
5. **Optimizes parameters** using Bayesian optimization (Optuna)
6. **Stops early** when no improvement is detected or target error is reached

### Why This Works

Point-LIO's performance heavily depends on the sensor covariance values and feature detection thresholds. This framework automates finding the optimal values for your specific sensor setup and environment.

---

## 🚀 Quick Start

### 1. Prerequisites

You need to be **inside the Docker container** with ROS 2 and Point-LIO set up:

```bash
# Inside Docker
which ros2           # Should work
ros2 --version       # Should show ROS2 version

# Install optimizer dependencies
pip install optuna scipy numpy pyyaml matplotlib tabulate
```

### 2. Basic Usage

Run optimization with default settings:

```bash
cd ~/point_lio_go2
python3 optimize_utlidar.py
```

This will:
- Use the rosbag at `tester_rosbag/rosbag2_2026_04_09-10_58_57/`
- Run up to 100 trials
- Save results to `tester_rosbag/optimization_results/`

### 3. Advanced Usage

```bash
# Specify rosbag and number of trials
python3 optimize_utlidar.py --rosbag /path/to/rosbag --trials 50

# Set target error threshold (stop when ATE reaches this)
python3 optimize_utlidar.py --target-ate 0.3

# Set optimization timeout (in seconds)
python3 optimize_utlidar.py --timeout 3600

# Save results to specific directory
python3 optimize_utlidar.py --output-dir /path/to/results
```

### 4. Analyze Results

After optimization completes:

```bash
# View all results
python3 analyze_optimization.py

# Show top 20 results
python3 analyze_optimization.py --top 20

# Plot convergence
python3 analyze_optimization.py --plot-convergence

# Plot parameter sensitivity (how each parameter affects ATE)
python3 analyze_optimization.py --plot-sensitivity

# Analyze which parameters impact ATE most
python3 analyze_optimization.py --analyze

# Run all analyses
python3 analyze_optimization.py --all
```

---

## 🔧 Parameters Being Optimized

| Parameter | Default Range | Impact | Description |
|-----------|--------------|--------|-------------|
| `lidar_meas_cov` | 0.001 - 0.1 | High | LiDAR measurement covariance (higher = trust LiDAR less) |
| `imu_meas_acc_cov` | 1.0 - 50.0 | High | IMU acceleration covariance |
| `imu_meas_omg_cov` | 1.0 - 20.0 | High | IMU gyro covariance |
| `plane_thr` | 0.05 - 0.5 | Medium | Plane detection threshold (lower = stricter) |
| `b_acc_cov` | 0.001 - 0.1 | Low | Acceleration bias covariance |
| `b_gyr_cov` | 0.001 - 0.1 | Low | Gyro bias covariance |
| `filter_size_surf` | 0.1 - 0.5 | Medium | Surface downsampling factor |
| `filter_size_map` | 0.1 - 0.5 | Medium | Map downsampling factor |

### Customizing Parameters

Edit `optimize_utlidar.py` to change parameters or bounds:

```python
PARAM_BOUNDS = {
    'your_param': (min_value, max_value),
    'another_param': (min_value, max_value),
}
```

---

## 📊 Understanding Results

### Output Structure

```
tester_rosbag/
└── optimization_results/
    ├── optimization_log.json    # All trial results
    ├── convergence.png          # Convergence plot
    ├── sensitivity.png          # Parameter sensitivity
    └── utlidar.yaml             # Best configuration found
```

### Interpreting Results

#### `optimization_log.json`
```json
[
  {
    "params": {
      "lidar_meas_cov": 0.015,
      "imu_meas_acc_cov": 15.3,
      ...
    },
    "ate": 0.234,           // Absolute Trajectory Error in meters
    "aligned_count": 456,   // Number of poses compared
    "timestamp": 1712234567.89
  },
  ...
]
```

#### Convergence Plot
- **Left:** Individual trial results (blue) and best ATE found so far (red)
- **Right:** Histogram showing distribution of all ATE values
- **Goal:** Red line should trend downward smoothly

#### Sensitivity Plot
- Shows relationship between each parameter and ATE
- Parameters with strong correlation have high impact
- Use this to understand which parameters matter most

---

## ⚡ Early Stopping Mechanisms

The optimizer stops when:

1. **Target ATE Reached**: If ATE drops below `target_ate_threshold` (default: 0.5m)
2. **Patience Exceeded**: If no improvement for `patience` iterations (default: 5)
3. **Max Trials Reached**: If total trials exceeds `max_trials` (default: 100)
4. **Timeout Reached**: If optimization runs longer than `timeout` seconds

Edit `OPTIMIZATION_CONFIG` in `optimize_utlidar.py` to change these:

```python
OPTIMIZATION_CONFIG = {
    'target_ate_threshold': 0.5,      # Stop if ATE < this
    'patience': 5,                     # Early stopping iterations
    'initial_trials': 20,              # Random trials before optimization
    'max_trials': 100,                 # Maximum iterations
}
```

---

## 🐛 Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Solution:** You're not inside the Docker container. The optimizer must run inside Docker where ROS 2 is installed.

### Issue: "Rosbag not found"

**Solution:** Specify the correct rosbag path:

```bash
python3 optimize_utlidar.py --rosbag /path/to/your/rosbag
```

### Issue: "ros2 bag play" hangs

**Solution:** The rosbag might be very large or corrupted. Check:

```bash
ros2 bag info /path/to/rosbag
```

### Issue: Optimizer produces very high ATE values

**Possible causes:**
- Point-LIO is not initializing properly
- The rosbag doesn't have the expected topics
- Parameter ranges are too extreme

**Solutions:**
- Check ROS 2 logs for Point-LIO errors
- Verify rosbag contains `/utlidar/imu`, `/utlidar/cloud`, `/mocap_path`
- Narrow parameter bounds

### Issue: Optimization is very slow

**Solutions:**
- Reduce `trajectory_collection_timeout` in code (currently 60s)
- Use fewer trials: `--trials 20`
- Use smaller rosbag or trim it to a specific time window

---

## 📐 Trajectory Alignment & ATE Calculation

The framework uses the following approach:

1. **Trajectory Alignment**
   - Uses first pose position as translation Reference
   - Computes rotation alignment from quaternions
   - Computes scale factor from total trajectory distance

2. **ATE Computation**
   ```
   ATE = sqrt(mean(||estimated_pos_i - truth_pos_i||^2))
   ```
   - For each estimated pose, finds nearest truth pose by timestamp
   - Computes Euclidean distance
   - Takes RMS of all distances

---

## 🎯 Tips for Best Results

1. **Use representative data**: Rosbag should include diverse motions (forward, turning, acceleration)

2. **Warm up ROS system**: Run a few manual tests before optimization to warm up the system

3. **Monitor disk space**: Each trial creates temporary files. Ensure you have several GB free

4. **Test sensitivity first**:
   ```bash
   python3 analyze_optimization.py --plot-sensitivity
   ```
   This shows which parameters actually matter for your setup

5. **Start conservative**: Use fewer trials first (--trials 20) to understand the optimization landscape

6. **Inspect outliers**: If some trials produce very high ATE, check logs:
   ```bash
   ros2 launch point_lio mapping_utlidar.launch
   ```

---

## 📈 Example Workflow

```bash
# 1. Run optimization
python3 optimize_utlidar.py --trials 50 --target-ate 0.3

# 2. After completion, analyze results
python3 analyze_optimization.py --all

# 3. View best parameters
tail -n 20 tester_rosbag/optimization_results/optimization_log.json

# 4. The best config is already saved to utlidar.yaml
# Deploy it to production:
cp point_lio/config/utlidar.yaml point_lio/config/utlidar.yaml.optimized
```

---

## 🔍 Advanced: Custom Trajectory Comparison

For debugging, use the standalone trajectory comparison tool:

```bash
# Extract trajectories from rosbag
python3 compare_trajectories.py --extract --rosbag /path/to/rosbag --output trajectories.json

# Compare extracted trajectories
python3 compare_trajectories.py --compare --input trajectories.json
```

---

## 📝 Configuration File Format

The optimizer modifies `point_lio/config/utlidar.yaml`. The structure:

```yaml
/**:
  ros__parameters:
    mapping:
      lidar_meas_cov: 0.01        # Optimizer changes these
      imu_meas_acc_cov: 10.0      # Optimizer changes these
      imu_meas_omg_cov: 5.0       # Optimizer changes these
      plane_thr: 0.1              # Optimizer changes these
      # ... other parameters
```

A backup is automatically created at `utlidar.yaml.bak` before optimization.

---

## 🚨 Important Notes

- **Destructive Operation**: The optimizer **modifies `utlidar.yaml`**. A backup is created, but ensure you have version control if needed.

- **Time-consuming**: One trial takes ~60-120 seconds depending on rosbag size. 50 trials = 1-2 hours.

- **Docker-only**: Must run inside Docker with ROS 2, Point-LIO, and rosbag available.

- **Idempotent**: Safe to run multiple times; each restart begins independently.

---

## 📞 Support

For issues or improvements, check:
- ROS 2 documentation: https://docs.ros.org
- Optuna documentation: https://optuna.readthedocs.io
- Point-LIO repository: https://github.com/KIT-ISAS/point_lio

---

**Last Updated:** April 2026
**Framework Version:** 1.0
**Tested With:** ROS 2 Humble, Point-LIO v2.0
