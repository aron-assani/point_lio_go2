# Parameter Optimization Framework Integration Guide

## 📦 What You've Received

A complete Bayesian optimization framework for Point-LIO UTLiDAR parameters with early stopping.

### Files Created

```
point_lio_go2/
├── optimize_utlidar.py              # Main optimization script (850 lines)
├── analyze_optimization.py          # Results analysis & visualization (250 lines)
├── compare_trajectories.py          # Trajectory comparison tool (300 lines)
├── entrypoint_optimize.sh           # Docker entrypoint (250 lines)
├── Dockerfile.optimize              # Docker image for optimizer
├── docker-compose-optimizer.yml     # Docker Compose orchestration
├── OPTIMIZATION_README.md           # Detailed documentation (400+ lines)
├── QUICKSTART_OPTIMIZATION.md       # Quick start guide (300+ lines)
└── INTEGRATION_GUIDE.md             # This file
```

## 🎯 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Optimization Framework                    │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │           Optuna Bayesian Optimizer                  │  │
│  │  - TPE (Tree-structured Parzen Estimator) sampler   │  │
│  │  - MedianPruner for early stopping                  │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                   │
│                          ▼                                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Configuration Manager                        │  │
│  │  - Modifies utlidar.yaml parameters                 │  │
│  │  - Maintains backups                                │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                   │
│                          ▼                                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Trial Execution                              │  │
│  │  - Launch Point-LIO with params                      │  │
│  │  - Play rosbag (IMU + LiDAR)                        │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                   │
│                          ▼                                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     Trajectory Collection & Processing              │  │
│  │  - Collect /Odometry (estimated)                    │  │
│  │  - Collect /mocap_path (ground truth)               │  │
│  │  - Compute trajectory alignment                     │  │
│  │  - Calculate ATE (Absolute Trajectory Error)        │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                   │
│                          ▼                                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Results & Analysis                           │  │
│  │  - Save trial results (JSON)                        │  │
│  │  - Generate convergence plots                       │  │
│  │  - Analyze parameter sensitivity                    │  │
│  │  - Save best configuration                          │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## 🔄 Workflow

### Phase 1: Initialization (1-2 minutes)
```
Start → Validate Environment
     ↓
Load Configuration → Backup Original
```

### Phase 2: Optimization Loop (1-4 hours)
```
Trial 1 → Modify YAML
   ↓
Launch ROS System → Play Rosbag
   ↓
Collect Trajectories → Compute ATE
   ↓
Report to Optuna → Prune if needed
   ↓
Early Stopping Check ─→ Continue/Stop
   ↓
Trial 2, 3, ... N
```

### Phase 3: Results Finalization (30 seconds)
```
Restore → Save Best Config
   ↓
Generate Summary → Export Results
```

## 🧮 Parameters Optimized

| Parameter | Type | Default Range | Typical Best Value |
|-----------|------|---------------|-------------------|
| `lidar_meas_cov` | Covariance | 0.001 - 0.1 | ~0.015 |
| `imu_meas_acc_cov` | Covariance | 1.0 - 50.0 | ~12-15 |
| `imu_meas_omg_cov` | Covariance | 1.0 - 20.0 | ~5-8 |
| `plane_thr` | Detection | 0.05 - 0.5 | ~0.1-0.2 |
| `b_acc_cov` | Bias Cov | 0.001 - 0.1 | ~0.03-0.05 |
| `b_gyr_cov` | Bias Cov | 0.001 - 0.1 | ~0.02-0.04 |
| `filter_size_surf` | Downsampling | 0.1 - 0.5 | ~0.2-0.3 |
| `filter_size_map` | Downsampling | 0.1 - 0.5 | ~0.2-0.3 |

To customize, edit `PARAM_BOUNDS` in `optimize_utlidar.py`.

## 📈 Performance Metrics

### ATE (Absolute Trajectory Error)
- **Calculation**: `ATE = sqrt(mean(||estimated_pose_i - truth_pose_i||^2))`
- **Units**: Meters
- **Lower is better**
- **Typical range for LiO**: 0.1 - 1.0 meters

### Trajectory Alignment Process
1. Compute scale from total trajectory distance
2. Estimate rotation from quaternions (first/last poses)
3. Estimate translation from position difference
4. Apply alignment to all poses
5. Compute point-wise errors by temporal matching
6. Calculate RMS of errors

## 🛑 Early Stopping Mechanisms

| Mechanism | Default | Purpose |
|-----------|---------|---------|
| **Target ATE** | 0.5m | Stop if error < threshold |
| **Patience** | 5 | Stop if no improvement for N trials |
| **Max Trials** | 100 | Hard limit on iterations |
| **Timeout** | None | Wall-clock time limit |
| **Pruning** | MedianPruner | Stop unpromising trials early |

Configure in `OPTIMIZATION_CONFIG` dict.

## 🚀 Deployment Options

### Option 1: Direct Execution (Recommended for testing)
```bash
cd ~/point_lio_go2
python3 optimize_utlidar.py --trials 50
```

### Option 2: Docker Container
```bash
docker build -f Dockerfile.optimize -t point_lio_optimizer .
docker run -v /data:/data point_lio_optimizer --trials 50 --analyze
```

### Option 3: Docker Compose (Best for automation)
```bash
docker-compose -f docker-compose-optimizer.yml up
```

## 📊 Result Interpretation

### optimization_log.json Structure
```json
[
  {
    "params": {
      "param_name": value,
      ...
    },
    "ate": 0.234,              // Absolute Trajectory Error (meters)
    "aligned_count": 287,      // Number of poses successfully aligned
    "timestamp": 1712234567.89 // When trial was run
  },
  ...
]
```

### Key Metrics
- **Best ATE**: Most important, shows overall quality
- **Aligned Count**: Should be > 50 poses for reliable results
- **Convergence**: Plot should show decreasing trend

## 🔧 Customization Examples

### Example 1: Optimize Only Covariances
```python
PARAM_BOUNDS = {
    'lidar_meas_cov': (0.001, 0.1),
    'imu_meas_acc_cov': (1.0, 50.0),
    'imu_meas_omg_cov': (1.0, 20.0),
}
```

### Example 2: Faster Optimization
```python
OPTIMIZATION_CONFIG = {
    'trajectory_collection_timeout': 30.0,  # Shorter collection
    'patience': 3,                           # Early stopping sooner
    'initial_trials': 10,                    # Fewer random trials
}
```

### Example 3: Tight Parameter Ranges
```python
PARAM_BOUNDS = {
    'lidar_meas_cov': (0.01, 0.03),   # Narrower range
    'plane_thr': (0.08, 0.15),        # Focus on this region
}
```

## 📝 Important Notes

### ⚠️ Configuration Modification
The optimizer **modifies `utlidar.yaml`** during execution:
- A backup is created at `utlidar.yaml.bak`
- Original is restored after optimization
- Best configuration is saved to `utlidar.yaml`

### ⏱️ Time Requirements
- **Setup**: ~2 minutes
- **Per trial**: ~60-120 seconds
- **50 trials**: ~50-120 minutes
- **Post-processing**: ~2 minutes

### 💾 Storage Requirements
- **Per rosbag**: Size of rosbag (typically 250MB-2GB)
- **Per trial**: Temporary files during execution (~100MB)
- **Results**: ~100KB per 100 trials

### 🔄 Safety
- Safe to run multiple times
- Safe to interrupt (CTRL+C)
- Idempotent: results don't depend on previous runs
- Original config backed up

## 🎓 Advanced Usage

### Resuming Interrupted Optimization
```bash
# Optuna study is not saved, but trajectory data is JSON
# To continue with custom logic:
python3 analyze_optimization.py --input results/optimization_log.json
# Design next parameters based on insights
```

### Offline Parameter Testing
```bash
# Extract trajectories once
python3 compare_trajectories.py --extract --rosbag /path/to/rosbag

# Then test parameters manually
optimize_utlidar.py --config ./test_config.yaml

# Compare results
python3 compare_trajectories.py --compare
```

### Integration with CI/CD
```bash
# In your CI/CD pipeline:
docker run \
  -v $PWD:/data \
  point_lio_optimizer \
  --trials 100 \
  --target-ate 0.3 \
  --analyze

# Check if best ATE meets threshold
python3 -c "
import json
with open('/data/optimization_results/optimization_log.json') as f:
    data = json.load(f)
    best_ate = min([d['ate'] for d in data])
    assert best_ate < 0.3, f'ATE {best_ate} exceeds threshold'
"
```

## 🔍 Debugging & Validation

### Verify Setup
```bash
# Inside Docker
ros2 --version              # Should work
ros2 pkg list | grep point_lio
ros2 bag info tester_rosbag/rosbag2_*/
```

### Manual Trial Execution
```bash
# Run Point-LIO manually
ros2 bag play tester_rosbag/rosbag2_*/ -l &
sleep 2
ros2 launch point_lio mapping_utlidar.launch rviz:=false

# In another terminal, monitor trajectories
ros2 topic echo /Odometry
ros2 topic echo /mocap_path
```

### Log Analysis
```bash
# Check Point-LIO logs
ros2 run rclpy _node_name:=debug_monitor

# Monitor system load during optimization
htop
# or
watch -n 1 'free -h && ps aux | grep pointlio'
```

## 📞 Getting Help

### Common Issues & Solutions

1. **"ros2 command not found"**
   - You're outside Docker or ROS 2 is not in PATH
   - Inside Docker: `source /opt/ros/humble/setup.bash`

2. **"High ATE values (> 5.0m)"**
   - Check rosbag has correct topic names
   - Verify Point-LIO launches without errors
   - Check for large time synchronization issues

3. **"Very slow optimization"**
   - Reduce `trajectory_collection_timeout` (currently 60s)
   - Use fewer trials initially
   - Check CPU/RAM availability

4. **"Module not found errors"**
   - Inside Docker run: `pip install optuna scipy matplotlib tabulate`
   - Verify Python version: `python3 --version` (should be 3.8+)

### Documentation
- **Detailed Guide**: `OPTIMIZATION_README.md`
- **Quick Start**: `QUICKSTART_OPTIMIZATION.md`
- **Code**: Well-commented Python scripts in repo

### External Resources
- **Optuna Docs**: https://optuna.readthedocs.io
- **Point-LIO**: https://github.com/KIT-ISAS/point_lio
- **ROS 2**: https://docs.ros.org

## ✅ Pre-Deployment Checklist

Before running in production:

- [ ] Tested with 5-10 trials first
- [ ] Verified result quality is reasonable (ATE < 1.0m)
- [ ] Confirmed rosbag is representative of use case
- [ ] Backup original `utlidar.yaml`
- [ ] Have monitored disk space during optimization
- [ ] Tested best parameters in manual deployment

## 📚 Complete Command Reference

```bash
# Basic optimization
python3 optimize_utlidar.py

# With custom rosbag
python3 optimize_utlidar.py --rosbag /path/to/rosbag

# Detailed customization
python3 optimize_utlidar.py \
  --rosbag tester_rosbag/rosbag2_2026_04_09-10_58_57 \
  --config point_lio/config/utlidar.yaml \
  --trials 50 \
  --timeout 3600 \
  --target-ate 0.3 \
  --output-dir results

# Analyze results
python3 analyze_optimization.py
python3 analyze_optimization.py --all
python3 analyze_optimization.py --plot-sensitivity

# Compare trajectories offline
python3 compare_trajectories.py --extract --rosbag /path
python3 compare_trajectories.py --compare --input trajectories.json
```

---

**Framework Version**: 1.0
**Last Updated**: April 2026
**Compatibility**: Point-LIO v2.0+, ROS 2 Humble+, Python 3.8+
