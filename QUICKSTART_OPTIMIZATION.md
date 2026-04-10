# Quick Start Guide: Parameter Optimization

This guide walks you through running the Point-LIO parameter optimization framework.

## 📋 Prerequisites

- Docker with this repository mounted
- Rosbag file in `tester_rosbag/` directory (containing `/utlidar/imu`, `/utlidar/cloud`, `/mocap_path` topics)
- ~5-10 GB free disk space (depending on rosbag size and number of trials)

## 🚀 Quick Start (3 Steps)

### Step 1: Verify Your Setup

```bash
# Inside Docker container
cd ~/point_lio_go2

# Check ROS 2
ros2 --version

# Verify rosbag exists
ls -lh tester_rosbag/rosbag2_*/
```

### Step 2: Run Optimization

```bash
# Basic optimization (50 trials, ~30-60 minutes)
python3 optimize_utlidar.py

# With custom parameters:
python3 optimize_utlidar.py \
  --rosbag tester_rosbag/rosbag2_2026_04_09-10_58_57 \
  --trials 100 \
  --target-ate 0.3 \
  --output-dir results
```

### Step 3: Analyze Results

```bash
# View summary
python3 analyze_optimization.py --top 10

# Generate plots
python3 analyze_optimization.py --all

# View convergence plot
display tester_rosbag/optimization_results/convergence.png
```

## 🎯 Expected Output

### Console Output During Optimization

```
═══════════════════════════════════════════════════════════════════
🚀 Starting Parameter Optimization
   Config: point_lio/config/utlidar.yaml
   Rosbag: tester_rosbag/rosbag2_2026_04_09-10_58_57
   Trials: 50, Timeout: None
   Target ATE: 0.5m
═══════════════════════════════════════════════════════════════════

Trial 1/50:
  Testing parameters: {'lidar_meas_cov': 0.032, 'imu_meas_acc_cov': 23.5, ...}
  📊 Collected: 287 estimated poses, 312 mocap poses
  ✓ Trial result: ATE=0.456m, aligned_poses=287
  🎯 New best! ATE=0.456m

Trial 2/50:
  Testing parameters: {'lidar_meas_cov': 0.041, 'imu_meas_acc_cov': 18.2, ...}
  📊 Collected: 287 estimated poses, 312 mocap poses
  ✓ Trial result: ATE=0.623m, aligned_poses=287
  ⏱️  No improvement (1/5)

...

✓ Optimization completed successfully!
═══════════════════════════════════════════════════════════════════
📈 Optimization Complete!
═══════════════════════════════════════════════════════════════════
Best ATE: 0.234m
Best parameters:
  - lidar_meas_cov: 0.018300
  - imu_meas_acc_cov: 12.450000
  - imu_meas_omg_cov: 6.280000
  - plane_thr: 0.145000
  - b_acc_cov: 0.045000
  - b_gyr_cov: 0.032000
  - filter_size_surf: 0.280000
  - filter_size_map: 0.320000
Total trials: 35
Results saved to: tester_rosbag/optimization_results
✓ Best configuration saved to: point_lio/config/utlidar.yaml
═══════════════════════════════════════════════════════════════════
```

### Generated Files

```
tester_rosbag/optimization_results/
├── optimization_log.json          # All trial data
├── convergence.png                # Convergence plot
├── sensitivity.png                # Parameter sensitivity
└── utlidar.yaml                   # Best config found
```

## 📊 Common Workflows

### Workflow 1: Quick Test (5-10 minutes)

For testing if setup is correct:

```bash
python3 optimize_utlidar.py --trials 5
```

**Good for:** Validating setup, checking if system works

### Workflow 2: Standard Optimization (1-2 hours)

Good balance of quality and time:

```bash
python3 optimize_utlidar.py --trials 50 --target-ate 0.3
```

**Good for:** Production optimization

### Workflow 3: Deep Optimization (3-4 hours)

Maximum quality but time-intensive:

```bash
python3 optimize_utlidar.py --trials 100 --timeout 14400
```

**Good for:** Getting absolute best parameters

### Workflow 4: Focused Parameter Search

If you know which parameters matter, edit `PARAM_BOUNDS` in script:

```python
# In optimize_utlidar.py
PARAM_BOUNDS = {
    'lidar_meas_cov': (0.005, 0.05),    # Narrower bounds
    'imu_meas_acc_cov': (8.0, 20.0),    # Focus on this range
}
```

Then run:

```bash
python3 optimize_utlidar.py --trials 40
```

## 🔍 Understanding the Results

### ATE Values

- **< 0.1m**: Excellent performance
- **0.1-0.3m**: Very good performance
- **0.3-0.5m**: Good performance
- **0.5-1.0m**: Acceptable performance
- **(> 1.0m)**: Poor performance, might indicate issues

### Parameter Sensitivity

Check which parameters matter most:

```bash
python3 analyze_optimization.py --analyze
```

**Output example:**
```
🔍 Parameter Impact Analysis:
Parameter                Correlation (|r|)
plane_thr                    0.6234
lidar_meas_cov              0.5891
imu_meas_acc_cov            0.4156
imu_meas_omg_cov            0.2034
filter_size_surf            0.1872
...
```

**Interpretation:** `plane_thr` has the highest impact, so tweaking it helps most.

## 💡 Tips for Success

### 1. Monitor the First Few Trials

Watch the first 5-10 trials to ensure they're working:

```bash
# In separate terminal, watch log file
tail -F tester_rosbag/optimization_results/optimization_log.json
```

### 2. Check Convergence

If ATE values are not improving after 20 trials, probably not going to get better. Consider stopping and analyzing what you have:

```bash
python3 analyze_optimization.py --plot-convergence
```

### 3. Validate Results

Run Point-LIO manually with best parameters to verify:

```bash
# Update config from results
# Run Point-LIO with rosbag
ros2 bag play tester_rosbag/rosbag2_2026_04_09-10_58_57 -l &
ros2 launch point_lio mapping_utlidar.launch rviz:=true
```

### 4. Save Best Configuration

After optimization completes, the best config is already saved to `utlidar.yaml`. Create backup:

```bash
cp point_lio/config/utlidar.yaml point_lio/config/utlidar.yaml.optimized_$(date +%Y%m%d_%H%M%S)
```

## 🐳 Using Docker Compose

For complete orchestration:

```bash
# Build images
docker-compose -f docker-compose-optimizer.yml build

# Run optimization
docker-compose -f docker-compose-optimizer.yml up

# View logs
docker-compose -f docker-compose-optimizer.yml logs -f optimizer

# Stop
docker-compose -f docker-compose-optimizer.yml down
```

## ❌ Troubleshooting

### Problem: "ros2 command not found"

**Solution:** Ensure you're inside the Docker container where ROS 2 is installed

### Problem: "Connection refused" errors

**Solution:** Point-LIO node may not be launching. Check if topics are being published:

```bash
ros2 topic list
# Should see /Odometry, /cloud_registered, etc.
```

### Problem: Very high ATE values (> 5.0m)

**Solution:** Something is wrong. Debug with:

```bash
# Check if mocap topic exists in rosbag
ros2 bag info tester_rosbag/rosbag2_*/
# Look for /mocap_path in topics

# Run Point-LIO manually and inspect
ros2 bag play tester_rosbag/rosbag2_*/ -l &
ros2 launch point_lio mapping_utlidar.launch rviz:=true
```

### Problem: Optimization runs but collects 0 poses

**Possible causes:**
- Point-LIO not starting
- Rosbag not being played
- Topic names mismatch

**Debug:**
```bash
# Check if nodes are running
ros2 node list

# Check topics
ros2 topic echo /Odometry  # Should show messages

# Check rosbag playback
ros2 bag play tester_rosbag/rosbag2_*/ -l
# In another terminal:
ros2 topic echo /utlidar/cloud
```

## 📊 Expected Runtime

- **First trial**: ~90 seconds (ROS 2 startup overhead)
- **Subsequent trials**: ~60 seconds each
- **50 trials**: ~50-60 minutes total
- **100 trials**: ~100-120 minutes total

## ✅ Validation Checklist

Before running optimization, verify:

- [ ] Docker container is running and you're inside it
- [ ] ROS 2 works: `ros2 --version`
- [ ] Point-LIO is installed: `ros2 pkg list | grep point_lio`
- [ ] Rosbag exists: `ls -l tester_rosbag/rosbag2_*/`
- [ ] Rosbag has required topics:
  ```bash
  ros2 bag info tester_rosbag/rosbag2_*/  # Should list the topics
  ```
- [ ] Disk space available: `df -h / | awk 'NR==2 {print $4}'` (need > 5GB)

---

**For detailed documentation, see:** `OPTIMIZATION_README.md`

**For issues or questions, check the troubleshooting section above or refer to main documentation.**
