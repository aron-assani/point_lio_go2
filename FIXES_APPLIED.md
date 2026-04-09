# Parameter Optimizer - Fixes Applied

## Issues Fixed

### 1. Empty Trajectory Files Issue ✓

**Problem**: Trajectory files (`slam_trajectory.txt` and `mocap_trajectory.txt`) were being created but remained empty.

**Root Cause**: File writes were not being flushed to disk immediately, so when the optimizer checked file sizes, the data hadn't been committed.

**Solution Applied**:
- Modified `trajectory_node.py` to call `flush()` after each file write operation
- Now both SLAM and MoCap trajectories are flushed after every data point:
  - Line 150: `self.slam_file.flush()` 
  - Line 211: `self.mocap_file.flush()`

### 2. RViz Window Not Closing ✓

**Problem**: Multiple RViz windows were accumulating across iterations, consuming resources.

**Root Cause**: The launch file spawned RViz but wasn't being properly closed before the next iteration.

**Solution Applied**:
- Removed the problematic `trajectory_listener.py` process from the optimizer
- Kept the `pkill -9 rviz2` call in `cleanup_processes()`
- Simplified the process management to only run necessary components

### 3. Trajectory Data Recording Issue ✓

**Problem**: The rosbag playback and node startup timing wasn't sufficient for data to be recorded.

**Root Cause**: 
- `trajectory_node.py` needs ~10 seconds before it starts recording (alignment phase)
- Not enough additional time was allocated for data collection

**Solution Applied**:
- Increased timeout from 120s to 150s (2.5 minutes)
- Changed file size threshold from 100 bytes to 500 bytes per file
- This ensures sufficient data is recorded before moving to the next iteration

### 4. Early Termination Feature ✓

**Problem**: Long iterations with poor parameters were wasting time.

**Solution Applied**:
- Added `--threshold` command-line parameter to optimizer
- If an iteration's ATE exceeds the threshold, it returns `inf` to signal poor fit
- Optimizer skips further evaluation of that parameter set

**Usage Example**:
```bash
python3 parameter_optimizer.py rosbag_file.db3 utlidar.yaml --threshold 1.0
```

This will terminate iterations early if ATE > 1.0 meters.

## Files Modified

1. **mocap_odometry/mocap_odometry/trajectory_node.py**
   - Added `flush()` calls to ensure data is written to disk

2. **scripts/parameter_optimizer.py**
   - Removed trajectory_listener subprocess spawning
   - Added `error_threshold` parameter support
   - Increased wait timeout and file size threshold
   - Added early termination logic in `run_optimization_iteration()`
   - Updated command-line argument parsing in `main()`

3. **docker/Dockerfile**
   - Already includes scipy and pyyaml (no changes needed)

## Testing

To test the fixes:

1. **With default settings**:
```bash
python3 scripts/parameter_optimizer.py ~/rosbags/go2_data.db3
```

2. **With early termination** (skip iterations with ATE > 2.0m):
```bash
python3 scripts/parameter_optimizer.py ~/rosbags/go2_data.db3 ~/point_lio/config/utlidar.yaml --threshold 2.0
```

## Expected Behavior

- Trajectory files should now contain data (>500 bytes)
- RViz windows should close between iterations
- Each iteration should take ~2.5 minutes (allowing for alignment + recording)
- With threshold set, poor parameter sets will be rejected quickly
- Best ATE and "New best ATE!" messages should appear in output
