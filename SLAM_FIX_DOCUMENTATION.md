# SLAM Trajectory Issue - Root Cause and Solution

## Problem Summary

**Symptom**: SLAM trajectory file (`slam_trajectory.txt`) was empty, but mocap trajectory file worked fine and was visible in RViz.

## Root Cause Analysis

The original `trajectory_node.py` has a **hardware dependency**:

1. **At initialization** (line 66-73):
   ```python
   self.mocap = motioncapture.connect("optitrack", {'hostname': '192.168.2.141'})
   ```
   Tries to connect to live OptiTrack system

2. **For SLAM logging** (line 143-145):
   ```python
   if self.mocap_aligned:  # Only logs if this flag is True
       self.slam_file.write(line)
   ```
   Only logs SLAM data if `mocap_aligned = True`

3. **Setting mocap_aligned** (line 178-179):
   - Requires `mocap_timer_callback` to run
   - Which calls `self.mocap.waitForNextFrame()` (line 155)
   - **This blocks without hardware connection**

### What happens during rosbag playback (NO hardware):
- ✓ Mocap data comes from rosbag → `/mocap_path` published → mocap trajectory saved ✓
- ✗ Point-LIO publishes `/Odometry` 
- ✗ BUT trajectory_node can't initialize mocap_aligned
- ✗ SLAM logging never happens → empty file ✗

## Solution Implemented

**New `trajectory_listener.py`** - Hardware Independent

Instead of relying on `trajectory_node.py`, created a standalone listener that:

1. **Subscribes directly to `/Odometry`** (Point-LIO raw output):
   ```python
   self.create_subscription(Odometry, '/Odometry', self.slam_callback, 10)
   ```

2. **Applies same transformation as trajectory_node.py**:
   ```python
   pos_sensor = np.array([msg.pose.pose.position.x, ...])
   rot_sensor = R.from_quat([...])
   global_offset = rot_sensor.apply(self.local_offset)  # offset: [-0.2894, 0.0, 0.0468]
   pos_body = pos_sensor + global_offset
   ```

3. **Saves directly to file**:
   ```python
   with open(self.slam_file, 'a') as f:
       f.write(line)
   ```

4. **No hardware connection required** - Works with pure rosbag playback

## Result

| File | Before | After |
|------|--------|-------|
| `slam_trajectory.txt` | ❌ Empty | ✅ Populated |
| `mocap_trajectory.txt` | ✅ Working | ✅ Working |
| RViz `/body_path` | ❌ Not visible | ✅ Visible (from Point-LIO) |
| Hardware needed | ✅ Yes (OptiTrack) | ❌ No |

## Files Changed

1. **trajectory_listener.py** - Complete rewrite:
   - Now imports `Odometry` and `scipy.spatial.transform.Rotation`
   - Does sensor-to-body transformation locally
   - No dependency on trajectory_node.py's hardware connection

2. **trajectory_node.py** - Minor fix:
   - Changed `/state_estimation` → `/Odometry` (line 21)

3. **parameter_optimizer.py** - Already correct:
   - Starts trajectory_listener instead of mocap_odometry node
   - No changes needed

4. **OPTIMIZATION_README.md** - Updated:
   - Explained new hardware-independent approach
   - Added troubleshooting section
   - Clarified why rosbag works now

## Dependencies

Already in Dockerfile (line 24):
```bash
pip3 install motioncapture scipy pyyaml numpy
```
- ✅ scipy (for Rotation)
- ✅ numpy (for array operations)
