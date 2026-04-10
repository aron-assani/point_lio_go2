#!/usr/bin/env python3
"""
Standalone Trajectory Comparison Tool

Compares trajectories from different sources without requiring Point-LIO to run.
Useful for analyzing rosbag trajectories and computing ATE metrics.

Usage: Extract trajectories from rosbag and compare them.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Dict, Tuple
import numpy as np
from collections import deque
import json

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from scipy.spatial.transform import Rotation as R
    from rclpy.serialization import deserialize_message
except ImportError:
    print("⚠️  ROS 2 packages not available. This tool must be run inside Docker.")
    sys.exit(1)


class TrajectoryExtractor(Node):
    """Extract trajectories from rosbag topics."""

    def __init__(self, output_file: Path = None):
        super().__init__('trajectory_extractor')

        self.trajectories = {}
        self.output_file = output_file
        self.extraction_complete = False

        # Subscribe to relevant topics
        self.subs = []

        # Subscribe to Odometry topics (Point-LIO output)
        self.subs.append(self.create_subscription(
            Odometry,
            '/Odometry',
            self._make_callback('odometry_estimated'),
            10
        ))

        # Subscribe to Path topics (mocap truth)
        self.subs.append(self.create_subscription(
            Path,
            '/mocap_path',
            self._make_callback('mocap_truth'),
            10
        ))

    def _make_callback(self, topic_name: str):
        """Create callback for a topic."""
        def callback(msg):
            if topic_name == 'odometry_estimated':
                self._handle_odometry(msg, topic_name)
            elif topic_name == 'mocap_truth':
                self._handle_path(msg, topic_name)

        return callback

    def _handle_odometry(self, msg: Odometry, topic_name: str):
        """Handle Odometry messages."""
        if topic_name not in self.trajectories:
            self.trajectories[topic_name] = []

        pose = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'position': np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]),
            'quaternion': np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
        }
        self.trajectories[topic_name].append(pose)

    def _handle_path(self, msg: Path, topic_name: str):
        """Handle Path messages."""
        if topic_name not in self.trajectories:
            self.trajectories[topic_name] = []

        self.trajectories[topic_name] = []  # Reset on new path
        for pose_stamped in msg.poses:
            pose = {
                'timestamp': pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9,
                'position': np.array([
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z
                ]),
                'quaternion': np.array([
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                    pose_stamped.pose.orientation.w
                ])
            }
            self.trajectories[topic_name].append(pose)

    def get_trajectories(self) -> Dict[str, List]:
        """Return collected trajectories."""
        return self.trajectories

    def save_trajectories(self):
        """Save trajectories to file."""
        if not self.output_file:
            return

        output = {}
        for topic_name, poses in self.trajectories.items():
            output[topic_name] = []
            for pose in poses:
                output[topic_name].append({
                    'timestamp': float(pose['timestamp']),
                    'position': pose['position'].tolist(),
                    'quaternion': pose['quaternion'].tolist()
                })

        with open(self.output_file, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"✓ Trajectories saved: {self.output_file}")


class TrajectoryComparator:
    """Compare and analyze multiple trajectories."""

    @staticmethod
    def align_trajectories(est_traj: List, gt_traj: List) -> List:
        """
        Align estimated trajectory to ground truth.

        Uses first and last poses to compute scale, rotation, and translation.
        """
        if len(est_traj) < 2 or len(gt_traj) < 2:
            return est_traj

        # Get first and last poses
        est_start = est_traj[0]['position']
        est_end = est_traj[-1]['position']
        gt_start = gt_traj[0]['position']
        gt_end = gt_traj[-1]['position']

        # Compute scale
        est_dist = np.linalg.norm(est_end - est_start)
        gt_dist = np.linalg.norm(gt_end - gt_start)
        scale = gt_dist / est_dist if est_dist > 1e-6 else 1.0
        scale = np.clip(scale, 0.5, 2.0)

        # Compute rotation (from first and last pose quaternions)
        est_rot_start = R.from_quat(est_traj[0]['quaternion'])
        est_rot_end = R.from_quat(est_traj[-1]['quaternion'])
        gt_rot_start = R.from_quat(gt_traj[0]['quaternion'])
        gt_rot_end = R.from_quat(gt_traj[-1]['quaternion'])

        rot_align = gt_rot_end * est_rot_end.inv()

        # Apply alignment
        aligned = []
        for pose in est_traj:
            pos = rot_align.apply(scale * pose['position']) + gt_start - rot_align.apply(scale * est_start)
            quat = (rot_align * R.from_quat(pose['quaternion'])).as_quat()

            aligned.append({
                'timestamp': pose['timestamp'],
                'position': pos,
                'quaternion': quat
            })

        return aligned

    @staticmethod
    def compute_ate(est_traj: List, gt_traj: List) -> Tuple[float, int, List]:
        """
        Compute Absolute Trajectory Error (ATE).

        Returns: ATE (meters), number of aligned poses, error per pose
        """
        if len(est_traj) < 2 or len(gt_traj) < 2:
            return float('inf'), 0, []

        # Align
        aligned_traj = TrajectoryComparator.align_trajectories(est_traj, gt_traj)

        # Compute ATE by temporal matching
        errors = []
        aligned_count = 0

        g_idx = 0
        for e_pose in aligned_traj:
            # Find nearest gt pose by timestamp
            while g_idx < len(gt_traj) - 1 and \
                  abs(gt_traj[g_idx + 1]['timestamp'] - e_pose['timestamp']) < \
                  abs(gt_traj[g_idx]['timestamp'] - e_pose['timestamp']):
                g_idx += 1

            if g_idx < len(gt_traj):
                error = np.linalg.norm(e_pose['position'] - gt_traj[g_idx]['position'])
                errors.append(error)
                aligned_count += 1

        ate = np.sqrt(np.mean(np.array(errors) ** 2)) if errors else float('inf')

        return ate, aligned_count, errors

    @staticmethod
    def compute_rpe(est_traj: List, gt_traj: List, delta: int = 1) -> Tuple[float, int]:
        """
        Compute Relative Pose Error (RPE).

        Compares relative motions between consecutive poses.

        Args:
            delta: Pose step to use for computing deltas

        Returns: RPE, number of valid comparisons
        """
        if len(est_traj) < delta + 1 or len(gt_traj) < delta + 1:
            return float('inf'), 0

        aligned_traj = TrajectoryComparator.align_trajectories(est_traj, gt_traj)

        errors = []
        valid_count = 0

        for i in range(len(aligned_traj) - delta):
            e_pos_delta = aligned_traj[i + delta]['position'] - aligned_traj[i]['position']
            e_rot_delta = R.from_quat(aligned_traj[i + delta]['quaternion']) * \
                         R.from_quat(aligned_traj[i]['quaternion']).inv()

            # Find corresponding gt poses
            g_idx_i = min(i, len(gt_traj) - delta - 1)
            g_pos_delta = gt_traj[g_idx_i + delta]['position'] - gt_traj[g_idx_i]['position']
            g_rot_delta = R.from_quat(gt_traj[g_idx_i + delta]['quaternion']) * \
                         R.from_quat(gt_traj[g_idx_i]['quaternion']).inv()

            # Position error
            pos_error = np.linalg.norm(e_pos_delta - g_pos_delta)

            # Rotation error (angle)
            rot_error = np.linalg.norm((e_rot_delta.inv() * g_rot_delta * e_rot_delta.inv()).as_rotvec())

            errors.append(pos_error + 0.1 * rot_error)  # Weighted sum
            valid_count += 1

        rpe = np.mean(errors) if errors else float('inf')

        return rpe, valid_count

    @staticmethod
    def print_statistics(est_traj: List, gt_traj: List):
        """Print trajectory comparison statistics."""
        print("\n" + "=" * 70)
        print("📊 Trajectory Comparison Statistics")
        print("=" * 70)

        print(f"\nEstimated trajectory: {len(est_traj)} poses")
        print(f"Ground truth trajectory: {len(gt_traj)} poses")

        if len(est_traj) > 0:
            est_distances = np.array([
                np.linalg.norm(est_traj[i+1]['position'] - est_traj[i]['position'])
                for i in range(len(est_traj) - 1)
            ])
            print(f"  - Distance traveled: {np.sum(est_distances):.2f}m")
            print(f"  - Mean distance between consecutive poses: {np.mean(est_distances):.4f}m")

        if len(gt_traj) > 0:
            gt_distances = np.array([
                np.linalg.norm(gt_traj[i+1]['position'] - gt_traj[i]['position'])
                for i in range(len(gt_traj) - 1)
            ])
            print(f"\n  Ground truth distance traveled: {np.sum(gt_distances):.2f}m")
            print(f"  - Mean distance between consecutive poses: {np.mean(gt_distances):.4f}m")

        # Compute errors
        ate, aligned_count, errors = TrajectoryComparator.compute_ate(est_traj, gt_traj)
        print(f"\n🎯 Absolute Trajectory Error (ATE):")
        print(f"  - Mean ATE: {ate:.4f}m")
        if errors:
            print(f"  - Min ATE: {min(errors):.4f}m")
            print(f"  - Max ATE: {max(errors):.4f}m")
            print(f"  - Aligned poses: {aligned_count}")

        # Print per-pose errors if not too many
        if errors and len(errors) <= 20:
            print(f"\n  Per-pose errors:")
            for i, e in enumerate(errors):
                print(f"    {i+1}: {e:.4f}m")

        print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Standalone trajectory comparison and analysis'
    )

    parser.add_argument(
        '--extract',
        action='store_true',
        help='Extract trajectories from rosbag'
    )

    parser.add_argument(
        '--rosbag',
        type=Path,
        help='Path to rosbag directory (for extraction)'
    )

    parser.add_argument(
        '--output',
        type=Path,
        default=Path('extracted_trajectories.json'),
        help='Output file for extracted trajectories'
    )

    parser.add_argument(
        '--compare',
        action='store_true',
        help='Compare previously extracted trajectories'
    )

    parser.add_argument(
        '--input',
        type=Path,
        default=Path('extracted_trajectories.json'),
        help='Input file with extracted trajectories (for comparison)'
    )

    args = parser.parse_args()

    if args.extract:
        if not args.rosbag:
            print("❌ --rosbag required for extraction")
            sys.exit(1)

        print("🔴 Extracting trajectories from rosbag...")
        print(f"   Rosbag: {args.rosbag}")

        rclpy.init(args=None)
        extractor = TrajectoryExtractor(args.output)

        # Run extraction with timeout
        import time
        timeout_start = time.time()
        timeout = 120  # seconds

        while (time.time() - timeout_start) < timeout:
            rclpy.spin_once(extractor, timeout_sec=0.5)

        extractor.save_trajectories()
        extractor.destroy_node()
        rclpy.shutdown()

        print("✓ Extraction complete!")

    elif args.compare:
        print("📊 Comparing trajectories...")

        if not args.input.exists():
            print(f"❌ Input file not found: {args.input}")
            sys.exit(1)

        with open(args.input, 'r') as f:
            data = json.load(f)

        est_traj = [
            {
                'timestamp': p['timestamp'],
                'position': np.array(p['position']),
                'quaternion': np.array(p['quaternion'])
            }
            for p in data.get('odometry_estimated', [])
        ]

        gt_traj = [
            {
                'timestamp': p['timestamp'],
                'position': np.array(p['position']),
                'quaternion': np.array(p['quaternion'])
            }
            for p in data.get('mocap_truth', [])
        ]

        if not est_traj or not gt_traj:
            print("❌ Could not find trajectory data in input file")
            sys.exit(1)

        TrajectoryComparator.print_statistics(est_traj, gt_traj)

    else:
        parser.print_help()


if __name__ == '__main__':
    main()
