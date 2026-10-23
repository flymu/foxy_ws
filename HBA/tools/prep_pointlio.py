#!/usr/bin/env python3
"""Prepare Point-LIO output for HBA with motion compensation.

Point-LIO exports:
- trajectory.txt: relative-time poses in Euler angles (roll/pitch/yaw deg) + velocity
- sparse keyframe PCDs (output_clouds/frame_*.pcd)

HBA needs:
- pcd/<seq>.pcd: one LiDAR-frame cloud per scan (in LiDAR coordinates, motion-compensated)
- pose.json: tx ty tz qw qx qy qz per frame (quaternions, world frame)

This tool:
1. Converts trajectory.txt (Euler) → pose_init.json (quaternion)
2. Generates per-scan PCD from raw LiDAR txt files
3. Applies motion compensation (undistortion) using trajectory velocity
4. Creates times.txt for reference

Motion compensation:
- Each point in a scan has a timestamp (scan duration ~100ms)
- Use trajectory velocity to extrapolate pose at each point's timestamp
- Transform point back to frame center timestamp (undistort)

usage: prep_pointlio.py <raw_scan_dir> <trajectory.txt> <out_dir> [--no-undistort]
"""

import sys
import os
import re
import struct
from pathlib import Path

import numpy as np


def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    """ZYX intrinsic Euler angles (deg) to quaternion wxyz."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cy, sy = np.cos(y/2), np.sin(y/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cr, sr = np.cos(r/2), np.sin(r/2)
    return np.array([
        cy*cp*cr + sy*sp*sr,
        cy*cp*sr - sy*sp*cr,
        sy*cp*sr + cy*sp*cr,
        sy*cp*cr - cy*sp*sr,
    ])


def quat_to_rotation_matrix(q):
    """Convert quaternion [w x y z] to 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def write_binary_pcd(pts, path):
    """Write N×3 points as PCL binary .pcd (PointXYZ)."""
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.astype(np.float32).tobytes())


def main():
    if len(sys.argv) < 4 or len(sys.argv) > 5:
        print(__doc__)
        sys.exit(1)
    raw_dir = Path(sys.argv[1])
    traj_file = Path(sys.argv[2])
    out_dir = Path(sys.argv[3])
    do_undistort = '--no-undistort' not in sys.argv

    # 1. Parse trajectory and build t->pose lookup
    poses = []
    with open(traj_file) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            t, x, y, z = map(float, parts[0:4])
            r, p, yaw = map(float, parts[4:7])
            vx, vy, vz = map(float, parts[7:10])
            q = euler_to_quat(r, p, yaw)
            poses.append((t, x, y, z, q, np.array([vx, vy, vz])))
    print(f"loaded {len(poses)} poses from {traj_file}, t=[{poses[0][0]:.3f}, {poses[-1][0]:.3f}] s")
    if do_undistort:
        print("motion compensation: enabled (using trajectory velocity)")
    else:
        print("motion compensation: disabled (--no-undistort)")

    # Convert to structured array for easier indexing
    pose_arr = np.array([(t, x, y, z, *q, *v) for t, x, y, z, q, v in poses],
                        dtype=[('t', 'f8'), ('x', 'f8'), ('y', 'f8'), ('z', 'f8'),
                               ('qw', 'f8'), ('qx', 'f8'), ('qy', 'f8'), ('qz', 'f8'),
                               ('vx', 'f8'), ('vy', 'f8'), ('vz', 'f8')])

    # 2. Parse raw scan filenames (scan_<seq>_<unix_ms>.txt) and match to trajectory
    scan_files = sorted(raw_dir.glob('scan_*.txt'))
    def parse_ts(fn):
        m = re.search(r'_(\d+)ms\.txt$', fn.name)
        return int(m.group(1)) / 1000.0 if m else None
    t_unix = np.array([parse_ts(fn) for fn in scan_files])
    t_unix0 = t_unix[0]
    t_rel = t_unix - t_unix0

    # Match scans to trajectory by relative time (for each trajectory pose, find closest scan)
    matched = []
    for j in range(len(pose_arr)):
        t_traj = pose_arr['t'][j]
        i_best = np.argmin(np.abs(t_rel - t_traj))
        if abs(t_rel[i_best] - t_traj) < 0.15:
            matched.append((i_best, j, scan_files[i_best]))
    print(f"matched {len(matched)}/{len(pose_arr)} trajectory poses to scans (tol=0.15s)")

    if not matched:
        sys.exit("no scans matched trajectory — check timestamp alignment")

    # 3. Convert matched scans to PCD with motion compensation
    (out_dir / 'pcd').mkdir(parents=True, exist_ok=True)
    pose_out = []
    times_out = []
    for seq_out, (i_scan, i_pose, scan_path) in enumerate(matched):
        # Read raw scan: unix_timestamp x y z reflectivity (skip header)
        pts_raw = []
        ts_pts = []
        with open(scan_path) as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                parts = line.split()
                if len(parts) >= 5:
                    ts_pts.append(float(parts[0]))
                    pts_raw.append([float(parts[1]), float(parts[2]), float(parts[3])])
        if not pts_raw:
            continue
        pts = np.array(pts_raw, dtype=np.float32)
        ts_pts = np.array(ts_pts)

        # Motion compensation (undistort)
        if do_undistort and len(ts_pts) > 0:
            # Frame center time (from trajectory)
            t_center = t_unix0 + pose_arr['t'][i_pose]

            # Velocity in WORLD frame (from Point-LIO trajectory)
            v_world = np.array([pose_arr['vx'][i_pose], pose_arr['vy'][i_pose], pose_arr['vz'][i_pose]])

            # Transform velocity to LiDAR frame
            # Point-LIO pose: T_world_lidar (lidar points → world coords)
            # We need v_lidar = R_world_lidar^T @ v_world
            q = np.array([pose_arr['qw'][i_pose], pose_arr['qx'][i_pose],
                         pose_arr['qy'][i_pose], pose_arr['qz'][i_pose]])
            R_world_lidar = quat_to_rotation_matrix(q)
            v_lidar = R_world_lidar.T @ v_world  # transform to LiDAR frame

            # For each point, compute dt from frame center
            # Point was captured at time t = t_center + dt
            # Sensor was displaced by v_lidar * dt in LiDAR frame at that time
            # To undistort, remove this displacement: p_corrected = p_raw - v_lidar * dt
            dt = ts_pts - t_center  # shape (N,)
            displacements = v_lidar[None, :] * dt[:, None]  # (N, 3) in LiDAR frame
            pts = pts - displacements.astype(np.float32)

        # Write PCD
        pcd_path = out_dir / 'pcd' / f'{seq_out:05d}.pcd'
        write_binary_pcd(pts, pcd_path)

        # Collect pose (trajectory is world-frame)
        p = pose_arr[i_pose]
        pose_out.append([p['x'], p['y'], p['z'], p['qw'], p['qx'], p['qy'], p['qz']])
        times_out.append(t_unix[i_scan])

        if (seq_out + 1) % 100 == 0:
            print(f"  {seq_out+1}/{len(matched)} scans converted, last pcd {pts.shape[0]} pts")

    # Write pose_init.json (HBA format: tx ty tz qw qx qy qz per line)
    pose_mat = np.array(pose_out)
    np.savetxt(out_dir / 'pose_init.json', pose_mat, fmt='%.9f')
    print(f"wrote {len(pose_out)} poses to {out_dir / 'pose_init.json'}")

    # Write times.txt
    np.savetxt(out_dir / 'times.txt', times_out, fmt='%.6f')
    print(f"wrote {len(times_out)} timestamps to {out_dir / 'times.txt'}")

    # Sanity check
    norms = np.linalg.norm(pose_mat[:, 3:7], axis=1)
    print(f"quat norms: min {norms.min():.9f}, max {norms.max():.9f}")


if __name__ == '__main__':
    main()
