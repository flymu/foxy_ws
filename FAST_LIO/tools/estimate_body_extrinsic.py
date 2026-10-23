#!/usr/bin/env python3
"""Estimate the LiDAR-to-body rotation for a LiDAR mounted on a legged robot.

Body frame convention: x forward, y left, z up.

Three of the six extrinsic degrees of freedom are recoverable from this data:

  roll, pitch  from gravity. The accelerometer's mean during a stationary
               stretch gives the body +z axis directly in LiDAR coordinates, and
               the mapped ground plane's normal confirms it independently.
  yaw          from the direction of travel, assuming the robot walks along its
               own +x. Weaker than the other two -- it rests on that assumption,
               not on a measurement.
  translation  NOT recoverable. See the note at the bottom of the output.

  python3 tools/estimate_body_extrinsic.py --imu /tmp/mid360/imu.txt \
          --traj /tmp/mid360_out/traj.txt --pcd_dir /tmp/mid360/pcd
"""
import argparse
import glob
import math
import os
import struct
import sys

import numpy as np

# A stationary window needs the gyro this quiet (rad/s) and the accelerometer
# this steady (m/s^2) across the whole window.
GYRO_STILL = 0.03
ACC_STILL = 0.15

# Only frames moving at least this fast (m/s, horizontal) vote on yaw; slow
# frames are dominated by the gait's sway rather than by travel.
MIN_SPEED = 0.40

# The IMU's nominal rate (Hz), used only to report the stationary stretch's
# duration; the per-sample timestamps are too jittery to differentiate.
IMU_RATE = 200.0


def load_imu(path):
    rows = []
    for line in open(path):
        if line.startswith("#"):
            continue
        c = line.split()
        if len(c) >= 7:
            rows.append([float(v) for v in c[:7]])
    if not rows:
        sys.exit(f"error: no IMU samples in {path}")
    a = np.array(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:7]


def load_traj(path):
    rows = [[float(v) for v in l.split()] for l in open(path) if not l.startswith("#")]
    a = np.array(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def quat2R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def gravity_from_rest(t, gyr, acc):
    """Longest leading stationary stretch -> body +z (up) in LiDAR coordinates."""
    gn = np.linalg.norm(gyr, axis=1)
    end = 0
    while end + 1 < len(gn) and gn[end + 1] < GYRO_STILL:
        end += 1
    if end < 100:
        sys.exit("error: no usable stationary stretch at the start of the IMU log")
    a = acc[:end + 1]
    if a.std(axis=0).max() > ACC_STILL:
        print("warning: the stationary stretch is noisy; check the data",
              file=sys.stderr)
    u = a.mean(axis=0)
    return u / np.linalg.norm(u), end + 1, np.linalg.norm(u), a.std(axis=0)


def ground_normal(pcd_dir, up, n_frames=20):
    """RANSAC the ground plane in the first few scans; an independent up check."""
    rec = struct.Struct("<fffffH")
    files = sorted(glob.glob(os.path.join(pcd_dir, "*.pcd")))[:n_frames]
    if not files:
        return None
    normals = []
    heights = []
    for p in files:
        f = open(p, "rb")
        n = 0
        while True:
            l = f.readline()
            if l.startswith(b"POINTS"):
                n = int(l.split()[1])
            if l.startswith(b"DATA"):
                break
        data = f.read()
        P = np.array([rec.unpack_from(data, i * 22)[:3] for i in range(n)])
        h = P @ up
        sel = P[(h < -0.15) & (h > -2.0)]
        if len(sel) < 200:
            continue
        rng = np.random.default_rng(0)
        best = None
        for _ in range(300):
            i = rng.choice(len(sel), 3, replace=False)
            a, b, c = sel[i]
            nv = np.cross(b - a, c - a)
            nn = np.linalg.norm(nv)
            if nn < 1e-6:
                continue
            nv /= nn
            if abs(nv @ up) < 0.9:      # keep near-horizontal planes only
                continue
            d = -nv @ a
            cnt = (np.abs(sel @ nv + d) < 0.06).sum()
            if best is None or cnt > best[0]:
                best = (cnt, nv, d)
        if best is None:
            continue
        _, nv, d = best
        inl = sel[np.abs(sel @ nv + d) < 0.06]
        c = inl.mean(0)
        nv = np.linalg.svd(inl - c)[2][2]
        if nv @ up < 0:
            nv = -nv
        normals.append(nv)
        heights.append(-(nv @ c))
    if not normals:
        return None
    m = np.mean(normals, axis=0)
    return m / np.linalg.norm(m), np.mean(heights), np.std(heights), len(normals)


def smooth(x, w=9):
    return np.convolve(x, np.ones(w) / w, mode="same")


def yaw_from_travel(t, P, Q, up):
    """Rotate about `up` until sideways velocity is minimal in the body frame."""
    e1 = np.cross(up, [0.0, 0.0, 1.0])
    e1 /= np.linalg.norm(e1)
    R_gw = np.vstack([e1, np.cross(up, e1), up])

    Pg = P @ R_gw.T
    Ps = np.column_stack([smooth(Pg[:, i]) for i in range(3)])
    dt = np.median(np.diff(t))
    Vg = np.gradient(Ps, axis=0) / dt
    speed = np.linalg.norm(Vg[:, :2], axis=1)

    # Velocity expressed in each frame's own LiDAR frame, projected horizontal.
    V = []
    for k in range(len(t)):
        v = quat2R(Q[k]).T @ (Vg[k] @ R_gw)
        V.append(v - (v @ up) * up)
    V = np.array(V)[speed > MIN_SPEED]
    if len(V) < 20:
        sys.exit("error: too few moving frames to estimate yaw")

    # An arbitrary horizontal reference to measure the rotation from; the search
    # below is global, so this only fixes where zero degrees sits.
    ref = V[0] - (V[0] @ up) * up
    ref /= np.linalg.norm(ref)

    def axes(dy):
        a = math.radians(dy)
        f = ref * math.cos(a) + np.cross(up, ref) * math.sin(a)
        x_b = f - (f @ up) * up
        x_b /= np.linalg.norm(x_b)
        return x_b, np.cross(up, x_b)

    def cost(dy, Vs):
        return np.median(np.abs(Vs @ axes(dy)[1]))

    def solve(Vs):
        # Scan the full circle: minimising |sideways| has two minima 180 apart,
        # and a local search cannot be trusted to start in the right basin.
        cand = [(cost(d, Vs), d) for d in range(0, 360)
                if np.median(Vs @ axes(d)[0]) > 0]   # forward, not reverse
        if not cand:
            sys.exit("error: no travel direction with positive forward speed")
        d0 = min(cand)[1]
        # Refine within one scan step.
        gr = (math.sqrt(5) - 1) / 2
        a, b = d0 - 1.0, d0 + 1.0
        c, d = b - gr * (b - a), a + gr * (b - a)
        for _ in range(40):
            if cost(c, Vs) < cost(d, Vs):
                b = d
            else:
                a = c
            c, d = b - gr * (b - a), a + gr * (b - a)
        return (a + b) / 2

    dy = solve(V)
    x_b, y_b = axes(dy)

    rng = np.random.default_rng(0)
    bs = np.array([solve(V[rng.choice(len(V), len(V))]) for _ in range(60)])
    # Unwrap around the point estimate before taking a spread.
    bs = (bs - dy + 180) % 360 - 180
    return x_b, y_b, len(V), np.median(V @ x_b), np.median(np.abs(V @ y_b)), bs.std()


def eulZYX(R):
    p = math.asin(max(-1, min(1, -R[2, 0])))
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])),
            math.degrees(p),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--imu", required=True, help="IMU text file (m/s^2)")
    ap.add_argument("--traj", required=True, help="TUM trajectory from fastlio_offline")
    ap.add_argument("--pcd_dir", help="scan directory, for the ground-plane cross-check")
    args = ap.parse_args()

    t, gyr, acc = load_imu(args.imu)
    up, n_rest, mag, sd = gravity_from_rest(t, gyr, acc)
    print(f"stationary stretch: {n_rest} samples ({n_rest / IMU_RATE:.1f} s)")
    print(f"  |acc| = {mag:.4f} m/s^2, per-axis sd = "
          f"({sd[0]:.3f}, {sd[1]:.3f}, {sd[2]:.3f})")
    print(f"  body +z (up) in LiDAR frame = "
          f"({up[0]:+.5f}, {up[1]:+.5f}, {up[2]:+.5f})")
    print(f"  LiDAR +z tilt off vertical  = {math.degrees(math.acos(up[2])):.2f} deg")
    print(f"  gyro bias = ({gyr[:n_rest].mean(0)[0]:+.5f}, "
          f"{gyr[:n_rest].mean(0)[1]:+.5f}, {gyr[:n_rest].mean(0)[2]:+.5f}) rad/s")

    if args.pcd_dir:
        g = ground_normal(args.pcd_dir, up)
        if g:
            nv, h, hsd, nf = g
            print(f"\nground plane over {nf} frames (independent check)")
            print(f"  normal = ({nv[0]:+.5f}, {nv[1]:+.5f}, {nv[2]:+.5f})")
            print(f"  angle to IMU gravity = "
                  f"{math.degrees(math.acos(min(1, abs(nv @ up)))):.2f} deg")
            print(f"  LiDAR height above ground = {h:.4f} m (sd {hsd:.4f})")

    tt, P, Q = load_traj(args.traj)
    x_b, y_b, nmv, lon, lat, dysd = yaw_from_travel(tt, P, Q, up)
    print(f"\nyaw from travel direction ({nmv} frames above {MIN_SPEED} m/s)")
    print(f"  median forward speed = {lon:+.4f} m/s "
          f"({'walks along +x' if lon > 0 else 'WALKS BACKWARDS: check sign'})")
    print(f"  median |sideways|    = {lat:.4f} m/s")
    print(f"  bootstrap sd         = {dysd:.2f} deg")

    R_BL = np.vstack([x_b, y_b, up])          # LiDAR vector -> body vector
    R_LB = R_BL.T                             # body -> LiDAR, i.e. extrinsic_R form
    roll, pitch, yaw = eulZYX(R_LB)
    single = math.degrees(math.acos(max(-1, min(1, (np.trace(R_LB) - 1) / 2))))

    print(f"\nR_lidar_from_body, ZYX Euler")
    print(f"  roll = {roll:+.2f} deg, pitch = {pitch:+.2f} deg, yaw = {yaw:+.2f} deg")
    print(f"  equivalent single rotation = {single:.2f} deg")
    print(f"  det = {np.linalg.det(R_BL):+.9f}, "
          f"orthogonality error = {np.abs(R_BL @ R_BL.T - np.eye(3)).max():.1e}")

    f = R_LB.flatten()
    print(f"\nextrinsic_R (row-major):")
    print(f"    extrinsic_R: [ {f[0]:+.6f}, {f[1]:+.6f}, {f[2]:+.6f},")
    print(f"                   {f[3]:+.6f}, {f[4]:+.6f}, {f[5]:+.6f},")
    print(f"                   {f[6]:+.6f}, {f[7]:+.6f}, {f[8]:+.6f} ]")

    print("\nTranslation is NOT estimated: a rigid LiDAR-to-body offset produces no\n"
          "signal that this data can separate from the trajectory itself. Measure it\n"
          "off the CAD model or the robot, or excite it with pure rotation about a\n"
          "known body point.")


if __name__ == "__main__":
    main()
