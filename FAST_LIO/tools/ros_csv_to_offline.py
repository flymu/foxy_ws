#!/usr/bin/env python3
"""Convert a ROS-extracted dataset into the layout fastlio_offline expects.

Input (as produced by exporting a rosbag's sensor_msgs/Imu and point clouds):

  imu_data.csv                header + rows: timestamp, angular_velocity_(x|y|z),
                              linear_acceleration_(x|y|z); orientation_(x|y|z|w)
                              columns are optional (ignored either way --
                              FAST-LIO estimates attitude itself)
  pcd/frame_NNNNNN_<ts>.pcd   one pcl::PointXYZI cloud per frame; any *.pcd
                              whose name ends in <unix_seconds>.pcd works

Output:

  <out>/imu.txt               'timestamp gx gy gz ax ay az', m/s^2
  <out>/pcd/<ts>.pcd          binary velodyne-layout clouds (x y z intensity
                              time ring) with synthesized per-point times

IMPORTANT: the clouds must be the RAW topic (sensor frame). If the exporter
saved clouds that an upstream SLAM/odom had already transformed into a fixed
world frame, no LIO can run on the pair: the IMU reports the sensor's real
motion while the clouds sit still, and the filter diverges into ghost
velocity. The tool screens for exactly that signature (consecutive clouds
staying aligned through windows where the gyro reports fast rotation) and
fails loudly; see check_sensor_frame().

Per-point times. The export dropped them, but raw MID-360 clouds are still in
capture order (the non-repetitive rosette is emitted as a continuous stream,
and consecutive points are spatially adjacent -- the ~20k points per file are
a ~113 ms chunk of that stream, so file k+1 continues where file k ends). A
uniform spread over the frame interval therefore reconstructs the capture
time of each point to within a few point steps, which is what the undistortion
needs. The span of frame k is taken as the interval to the next frame
(--span overrides with a constant).

Frame stamping. If the exporter stamped each cloud at the END of its capture
window instead of its start, pass --stamp-shift <seconds> (the mean interval,
e.g. 0.1127) to move the effective frame start back accordingly.

  python3 tools/ros_csv_to_offline.py --in /home/mu/dataset/extracted --out /tmp/ros_dataset
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import pandas as pd

G_M_S2 = 9.81

PCD_HEADER = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity time ring
SIZE 4 4 4 4 4 2
TYPE F F F F F U
COUNT 1 1 1 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA binary
"""

PCD_RECORD = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("intensity", "<f4"), ("time", "<f4"), ("ring", "<u2")])

# The runner consumes timestamp + gyro + accel; the orientation columns are
# optional (some exports drop them, and FAST-LIO estimates attitude itself).
IMU_REQUIRED = ("timestamp",
                "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
                "linear_acceleration_x", "linear_acceleration_y",
                "linear_acceleration_z")
IMU_ORIENTATION = ("orientation_x", "orientation_y", "orientation_z",
                   "orientation_w")

# Any *.pcd name ending in a unix-seconds stamp, e.g. frame_000000_1792208537.731664.pcd
STAMP_RE = re.compile(r"(\d{9,10}\.\d+)\.pcd$")


def convert_imu(src, dst, acc_scale):
    """Read the sensor_msgs/Imu CSV, write the runner's imu.txt."""
    with open(src, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        col = {name: header.index(name)
               for name in IMU_REQUIRED + IMU_ORIENTATION if name in header}
        missing = [n for n in IMU_REQUIRED if n not in col]
        if missing:
            sys.exit(f"error: {src} is missing column(s) "
                     f"{', '.join(missing)}; expected the sensor_msgs/Imu "
                     "layout: " + ", ".join(IMU_REQUIRED + IMU_ORIENTATION))
        has_ori = all(n in col for n in IMU_ORIENTATION)

        rows = []
        for row in reader:
            if len(row) < len(header):
                continue
            quat = [float(row[col[n]]) for n in IMU_ORIENTATION] if has_ori else None
            rows.append((
                float(row[col["timestamp"]]),
                [float(row[col["angular_velocity_" + a]]) for a in "xyz"],
                [float(row[col["linear_acceleration_" + a]]) for a in "xyz"],
                quat,
            ))

    if not rows:
        sys.exit(f"error: no IMU rows in {src}")

    ts = [r[0] for r in rows]
    gyr = [(r[0], math.sqrt(sum(c * c for c in r[1]))) for r in rows]
    if any(b <= a for a, b in zip(ts, ts[1:])):
        sys.exit("error: IMU timestamps are not strictly increasing")

    # The orientation quaternion is ignored; make sure that is not silently
    # dropping real information.
    if has_ori:
        identity = [0.0, 0.0, 0.0, 1.0]
        varied = sum(
            1 for r in rows
            if any(abs(r[3][k] - identity[k]) > 1e-6 for k in range(4)))
        if varied:
            print(f"warning: {varied} IMU rows carry a non-identity orientation; "
                  "it is ignored (FAST-LIO estimates attitude itself)",
                  file=sys.stderr)

    # Units: a static or slowly walking platform spends most of its time near
    # 1 g, so the median norm separates g from m/s^2 robustly.
    norms = sorted(math.sqrt(sum(c * c for c in r[2])) for r in rows)
    median_norm = norms[len(norms) // 2]
    if acc_scale is None:
        if 0.75 <= median_norm <= 1.25:
            acc_scale = G_M_S2
            print(f"acceleration median norm {median_norm:.4f} -> assuming g, "
                  f"scaling by {G_M_S2}")
        elif 8.5 <= median_norm <= 11.0:
            acc_scale = 1.0
            print(f"acceleration median norm {median_norm:.4f} -> already m/s^2")
        else:
            sys.exit(f"error: acceleration median norm {median_norm:.4f} matches "
                     "neither g nor m/s^2; pass --acc-scale explicitly")
    else:
        print(f"acceleration scaled by {acc_scale} (--acc-scale)")

    with open(dst, "w") as f:
        f.write("# timestamp gx gy gz ax ay az   (s, rad/s, m/s^2)\n")
        for t, g, a, _ in rows:
            f.write(f"{t:.9f} {g[0]:.6f} {g[1]:.6f} {g[2]:.6f} "
                    f"{a[0] * acc_scale:.6f} {a[1] * acc_scale:.6f} "
                    f"{a[2] * acc_scale:.6f}\n")

    span = ts[-1] - ts[0]
    gaps = [(b - a) for a, b in zip(ts, ts[1:]) if b - a > 0.05]
    print(f"wrote {len(rows)} IMU samples to {dst} "
          f"({len(rows) / span:.0f} Hz over {span:.1f} s)")
    if gaps:
        print(f"warning: {len(gaps)} IMU gaps above 50 ms "
              f"(largest {max(gaps) * 1000:.0f} ms)", file=sys.stderr)
    return ts[0], ts[-1], gyr


def load_xyz_intensity(path):
    """Read an ASCII PointXYZI PCD into an (N, 4) float32 array."""
    header_lines = 0
    with open(path) as f:
        for line in f:
            if line.startswith("DATA"):
                break
            header_lines += 1
    df = pd.read_csv(path, sep=" ", header=None, skiprows=header_lines + 1,
                     usecols=[0, 1, 2, 3], names=["x", "y", "z", "intensity"],
                     dtype=np.float32, engine="c")
    return df.to_numpy()


def list_scans(src_dir):
    scans = []
    for name in os.listdir(src_dir):
        m = STAMP_RE.search(name)
        if m:
            scans.append((float(m.group(1)), name))
    if not scans:
        sys.exit(f"error: no *<unix_seconds>.pcd files in {src_dir}")
    scans.sort()
    for (sa, _), (sb, _) in zip(scans, scans[1:]):
        if sb <= sa:
            sys.exit(f"error: frame timestamps are not strictly increasing "
                     f"({sb} follows {sa})")
    return scans


def check_sensor_frame(scans, gyr, src_dir, voxel=0.03, gyro_threshold=0.5):
    """Fail loudly if the clouds look pre-registered instead of raw.

    Raw clouds rotate with the sensor: through a window where the gyro
    reports fast rotation, consecutive frames land on different cells (cell
    overlap drops to a few percent already at 0.5 rad/s, i.e. ~1.7 deg per
    113 ms frame). Clouds that an upstream SLAM/odom already transformed
    into a fixed world frame stay aligned through the same window (measured
    0.13-0.15 on a MID-360 capture saved post-registration). That case is
    unrunnable for any LIO: the IMU reports the sensor's real motion while
    the clouds no longer contain it, and the filter reconciles the conflict
    by inventing ghost velocity.
    """
    rates = []
    for (s0, _), (s1, _) in zip(scans, scans[1:]):
        w = [g for t, g in gyr if s0 <= t < s1]
        rates.append(float(np.mean(w)) if w else 0.0)
    top = sorted(range(len(rates)), key=lambda i: -rates[i])[:10]
    top = [i for i in top if rates[i] >= gyro_threshold]
    if not top:
        print("note: no frame window with mean |gyr| >= "
              f"{gyro_threshold} rad/s; the clouds could not be screened "
              "for pre-registration (scene never rotated fast enough)")
        return
    overlaps = []
    for i in top:
        a = load_xyz_intensity(os.path.join(src_dir, scans[i][1]))[:, :3]
        b = load_xyz_intensity(os.path.join(src_dir, scans[i + 1][1]))[:, :3]
        ka = set(map(tuple, np.floor(a / voxel).astype(np.int64)))
        kb = set(map(tuple, np.floor(b / voxel).astype(np.int64)))
        overlaps.append(len(ka & kb) / len(ka))
    med = float(np.median(overlaps))
    print(f"sensor-frame screen: cell overlap {min(overlaps):.2f}..{max(overlaps):.2f} "
          f"over {len(top)} fast-rotation pairs (median {med:.2f}, "
          f"|gyr| up to {max(rates[i] for i in top):.2f} rad/s)")
    if med > 0.10:
        sys.exit(
            "error: consecutive clouds stay aligned through windows where the "
            f"gyro reports up to {max(rates[i] for i in top):.1f} rad/s "
            f"(cell overlap {med:.2f} vs a few percent for raw clouds). "
            "These clouds were probably saved AFTER an upstream SLAM or odom "
            "transformed them into a fixed world frame. No LIO can run on "
            "that: the IMU measures the sensor's real motion, the clouds no "
            "longer contain it. Re-export the raw sensor-frame topic "
            "(and ideally the per-point times), or accumulate the clouds at "
            "identity as an already-registered map.")


def convert_scans(src_dir, scans, dst_dir, span_override, stamp_shift):
    """Rewrite each source cloud with synthesized per-point times."""
    os.makedirs(dst_dir, exist_ok=True)

    # Span of frame k: the interval to frame k+1 (the chunks tile the capture
    # stream, so a chunk holds about one interval of returns).
    spans = [b - a for (a, _), (b, _) in zip(scans, scans[1:])]
    spans.append(spans[-1])                     # last frame: reuse the previous

    n_points_total = 0
    for (stamp, name), span in zip(scans, spans):
        if span_override is not None:
            span = span_override
        if span <= 0:
            sys.exit(f"error: frame at {stamp} has a non-positive span")
        pts = load_xyz_intensity(os.path.join(src_dir, name))
        n = len(pts)
        if n < 2:
            print(f"warning: {name} has {n} points; skipped", file=sys.stderr)
            continue
        out = np.empty(n, dtype=PCD_RECORD)
        out["x"], out["y"], out["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        out["intensity"] = pts[:, 3]
        # Capture order -> uniform offsets across the frame interval, in
        # seconds (timestamp_unit: 0 in the runner's config).
        out["time"] = np.linspace(0, span, n, dtype=np.float32)
        out["ring"] = 0

        out_name = f"{stamp - stamp_shift:.6f}.pcd"
        with open(os.path.join(dst_dir, out_name), "wb") as f:
            f.write(PCD_HEADER.format(n=n).encode())
            f.write(out.tobytes())
        n_points_total += n

    stamps = [s - stamp_shift for s, _ in scans[1:]] + [scans[-1][0] - stamp_shift]
    span_all = scans[-1][0] - scans[0][0]
    gaps = [(b - a) for a, b in zip(stamps, stamps[1:]) if b - a > 0.2]
    print(f"wrote {len(scans)} scans ({n_points_total / len(scans):.0f} pts avg) "
          f"to {dst_dir} ({len(scans) / span_all:.2f} Hz over {span_all:.1f} s)")
    if gaps:
        print(f"note: {len(gaps)} frame gaps above 0.2 s "
              f"(largest {max(gaps) * 1000:.0f} ms)", file=sys.stderr)
    return scans[0][0] - stamp_shift, scans[-1][0] - stamp_shift


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="src", required=True,
                    help="directory holding imu_data.csv and the pcd/ folder")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--imu", default="imu_data.csv",
                    help="IMU csv file name under --in (default %(default)s)")
    ap.add_argument("--acc-scale", type=float, default=None,
                    help="multiply acceleration by this; default auto-detects "
                         "g (~1) vs m/s^2 (~9.81) from the median norm")
    ap.add_argument("--span", type=float, default=None,
                    help="constant per-point time span in seconds; default "
                         "uses the interval to the next frame")
    ap.add_argument("--stamp-shift", type=float, default=0.0,
                    help="seconds to subtract from each frame stamp (use the "
                         "mean interval if the exporter stamped frame ENDS)")
    args = ap.parse_args()

    imu_src = os.path.join(args.src, args.imu)
    pcd_src = os.path.join(args.src, "pcd")
    if not os.path.isfile(imu_src):
        sys.exit(f"error: {imu_src} not found")
    if not os.path.isdir(pcd_src):
        sys.exit(f"error: {pcd_src} not found")

    os.makedirs(args.out, exist_ok=True)
    imu_t0, imu_t1, gyr = convert_imu(imu_src, os.path.join(args.out, "imu.txt"),
                                      args.acc_scale)
    scans = list_scans(pcd_src)
    check_sensor_frame(scans, gyr, pcd_src)
    scan_t0, scan_t1 = convert_scans(pcd_src, scans, os.path.join(args.out, "pcd"),
                                     args.span, args.stamp_shift)

    # The filter needs IMU data across each scan window [stamp, stamp+span].
    lead = scan_t0 - imu_t0
    tail = imu_t1 - scan_t1
    if lead < 0.05:
        print(f"warning: IMU starts only {lead * 1000:.0f} ms before the first "
              "scan; the first scan's window may be incomplete", file=sys.stderr)
    if tail < 0.15:
        print(f"warning: IMU ends only {tail * 1000:.0f} ms after the last "
              "scan; the last scan's window may be incomplete", file=sys.stderr)
    print(f"IMU lead-in {lead * 1000:.0f} ms, tail {tail * 1000:.0f} ms")


if __name__ == "__main__":
    main()