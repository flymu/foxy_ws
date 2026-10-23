#!/usr/bin/env python3
"""Convert a Livox-SDK2 text dump into the layout fastlio_offline expects.

Input (as produced by the Livox-SDK2 sample recorder):

  scan_000000_<ms>ms.txt   header comments, then 'unix_ts x y z reflectivity'
  imu_data.txt             '# ...' comments, then 'unix_ts gx gy gz ax ay az'

Output:

  <out>/pcd/<scan_start_ts>.pcd   binary PCD with fields x y z intensity time ring
  <out>/imu.txt                   same columns, acceleration in m/s^2

Two fixes are applied on the way through; see --help output of each flag.

  python3 tools/livox_txt_to_pcd.py --in ~/Livox-SDK2/my_slam_data --out /tmp/mid360
"""
import argparse
import os
import struct
import sys

# Points arrive in fixed-size hardware packets. The recorder writes the packet's
# total duration (`time_interval`) as the *per-point* step, so intra-packet
# offsets come out POINTS_PER_PACKET times too large and every packet boundary
# looks like a backwards time jump.
POINTS_PER_PACKET = 96

# Livox MID-360 reports acceleration in g, despite the header claiming m/s^2.
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


def parse_scan(path):
    """Return (frame_start, frame_end, [(ts, x, y, z, refl), ...])."""
    frame_start = frame_end = None
    pts = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                if "Timestamp Start:" in line:
                    frame_start = float(line.split(":")[1].split()[0])
                elif "Timestamp End:" in line:
                    frame_end = float(line.split(":")[1].split()[0])
                continue
            c = line.split()
            if len(c) < 5:
                continue
            pts.append((float(c[0]), float(c[1]), float(c[2]), float(c[3]), float(c[4])))
    return frame_start, frame_end, pts


def fix_timestamps(pts, declared_span=None):
    """Rebuild per-point timestamps by rescaling the intra-packet step.

    Packets are recovered from the backwards jumps in the recorded timestamps.
    Within each packet the recorded step is the packet's whole duration, so the
    true offset of point k is k * step / POINTS_PER_PACKET.

    Frames whose timestamps are already correct are returned unchanged: the
    signature of the broken recorder is the backwards jump at every packet
    boundary, so a frame with no jumps at all and a span comparable to the
    frame duration the header declares needs no repair. Rescaling such a
    frame collapses it to under a millisecond and silently destroys motion
    undistortion.
    """
    if not pts:
        return []

    # Split at every backwards jump: those are the packet boundaries.
    packets = []
    start = 0
    for i in range(1, len(pts)):
        if pts[i][0] < pts[i - 1][0]:
            packets.append((start, i))
            start = i
    packets.append((start, len(pts)))

    if len(packets) == 1:
        # No packet boundaries: either the recorder wrote proper per-point
        # timestamps, or the whole frame is one pathological packet. Tell them
        # apart by the span: correct points stay inside the declared frame
        # duration, while a single broken packet spans POINTS_PER_PACKET times
        # too long. Without a declared duration, trust the monotonicity.
        span = pts[-1][0] - pts[0][0]
        if declared_span is None or span <= 1.5 * declared_span:
            return pts

    fixed = []
    for lo, hi in packets:
        base = pts[lo][0]
        n = hi - lo
        # The recorded step is constant inside a packet; take the first gap, and
        # fall back to spanning the packet if it is a single point.
        step = (pts[lo + 1][0] - base) if n > 1 else 0.0
        per_point = step / POINTS_PER_PACKET
        for k in range(n):
            ts, x, y, z, refl = pts[lo + k]
            fixed.append((base + k * per_point, x, y, z, refl))
    return fixed


def write_pcd(path, pts, frame_start):
    """Write points with `time` as the offset in seconds from frame_start."""
    rows = []
    for ts, x, y, z, refl in pts:
        off = ts - frame_start
        if off < 0.0:
            off = 0.0
        rows.append((x, y, z, refl, off))
    # velodyne_handler decides whether per-point times are usable by looking at
    # the LAST point, so the cloud has to be in time order.
    rows.sort(key=lambda r: r[4])

    with open(path, "wb") as f:
        f.write(PCD_HEADER.format(n=len(rows)).encode())
        pack = struct.Struct("<fffffH").pack
        for x, y, z, refl, off in rows:
            f.write(pack(x, y, z, refl, off, 0))
    return len(rows)


def convert_imu(src, dst, scale):
    n = 0
    with open(src) as fin, open(dst, "w") as fout:
        fout.write("# timestamp gx gy gz ax ay az   (s, rad/s, m/s^2)\n")
        for line in fin:
            if line.startswith("#"):
                continue
            c = line.split()
            if len(c) < 7:
                continue
            ts = float(c[0])
            g = [float(v) for v in c[1:4]]
            a = [float(v) * scale for v in c[4:7]]
            fout.write(
                f"{ts:.6f} {g[0]:.6f} {g[1]:.6f} {g[2]:.6f} "
                f"{a[0]:.6f} {a[1]:.6f} {a[2]:.6f}\n"
            )
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="src", required=True, help="directory holding the txt dump")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--no-fix-time", action="store_true",
                    help="keep the recorded per-point timestamps as they are "
                         "(they span ~145 ms per 100 ms frame and are non-monotonic; "
                         "frames that already look correct are never rescaled)")
    ap.add_argument("--acc-scale", type=float, default=G_M_S2,
                    help="multiply acceleration by this (default %(default)s, "
                         "converting g to m/s^2; pass 1.0 if your dump is already m/s^2)")
    args = ap.parse_args()

    pcd_dir = os.path.join(args.out, "pcd")
    os.makedirs(pcd_dir, exist_ok=True)

    scans = sorted(f for f in os.listdir(args.src)
                   if f.startswith("scan_") and f.endswith(".txt"))
    if not scans:
        sys.exit(f"error: no scan_*.txt found in {args.src}")

    total = 0
    for i, name in enumerate(scans):
        frame_start, frame_end, pts = parse_scan(os.path.join(args.src, name))
        if frame_start is None:
            print(f"warning: {name} has no start timestamp, skipping", file=sys.stderr)
            continue
        if not pts:
            print(f"warning: {name} has no points, skipping", file=sys.stderr)
            continue
        if not args.no_fix_time:
            declared_span = (frame_end - frame_start) if frame_end else None
            pts = fix_timestamps(pts, declared_span)
        n = write_pcd(os.path.join(pcd_dir, f"{frame_start:.6f}.pcd"), pts, frame_start)
        total += n
        if i % 100 == 0:
            print(f"  [{i + 1}/{len(scans)}] {name} -> {n} points")

    imu_src = os.path.join(args.src, "imu_data.txt")
    n_imu = convert_imu(imu_src, os.path.join(args.out, "imu.txt"), args.acc_scale)

    print(f"wrote {len(scans)} scans ({total} points) to {pcd_dir}")
    print(f"wrote {n_imu} IMU samples to {os.path.join(args.out, 'imu.txt')}")


if __name__ == "__main__":
    main()
