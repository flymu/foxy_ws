#!/usr/bin/env python3
"""viz_capture.py — 定位可视化数据采集（容器内跑，给 viz_plot.py 出图用）

抓 N 秒 /cloud_registered（odom 系），用 loc 当前 T_mb 与 odom T_ob 反解
T_mo = T_mb · T_ob⁻¹，把扫描变换进 map 系与校园图同框对比——一眼看出
"当前看到的场景是否真的落在地图上"（定位效果的可视证据）。

用法:
  python3 viz_capture.py [--secs 2] [--outdir /home/mu/bags/viz_cap]
产物: viz_cap_<HHMMSS>.npz（scan_odom/scan_map/loc_pose/t_mo/path）
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def mat_from_quat_pos(p, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy, p[0]],
        [xy + wz, 1 - (xx + zz), yz - wx, p[1]],
        [xz - wy, yz + wx, 1 - (xx + yy), p[2]],
        [0, 0, 0, 1]])


class Cap(Node):
    def __init__(self, secs):
        super().__init__("viz_capture")
        self.secs = secs
        self.t0 = time.time()
        self.clouds = []
        self.loc = None      # (pos, quat)
        self.ob = None
        self.path = None
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, "/cloud_registered", self.on_cloud, be)
        self.create_subscription(Odometry, "/loc/pose", self.on_loc, be)
        self.create_subscription(Odometry, "/Odometry", self.on_ob, be)
        self.create_subscription(Path, "/loc/path", self.on_path, be)

    def on_cloud(self, m):
        off = {f.name: f.offset for f in m.fields if f.name in ("x", "y", "z")}
        if len(off) < 3:
            return
        raw = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
        pts = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
                        for k in ("x", "y", "z")], axis=1)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts):
            self.clouds.append(pts)

    def on_loc(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.loc = ((p.x, p.y, p.z), q)

    def on_ob(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.ob = ((p.x, p.y, p.z), q)

    def on_path(self, m):
        self.path = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
                     for ps in m.poses]

    def done(self):
        return time.time() - self.t0 >= self.secs


def main():
    secs, outdir = 2.0, "/home/mu/bags/viz_cap"
    args = sys.argv[1:]
    for i, a in enumerate(args):
        if a == "--secs":
            secs = float(args[i + 1])
        elif a == "--outdir":
            outdir = args[i + 1]
    rclpy.init()
    node = Cap(secs)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.3)
    if not node.clouds or node.loc is None or node.ob is None:
        print(f"[viz_capture] 数据不足: cloud={len(node.clouds)} loc={node.loc is not None} "
              f"odom={node.ob is not None}", flush=True)
        sys.exit(2)
    scan_odom = np.concatenate(node.clouds)
    key = np.floor(scan_odom / 0.15).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    scan_odom = scan_odom[idx]
    t_mb = mat_from_quat_pos(node.loc[0], node.loc[1])
    t_ob = mat_from_quat_pos(node.ob[0], node.ob[1])
    t_mo = t_mb @ np.linalg.inv(t_ob)
    scan_map = (t_mo[:3, :3] @ scan_odom.T).T + t_mo[:3, 3]
    path = np.array(node.path) if node.path else np.zeros((0, 3))
    import os
    os.makedirs(outdir, exist_ok=True)
    ts = time.strftime("%H%M%S")
    out = f"{outdir}/viz_cap_{ts}.npz"
    np.savez_compressed(out, scan_odom=scan_odom.astype(np.float32),
                        scan_map=scan_map.astype(np.float32),
                        loc_pose=np.array([*node.loc[0]], dtype=np.float64),
                        t_mb=t_mb, t_ob=t_ob, t_mo=t_mo, path=path.astype(np.float32))
    print(f"[viz_capture] {out}: scan {len(scan_odom)} pts "
          f"loc=({node.loc[0][0]:.2f},{node.loc[0][1]:.2f}) path {len(path)} 点", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
