#!/usr/bin/env python3
"""bbs_scan_capture.py — 从运行中的 point_lio 抓 /cloud_registered (LIO odom 系, 已重力对齐)
累积 ~3 秒 → 估地面层 → 剥地面带 (地面+0.15 ~ +2.8, 复刻 Phase 0b 配方) → 15m 截断
→ 二进制 XYZ PCD 给 bbs_pose 做全局重定位。
在 foxy-live 容器内跑: python3 /home/mu/foxy_ws/bbs_scan_capture.py [帧数] [输出路径]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

N_FRAMES = int(sys.argv[1]) if len(sys.argv) > 1 else 30
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/mu/maps/bbs_live_scan.pcd"


class Cap(Node):
    def __init__(self):
        super().__init__("bbs_scan_capture")
        self.pts = None
        self.n = 0
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb, 20)

    def cb(self, m):
        step = m.point_step
        raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, step)
        offs = {f.name: f.offset for f in m.fields}

        def f32(name):
            o = offs[name]
            return raw[:, o:o + 4].copy().view(np.float32).ravel()
        xyz = np.stack([f32("x"), f32("y"), f32("z")], axis=1).astype(np.float64)
        self.pts = xyz if self.pts is None else np.concatenate([self.pts, xyz], axis=0)
        self.n += 1


def write_xyz_pcd(path, pts):
    arr = np.ascontiguousarray(pts.astype(np.float32))
    n = len(arr)
    with open(path, "wb") as f:
        f.write(b"# .PCD v0.7 - Point Cloud Data file format\n")
        f.write(b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n".encode())
        f.write(arr.tobytes())


def main():
    rclpy.init()
    node = Cap()
    t0 = time.time()
    while node.n < N_FRAMES and time.time() - t0 < 12:
        rclpy.spin_once(node, timeout_sec=0.5)
    pts = node.pts
    if pts is None:
        print("无帧! cloud_registered 未发布")
        sys.exit(1)
    print(f"[capture] frames={node.n} pts={len(pts)} "
          f"bbox x[{pts[:,0].min():.1f},{pts[:,0].max():.1f}] "
          f"y[{pts[:,1].min():.1f},{pts[:,1].max():.1f}] z[{pts[:,2].min():.1f},{pts[:,2].max():.1f}]")
    # 地面层 = z 低分位; 剥地面带 (Phase 0b 配方: +0.15 ~ +2.8)
    gz = np.percentile(pts[:, 2], 5)
    m = (pts[:, 2] > gz + 0.15) & (pts[:, 2] < gz + 2.8)
    keep = pts[m]
    print(f"[strip] 地面层z={gz:.2f}, 剥除后 {len(keep)} pts ({len(keep)/len(pts)*100:.0f}%)")
    # 15m 范围截断 (与 bbs_pose max_range 一致, 先截让 src_leaf 更有效)
    d2 = (keep ** 2).sum(axis=1)
    keep = keep[d2 < 15.0 ** 2]
    print(f"[range] 15m 内 {len(keep)} pts")
    write_xyz_pcd(OUT, keep)
    print(f"[done] {OUT} {len(keep)} pts")


if __name__ == "__main__":
    main()
