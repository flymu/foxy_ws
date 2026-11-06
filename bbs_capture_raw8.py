#!/usr/bin/env python3
"""bbs_capture_raw8.py — 复刻 map_localization_node 的子图: 8帧 /cloud_registered 原样 (不剥地面/不截断)"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/mu/maps/bbs_raw8.pcd"

class Cap(Node):
    def __init__(self):
        super().__init__("bbs_cap_raw8")
        self.frames = []
        self.n = 0
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb, 20)
    def cb(self, m):
        step = m.point_step
        raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, step)
        offs = {f.name: f.offset for f in m.fields}
        def f32(nm):
            o = offs[nm]
            return raw[:, o:o+4].copy().view(np.float32).ravel()
        xyz = np.stack([f32("x"), f32("y"), f32("z")], axis=1).astype(np.float64)
        if len(self.frames) < 8:
            self.frames.append(xyz)
            self.n += 1

def write_xyz_pcd(path, pts):
    arr = np.ascontiguousarray(pts.astype(np.float32))
    n = len(arr)
    with open(path, "wb") as f:
        f.write(b"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n".encode())
        f.write(arr.tobytes())

rclpy.init()
node = Cap()
t0 = time.time()
while node.n < 8 and time.time() - t0 < 12:
    rclpy.spin_once(node, timeout_sec=0.5)
pts = np.concatenate(node.frames, axis=0)
print(f"[raw8] frames={node.n} pts={len(pts)}")
write_xyz_pcd(OUT, pts)
print(f"[done] {OUT}")
