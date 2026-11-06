#!/usr/bin/env python3
"""veto_window_capture.py — 抓 N 帧 /livox/points2 存 npz（veto 视角诊断用）
用法: python3 veto_window_capture.py [帧数=8] → /home/unitree/veto_frames.npz
数组: pts[N,W,3] 传感器系点（含 frame 索引第4列），另有 status 字符串"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

N_FRAMES = int(sys.argv[1]) if len(sys.argv) > 1 else 8

frames = []
rclpy.init()
n = Node("veto_window_capture")
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=2)


def cb(m):
    if len(frames) < N_FRAMES:
        a = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
        frames.append(np.ascontiguousarray(a))


n.create_subscription(PointCloud2, "/livox/points2", cb, q)
t0 = time.monotonic()
while len(frames) < N_FRAMES and time.monotonic() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
rclpy.shutdown()
if not frames:
    print("无云")
    sys.exit(1)
print("抓到 %d 帧" % len(frames))
out = []
for i, f in enumerate(frames):
    idx = np.full((f.shape[0], 1), i, dtype=np.float32)
    out.append(np.hstack([f, idx]))  # x y z refl frame_id（帧点数不一，纵向拼）
np.savez("/home/unitree/veto_frames.npz",
         pts=np.vstack(out), n=len(frames))
print("已存 /home/unitree/veto_frames.npz")