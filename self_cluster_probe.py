#!/usr/bin/env python3
"""self_cluster_probe.py — 解剖狗格 126 点自簇: LIO系距离/方位/z分布/逐帧稳定性"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
n = Node("self_cluster_probe")
frames = []


def cb(m):
    if len(frames) < 5:
        dtype = np.dtype([(f.name, "<f4") for f in m.fields[:4]])
        arr = np.frombuffer(bytes(m.data), dtype=dtype, count=m.width * m.height)
        frames.append(np.stack([arr["x"], arr["y"], arr["z"]], axis=1))


n.create_subscription(PointCloud2, "/cloud_registered", cb, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8 and len(frames) < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"{len(frames)} 帧")
for i, pts in enumerate(frames):
    r = np.linalg.norm(pts, axis=1)  # LIO 系原点即传感器
    near = pts[r < 1.2]
    if not len(near):
        print(f"帧{i}: 无 <1.2m 点")
        continue
    az = np.degrees(np.arctan2(near[:, 1], near[:, 0]))
    print(f"帧{i}: <1.2m 点数={len(near)} | range min/med/max="
          f"{np.min(np.linalg.norm(near,axis=1)):.3f}/{np.median(np.linalg.norm(near,axis=1)):.3f}/"
          f"{np.max(np.linalg.norm(near,axis=1)):.3f} | z min/med/max={near[:,2].min():.3f}/"
          f"{np.median(near[:,2]):.3f}/{near[:,2].max():.3f}")
    print(f"     方位直方图(30°桶): {np.bincount((az.astype(int)+360)//30, minlength=12).tolist()}")
    if i == len(frames) - 1:
        for pt in near[np.argsort(np.linalg.norm(near, axis=1))][:8]:
            print(f"     最近点: ({pt[0]:+.3f},{pt[1]:+.3f},{pt[2]:+.3f}) r={np.linalg.norm(pt):.3f}")
        uniq, cnt = np.unique(np.round(near, 2), axis=0, return_counts=True)
        top = np.argsort(-cnt)[:6]
        print("     去重top(坐标@计数):", [(uniq[j].tolist(), int(cnt[j])) for j in top])
rclpy.shutdown()
