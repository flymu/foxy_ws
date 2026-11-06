#!/usr/bin/env python3
"""cloud_dogcell_probe.py — 诊断 'drone is in obstacle': 探 /cloud_registered
在 狗格/目标格/走廊中点 的占据情况。云先经 loc_state.txt 的 t_mo 搬到 map 系分析。"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

# ---- 读 t_mo（行主序 3x4，12 个数）----
row = None
with open("/home/mu/maps/loc_state.txt") as f:
    for line in f:
        if line.startswith("t_mo"):
            row = [float(v) for v in line.split()[1:13]]
R = np.array([[row[0], row[1], row[2]],
              [row[4], row[5], row[6]],
              [row[8], row[9], row[10]]])
t = np.array([row[3], row[7], row[11]])

CELLS = {
    "狗格(0.71,0.16)": np.array([0.71, 0.16, -0.03]),
    "目标格(-1.16,0.04)": np.array([-1.156, 0.044, -0.03]),
    "走廊中点(0,0)": np.array([0.0, 0.0, -0.03]),
}

rclpy.init()
n = Node("cloud_dogcell_probe")
got = {"msg": None, "n": 0}


def cb(m):
    got["n"] += 1
    if got["n"] >= 3:  # 跳过开头两帧
        got["msg"] = m


n.create_subscription(PointCloud2, "/cloud_registered", cb, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8 and got["msg"] is None:
    rclpy.spin_once(n, timeout_sec=0.2)
m = got["msg"]
if m is None:
    print("无 /cloud_registered 帧流！")
    raise SystemExit(1)

dtype = np.dtype([(f.name, "<f4") for f in m.fields[:4]])
arr = np.frombuffer(bytes(m.data), dtype=dtype, count=m.width * m.height)
pts_lio = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
pts = (R @ pts_lio.T).T + t  # → map 系
print(f"帧: {len(pts)} 点（已搬到 map 系）")
zs = np.sort(pts[:, 2])
q = lambda p: zs[min(int(p * len(zs)), len(zs) - 1)]
print(f"map系 z 分布: 1%={q(0.01):.2f} 5%={q(0.05):.2f} 50%={q(0.50):.2f} 95%={q(0.95):.2f} max={zs[-1]:.2f}")

for name, c in CELLS.items():
    d = np.linalg.norm(pts[:, :2] - c[:2], axis=1)
    near = pts[d < 0.6]
    if not len(near):
        print(f"{name}: r<0.6m 无点（空格）")
        continue
    print(f"{name}: r<0.6m 点数={len(near)} | z: min={near[:,2].min():.2f} "
          f"med={np.median(near[:,2]):.2f} max={near[:,2].max():.2f}")
    # 分高度层统计（狗传感器高 -0.03）
    for lo, hi, tag in [(-10, -1.0, "z<-1.0(地面下方/截掉)"), (-1.0, -0.35, "地板层"),
                        (-0.35, -0.13, "低位"), (-0.13, 0.07, "狗高度带±0.1"),
                        (0.07, 10, "高位")]:
        c2 = near[(near[:, 2] >= lo) & (near[:, 2] < hi)]
        if len(c2):
            print(f"    {tag}: {len(c2)} 点  样例={[tuple(np.round(p,2)) for p in c2[:3]]}")
rclpy.shutdown()
