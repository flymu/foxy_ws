#!/usr/bin/env python3
"""refl_probe.py — 采 3 帧打反射率基准：地板/远处真结构/近场频带 各自多少"""
import time

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

N_UP = np.array([-0.7330, 0.0001, 0.6802]); N_UP /= np.linalg.norm(N_UP)
H = 0.486
F_AX = np.array([1.0, 0.0, 0.0]); F_AX = F_AX - (F_AX @ N_UP) * N_UP; F_AX /= np.linalg.norm(F_AX)
L_AX = np.cross(N_UP, F_AX)

rclpy.init()
n = Node('refl_probe')
got = []


def on_cloud(m):
    if len(got) >= 3 or len(m.points) < 2000:
        return
    P = np.array([[q.x, q.y, q.z] for q in m.points])
    R = np.array([q.reflectivity for q in m.points], np.float64)
    got.append((P, R))


q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=10)
n.create_subscription(CustomMsg, '/livox/lidar', on_cloud, q)
t0 = time.monotonic()
while len(got) < 3 and time.monotonic() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.5)

if not got:
    print('没收到云')
else:
    P = np.concatenate([g[0] for g in got]); R = np.concatenate([g[1] for g in got])
    h = P @ N_UP + H
    d = np.hypot(P @ F_AX, P @ L_AX)
    for lab, m in (('全部点', np.ones(len(P), bool)),
                   ('地板 h<0.1', h < 0.1),
                   ('远处真结构 d 2-5m', (d > 2) & (d < 5) & (h > 0.1)),
                   ('近场频带 d<1.25 h0.2-0.75', (d < 1.25) & (h > 0.2) & (h < 0.75))):
        if m.sum():
            print('%-22s n=%6d  反射率 p10=%3.0f p50=%3.0f p90=%3.0f' %
                  (lab, m.sum(), *np.percentile(R[m], [10, 50, 90])), flush=True)
