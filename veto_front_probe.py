#!/usr/bin/env python3
"""veto_front_probe.py — 一次快照：/livox/points2 → veto 入库 → 报告拦截窗内
每个方位簇的距离/高度，人眼比对"那里到底有什么"（2026-11-07 蠕行复查用）"""
import collections
import math
import sys
import time

sys.path.insert(0, "/home/unitree")
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import veto_core

P = {"data": None}
rclpy.init()
n = Node("veto_front_probe")
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=1)


def cb(m):
    if P["data"] is None:
        a = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
        P["data"] = np.ascontiguousarray(a[:, :3])


n.create_subscription(PointCloud2, "/livox/points2", cb, q)
t0 = time.monotonic()
while P["data"] is None and time.monotonic() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.1)
rclpy.shutdown()
if P["data"] is None:
    print("无云")
    sys.exit(1)

g = veto_core.Geom(veto_core.PRIOR_N, veto_core.PRIOR_H, body_along=0.05)
prm = veto_core.Params()
sl = veto_core.ingest(P["data"], g, prm)
print("入库 %d 点（频带+非自体+<1.25m）" % len(sl))

bins = collections.defaultdict(list)
for dh, az in sl.tolist():
    bins[int(math.degrees(az) // 10) * 10].append(dh)
print("-- 方位簇（10° 一格，+左 -右）--")
for k in sorted(bins):
    v = bins[k]
    print("%+4d°: %4d 点, dh %.2f-%.2f (最近 %.2f)"
          % (k, len(v), min(v), max(v), min(v)))

print("-- 拦截半径内点明细（最近 12 个）--")
sel = []
for p in P["data"].tolist():
    px = np.array(p)
    if abs(px[0]) < 0.25 and abs(px[1]) < 0.15 and -0.30 < px[2] < 0.35:
        continue  # 自体掩模（与 veto_core.ingest 同款），别把狗自己的点当障碍
    hgt = float(px @ g.n + g.h)
    if not (prm.band[0] < hgt < prm.band[1]):
        continue
    rel = px - g.body_c
    vh = rel - float(rel @ g.n) * g.n
    dh = math.hypot(float(vh @ g.f), float(vh @ g.l))
    if dh >= 1.25:
        continue
    az = math.atan2(float(vh @ g.l), float(vh @ g.f))
    if dh < (prm.sector_r if abs(az) < prm.sector_half else prm.ring_r):
        sel.append((dh, az, hgt))
sel.sort()
for dh, az, hgt in sel[:12]:
    print("拦: dh=%.2f az=%+.0f° 离地%.2fm  (前向%.2fm 侧向%+.2fm)"
          % (dh, math.degrees(az), hgt, dh * math.cos(az), dh * math.sin(az)))
