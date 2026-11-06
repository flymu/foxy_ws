#!/usr/bin/env python3
"""loc_twist_probe.py — 读 /loc/pose 的 twist（规划器 setOdometry 的速度来源）
狗静止时 |v| 应≈0；非零=喂给规划器的速度有鬼（动态可行性拒绝的头号嫌疑人）"""
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("loc_twist_probe")
vs, ps = [], []


def cb(m):
    t = m.twist.twist.linear
    w = m.twist.twist.angular
    p = m.pose.pose.position
    vs.append((t.x, t.y, t.z, w.z))
    ps.append((p.x, p.y, p.z))


n.create_subscription(Odometry, "/loc/pose", cb, 20)
t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"{len(vs)} 帧 /loc/pose (6s)")
if vs:
    mags = sorted(math.hypot(v[0], v[1]) for v in vs)
    print(f"|v_xy| 最小/中位/最大: {mags[0]:.4f}/{mags[len(mags)//2]:.4f}/{mags[-1]:.4f} m/s")
    print(f"|w_z| 最大: {max(abs(v[3]) for v in vs):.4f} rad/s")
    print(f"最新 pos: ({ps[-1][0]:.3f},{ps[-1][1]:.3f},{ps[-1][2]:.3f}) "
          f"twist: vx={vs[-1][0]:.3f} vy={vs[-1][1]:.3f} wz={vs[-1][3]:.3f}")
    xs = [p[0] for p in ps]
    ys = [p[1] for p in ps]
    print(f"pos 抖动范围: x[{min(xs):.3f},{max(xs):.3f}] y[{min(ys):.3f},{max(ys):.3f}]")
rclpy.shutdown()
