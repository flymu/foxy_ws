#!/usr/bin/env python3
"""health_pose_probe.py — 一发探针: /loc/health 三态 + /loc/pose 位姿一行打印
用法: python3 health_pose_probe.py  （输出: OK|lio_ndt|0.12|pos=(x,y,z)|yaw=deg）"""
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

rclpy.init()
n = Node("health_pose_probe")
st = {"h": None, "p": None, "q": None}


def cb_h(m):
    st["h"] = m.data


def cb_p(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    st["p"] = (p.x, p.y, p.z)
    st["q"] = (q.w, q.x, q.y, q.z)


n.create_subscription(String, "/loc/health", cb_h, 10)
n.create_subscription(Odometry, "/loc/pose", cb_p, 10)
t0 = time.time()
while time.time() - t0 < 4 and (st["h"] is None or st["p"] is None):
    rclpy.spin_once(n, timeout_sec=0.2)
h = st["h"] or "NO-HEALTH"
if st["p"] is None:
    print(f"{h}|NO-POSE")
else:
    w, x, y, z = st["q"]
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y)))
    print(f"{h}|pos=({st['p'][0]:.3f},{st['p'][1]:.3f},{st['p'][2]:.3f})|yaw={yaw:.1f}")
rclpy.shutdown()
