#!/usr/bin/env python3
"""bridge_status_probe.py — 读 /go2/status（桥心跳，String）+ /go2/cmd_vel 当前值"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rclpy.init()
n = Node("bridge_status_probe")
got = {"s": [], "v": []}
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5)
n.create_subscription(String, "/go2/status", lambda m: got["s"].append(m.data), 10)
n.create_subscription(Twist, "/go2/cmd_vel", lambda m: got["v"].append(
    (m.linear.x, m.angular.z)), be)
t0 = time.time()
while time.time() - t0 < 6 and len(got["s"]) < 3:
    rclpy.spin_once(n, timeout_sec=0.3)
print(f"go2/status: {len(got['s'])} 帧, 最新: {got['s'][-1] if got['s'] else '无'}")
print(f"go2/cmd_vel: {len(got['v'])} 帧, 最新: {got['v'][-1] if got['v'] else '无'}")