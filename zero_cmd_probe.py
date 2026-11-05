#!/usr/bin/env python3
"""zero_cmd_probe.py — 净空勘察：向 /cmd_vel_raw 发全零 Twist（20Hz×N 秒）驱动
veto 评估近场。桥不开狗不动，纯旁路勘察。结束打印最后一条 /veto/status。
用法: python3 zero_cmd_probe.py [秒数=10]
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
rclpy.init()
n = Node("zero_probe")
pub = n.create_publisher(Twist, "/cmd_vel_raw", 5)
last = []
n.create_subscription(String, "/veto/status", lambda m: last.append(m.data), 5)
t0 = time.monotonic()
while time.monotonic() - t0 < DUR:
    pub.publish(Twist())
    rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.03)
print(last[-1] if last else "NO STATUS")
