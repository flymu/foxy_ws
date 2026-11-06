#!/usr/bin/env python3
"""veto_status_grab.py — 一次性拉双版本否决层状态（Orin 上用，ros2 CLI 不可信时代替品）
打印 /veto/status(Python 生产) 与 /veto/status_shadow(C++) 各一条后退出。"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

got = {}
rclpy.init()
n = Node("veto_status_grab")


def mk(side, topic):
    def cb(m):
        if side not in got:
            got[side] = m.data
    n.create_subscription(String, topic, cb, 5)


mk("py", "/veto/status")
mk("cpp", "/veto/status_shadow")
t0 = time.monotonic()
while len(got) < 2 and time.monotonic() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
for k in ("py", "cpp"):
    print(k + ": " + got.get(k, "无"))
rclpy.shutdown()
