#!/usr/bin/env python3
"""match_probe.py — 判定规划器端 DDS 发现是否还活着:
新进程建 /plan 与 /cloud_registered 订阅, 每 2s 报匹配到的远端发布者数。
匹配数>0 = 对端发现正常; 恒 0 = 对端 metatraffic 被毒包风暴搞死。"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2

rclpy.init()
n = Node("match_probe")
plan_sub = n.create_subscription(Path, "/plan", lambda m: None, 10)
cloud_sub = n.create_subscription(PointCloud2, "/cloud_registered",
                                  lambda m: None, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.5)
    el = time.time() - t0
    if int(el * 2) != int((el - 0.5) * 2):  # 每~2s 打一行
        print(f"t={el:.0f}s  /plan 匹配发布者数={plan_sub.get_publisher_count()}  "
              f"/cloud_registered 匹配数={cloud_sub.get_publisher_count()}", flush=True)
rclpy.shutdown()
