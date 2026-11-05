#!/usr/bin/env python3
"""lag_probe.py — 量 /livox/lidar 到达滞后与帧间隔（BEST_EFFORT 直订，foxy 的
ros2 topic hz/delay 订不上 BE 话题）。
到达滞后 = 本机 wall clock - 消息 stamp（站间时钟差 ~80ms 已知，含在里面）。
用法（foxy-live 容器内）: python3 ~/foxy_ws/lag_probe.py
"""
import time

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

rclpy.init()
n = Node('lag_probe')
buf = []


def on_cloud(m):
    buf.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                len(m.points)))


q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=2)
n.create_subscription(CustomMsg, '/livox/lidar', on_cloud, q)
t_start_wall = time.time()
t0 = time.monotonic()
while not buf and time.monotonic() - t0 < 40:      # 跨机 DDS 发现可能 >12s
    rclpy.spin_once(n, timeout_sec=0.5)
if buf:
    disc = buf[0][0] - t_start_wall
    t2 = time.monotonic()
    while time.monotonic() - t2 < 8:                # 发现后再采 8s 有效帧
        rclpy.spin_once(n, timeout_sec=0.5)

if len(buf) < 5:
    print('没收到云: %d 帧（40s，可能驱动断流）' % len(buf))
else:
    arr = np.array(buf)
    print('DDS 发现耗时 %.1fs' % disc)
    lag = arr[:, 0] - arr[:, 1]
    itv = np.diff(arr[:, 0])
    if arr[0, 1] < 1.6e9:
        print('stamp 异常（非 epoch 域）: %.3f，滞后不可量，只看间隔' % arr[0, 1])
    print('帧数=%d 点数/帧 p50=%.0f' % (len(arr), np.median(arr[:, 2])))
    print('到达滞后(本机-stamp): p10=%.3f p50=%.3f p90=%.3f s'
          % tuple(np.percentile(lag, [10, 50, 90])))
    print('到达间隔: p10=%.3f p50=%.3f p90=%.3f s（10Hz 理想 0.100，'
          '>0.2 说明网络/驱动抖动积压）' % tuple(np.percentile(itv, [10, 50, 90])))
