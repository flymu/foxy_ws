#!/usr/bin/env python3
"""ros_logger.py — unitree 用户订阅 /livox/lidar，写 (wall, stamp, npts) 25s"""
import sys
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
OUT = '/tmp/ros_log.txt'
rclpy.init()
node = rclpy.create_node('ros_logger')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=10)
got = []


def on_cloud(m):
    got.append((time.time(),
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                len(m.points)))


node.create_subscription(CustomMsg, '/livox/lidar', on_cloud, q)
t0 = time.time()
while not got and time.time() - t0 < 40:
    rclpy.spin_once(node, timeout_sec=0.3)
if not got:
    print('40s 没收到云', flush=True)
    raise SystemExit(1)
print('DDS 发现耗时 %.1fs' % (time.time() - t0), flush=True)
got.clear()
with open(OUT, 'w') as f:
    te = time.time() + DUR
    while time.time() < te:
        rclpy.spin_once(node, timeout_sec=0.05)
        if got:
            for w, st, n in got:
                f.write('%.6f %.6f %d\n' % (w, st, n))
            got.clear()
print('ros_logger 完成', flush=True)