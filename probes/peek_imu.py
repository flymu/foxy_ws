#!/usr/bin/env python3
"""peek_imu.py — 旁听 /livox/imu：条数/频率/加速度均值/与 veto 先验法线的夹角与符号。
用法: python3 peek_imu.py [秒=6] [话题=/livox/imu]
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
topic = sys.argv[2] if len(sys.argv) > 2 else "/livox/imu"
PRIOR_N = np.array([-0.7330, 0.0001, 0.6802])
PRIOR_N = PRIOR_N / np.linalg.norm(PRIOR_N)

rclpy.init()
n = rclpy.create_node("peek_imu")
acc = []
qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=50)
n.create_subscription(Imu, topic,
                      lambda m: acc.append((m.linear_acceleration.x,
                                            m.linear_acceleration.y,
                                            m.linear_acceleration.z)), qb)
t0 = time.monotonic()
while time.monotonic() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
if not acc:
    print("NO IMU in %.0fs (topic=%s)" % (dur, topic))
    sys.exit(0)
A = np.asarray(acc)
mu = A.mean(axis=0)
mags = np.linalg.norm(A, axis=1)
up = mu / np.linalg.norm(mu)
dot = float(up @ PRIOR_N)
dev = math.degrees(math.acos(min(1.0, abs(dot))))
print("n=%d rate=%.0fHz |a| mean=%.3f std=%.3f" % (len(A), len(A) / dur,
                                                   mags.mean(), mags.std()))
print("mean_acc=[%.3f %.3f %.3f]" % tuple(mu))
print("与先验法线夹角=%.1f°  dot=%+.3f  -> 加速度计读数与'向上'%s"
      % (dev, dot, "同向(+g)" if dot > 0 else "反向(-g，需翻转符号)"))
