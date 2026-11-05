#!/usr/bin/env python3
"""probe_observer.py — 流量见证者：独立进程旁观某话题，每10s打印累计计数
用于定位 50Hz 流断流的断点在写者侧还是读者侧（两个见证者对照）。
用法: python3 probe_observer.py <话题> [时长秒]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

topic = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 130.0

rclpy.init()
node = Node("obs_" + topic.strip("/").replace("/", "_"))
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=10)
n = [0]
node.create_subscription(
    __import__("geometry_msgs.msg", fromlist=["Twist"]).Twist, topic,
    lambda m: n.__setitem__(0, n[0] + 1), q)

import threading
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
t0 = time.monotonic()
last = 0
while time.monotonic() - t0 < dur:
    time.sleep(10.0)
    print("%5.0fs 累计 %d  近10s %d" % (
        time.monotonic() - t0, n[0], n[0] - last), flush=True)
    last = n[0]
node.destroy_node()
rclpy.shutdown()
