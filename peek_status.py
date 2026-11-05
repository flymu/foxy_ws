#!/usr/bin/env python3
"""peek_status.py — 一次性旁观任意 std_msgs/String 话题 6s，打印首条消息。
用法: python3 peek_status.py <话题>   （RELIABLE 订阅）
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

topic = sys.argv[1]
rclpy.init()
n = Node("peek_" + topic.strip("/").replace("/", "_"))
got = []
n.create_subscription(String, topic, lambda m: got.append(m.data), 5)
t0 = time.monotonic()
while not got and time.monotonic() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.3)
print(got[0] if got else "NO MSG in 6s")
