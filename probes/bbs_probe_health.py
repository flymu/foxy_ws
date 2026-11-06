#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("bbs_probe_health")
got = {}

n.create_subscription(String, "/loc/health", lambda m: got.update(health=m.data), 10)
n.create_subscription(Odometry, "/loc/pose", lambda m: got.update(
    pose=(round(m.pose.pose.position.x,3), round(m.pose.pose.position.y,3),
          round(m.pose.pose.position.z,3))), 10)
t0 = time.time()
while time.time()-t0 < 8 and len(got) < 2:
    rclpy.spin_once(n, timeout_sec=0.4)
print(got if got else "TIMEOUT: /loc/* 全静默")
