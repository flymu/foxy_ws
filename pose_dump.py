#!/usr/bin/env python3
"""pose_dump.py — 记录 /loc/pose 到 /tmp/pose_dump.csv（stamp,x,y），用于事后检查位姿连续性"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

rclpy.init()
node = Node("pose_dump")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)
f = open("/tmp/pose_dump.csv", "w", buffering=1)
n = [0]


def on_pose(m):
    t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    p = m.pose.pose.position
    f.write("%.3f %.4f %.4f\n" % (t, p.x, p.y))
    n[0] += 1
    if n[0] % 100 == 0:
        print("%d msgs" % n[0], flush=True)


node.create_subscription(Odometry, "/loc/pose", on_pose, qos)
print("pose_dump ready", flush=True)
rclpy.spin(node)
