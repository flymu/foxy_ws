#!/usr/bin/env python3
"""initialpose_pub.py — 手动重种子: 发 /initialpose (map系) 指定位姿
用法: python3 initialpose_pub.py x y yaw_deg [z]
(RViz "2D Pose Estimate" 同款话题; loc 收到后 [reseed] 重建 T_mb/T_mo, NDT 精修)"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

x, y, yawd = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
yaw = math.radians(yawd)

rclpy.init()
n = Node("initialpose_pub")
pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
g = PoseWithCovarianceStamped()
g.header.frame_id = "map"
g.header.stamp = n.get_clock().now().to_msg()
g.pose.pose.position.x = x
g.pose.pose.position.y = y
g.pose.pose.position.z = z
g.pose.pose.orientation.w = math.cos(yaw / 2)
g.pose.pose.orientation.z = math.sin(yaw / 2)
for _ in range(5):  # 重复发 2s 防丢
    pub.publish(g)
    time.sleep(0.4)
print(f"已发 /initialpose ({x:.3f},{y:.3f},{z:.3f}) yaw={yawd:.1f}° ×5")
rclpy.shutdown()
