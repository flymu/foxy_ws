#!/usr/bin/env python3
"""g4_goal_pub.py — G4 发目标: 按当前位姿+朝向+距离算目标点, 发 /goal_pose (map系)
用法: python3 g4_goal_pub.py [距离m, 默认2.0]  （可选 --at x y 直接指定目标点）"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

DIST = 2.0
AT = None
args = sys.argv[1:]
if "--at" in args:
    i = args.index("--at")
    AT = (float(args[i + 1]), float(args[i + 2]))
else:
    for a in args:
        try:
            DIST = float(a)
            break
        except ValueError:
            pass

rclpy.init()
n = Node("g4_goal_pub")
pose = {}


def cb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y))
    pose.update(x=p.x, y=p.y, yaw=yaw)


n.create_subscription(Odometry, "/loc/pose", cb, 10)
pub = n.create_publisher(PoseStamped, "/goal_pose", 10)
t0 = time.time()
while time.time() - t0 < 5 and "x" not in pose:
    rclpy.spin_once(n, timeout_sec=0.2)
if "x" not in pose:
    print("无 /loc/pose！不发。")
    sys.exit(1)
if AT:
    gx, gy = AT
else:
    gx = pose["x"] + DIST * math.cos(pose["yaw"])
    gy = pose["y"] + DIST * math.sin(pose["yaw"])
g = PoseStamped()
g.header.frame_id = "map"
g.header.stamp = n.get_clock().now().to_msg()
g.pose.position.x = gx
g.pose.position.y = gy
g.pose.orientation.w = 1.0
pub.publish(g)
print(f"当前位姿 ({pose['x']:.2f},{pose['y']:.2f}) yaw={math.degrees(pose['yaw']):.1f}°")
print(f"已发目标 ({gx:.2f},{gy:.2f})" + (f"（指定点）" if AT else f"（前方{DIST}m）"))
time.sleep(1.5)  # 等 DDS 送出
rclpy.shutdown()