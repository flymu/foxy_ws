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
print(f"当前位姿 ({pose['x']:.2f},{pose['y']:.2f}) yaw={math.degrees(pose['yaw']):.1f}°")
# 等 DDS 匹配上订阅者再发——刚重启的规划器/毒包风暴时代 SEDP 可能 >2s，
# 盲发 2s 窗口会在匹配完成前打完，消息一条都到不了（11-06 返程腿实测踩过）
t0 = time.time()
matched_at = None
sent = 0
last_pub = 0.0
last_log = 0.0
while time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.2)
    el = time.time() - t0
    cnt = pub.get_subscription_count()
    if cnt > 0 and matched_at is None:
        matched_at = el
        print(f"t={el:.1f}s 匹配上 {cnt} 个订阅者！", flush=True)
    if cnt > 0 and el - last_pub >= 0.5:
        pub.publish(g)  # 匹配后 0.5s 一次重发，防首帧滞留
        sent += 1
        last_pub = el
    if cnt == 0 and el - last_log >= 5:
        print(f"t={el:.0f}s 订阅者匹配数=0 ...", flush=True)
        last_log = el
if matched_at is None:
    print("30s 未匹配上 /goal_pose 订阅者（规划器发现层可能被毒包搞死）", flush=True)
elif sent == 0:
    pub.publish(g)
print(f"已发目标 ({gx:.2f},{gy:.2f})×{sent}" + (f"（指定点）" if AT else f"（前方{DIST}m）"), flush=True)
rclpy.shutdown()