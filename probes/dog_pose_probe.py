#!/usr/bin/env python3
"""dog_pose_probe.py — 读几帧 /go2/odom 的位姿（狗航位推算，狗唤醒后才有流）"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("dog_pose_probe")
got = []


def cb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    got.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))


n.create_subscription(Odometry, "/go2/odom", cb, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 6 and len(got) < 5:
    rclpy.spin_once(n, timeout_sec=0.3)
if got:
    x, y, z, qx, qy, qz, qw = got[-1]
    print(f"dog pose: x={x:.6f} y={y:.6f} z={z:.6f} quat=({qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}) [{len(got)}帧]")
else:
    print("无 /go2/odom 流（狗未唤醒？）")