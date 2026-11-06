#!/usr/bin/env python3
"""bbs_initialpose_pub.py — 把 bbs_pose 输出的 T_map_odom (4x4) 以 T_mb 语义注入 /initialpose。
定位节点按 R5 设计取 x/y/yaw, z/roll/pitch 沿用当前, T_mo=T_mb·T_ob⁻¹, NDT 精修接管。
在 foxy-live 容器内跑: python3 bbs_initialpose_pub.py <matrix_file> [--show]
matrix_file = 4 行浮点 (行优先 3x4 或 4x4)。"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    T = np.loadtxt(sys.argv[1])
    if T.shape == (3, 4):
        T = np.vstack([T, [0, 0, 0, 1]])
    assert T.shape == (4, 4)
    x, y = T[0, 3], T[1, 3]
    yaw = np.arctan2(T[1, 0], T[0, 0])
    print(f"T_map_odom: t=({x:.3f},{y:.3f},{T[2,3]:.3f}) yaw={np.degrees(yaw):.2f}°")
    if "--show" in sys.argv:
        print(np.array2string(T, precision=3, suppress_small=True))
        return
    rclpy.init()
    node = Node("bbs_initialpose_pub")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(T[2, 3])
    qz, qc = np.sin(yaw / 2.0), np.cos(yaw / 2.0)
    msg.pose.pose.orientation.z = float(qz)
    msg.pose.pose.orientation.w = float(qc)
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    # 等 publisher 发现
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() > 0:
            break
    pub.publish(msg)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("[published] /initialpose 已注入, 等 NDT 精修 (几秒内 /loc/health 应回 OK)")


if __name__ == "__main__":
    main()
