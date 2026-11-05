#!/usr/bin/env python3
"""r5_seed_pose.py — /initialpose 重种子注入器（容器内跑，rclpy 直发不走 CLI）
读采集文件里最新一条 loc 行，按 +(3,1)/yaw+90° 构造 PoseWithCovarianceStamped
发到 /initialpose（RViz "2D Pose Estimate" 等价路径，BBS 种子兼容格式）。
等 loc 节点订阅匹配上才发（最多 5s），发完 1s 收尾退出。
用法: python3 r5_seed_pose.py <capture_file>
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main():
    last = None
    with open(sys.argv[1]) as f:
        for line in f:
            p = line.split(" ", 2)
            if len(p) >= 3 and p[1] == "loc":
                last = p[2].split()
    if last is None:
        raise SystemExit("无 loc 行")
    x, y, z, qz, qw = map(float, last[:5])
    yaw = 2.0 * math.atan2(qz, qw) + math.pi / 2.0
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x + 3.0
    msg.pose.pose.position.y = y + 1.0
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    print(f"[r5_seed_pose] seed T_mb=({x + 3.0:.3f},{y + 1.0:.3f}) yaw={math.degrees(yaw) % 360:.1f}°",
          flush=True)

    rclpy.init()
    node = Node("r5_seed_pose")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    import time
    t0 = time.time()
    while pub.get_subscription_count() < 1 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() < 1:
        print("[r5_seed_pose] !! 无订阅者(loc 未起?)", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    pub.publish(msg)
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    print("[r5_seed_pose] 已发", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
