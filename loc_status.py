#!/usr/bin/env python3
"""loc_status.py — 一次性读取 /loc/pose 与 /loc/health 现状（foxy-live 容器内跑）

容器内调试订阅一律 RELIABLE（Foxy 同容器 best_effort 订阅收不到 reliable 发布）。
用法:
  source /opt/ros/foxy/setup.bash && source ~/foxy_ws/install_x86/setup.bash
  timeout -s INT 25 python3 ~/foxy_ws/loc_status.py
"""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WAIT_S = 20.0  # 跨 WiFi 发现要 5-15s，给足


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                    1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    rclpy.init()
    node = Node("loc_status")
    qr = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=5)
    state = {"pose": None, "yaw": None, "health": []}

    def on_pose(m):
        p = m.pose.pose.position
        state["pose"] = (p.x, p.y, p.z)
        state["yaw"] = yaw_of(m.pose.pose.orientation)

    def on_health(m):
        state["health"].append(m.data)

    node.create_subscription(Odometry, "/loc/pose", on_pose, qr)
    node.create_subscription(String, "/loc/health", on_health, qr)

    t0 = time.time()
    while time.time() - t0 < WAIT_S:
        rclpy.spin_once(node, timeout_sec=0.5)

    print("----STATUS----")
    if state["pose"] is None:
        print("/loc/pose: 20s 内未收到")
    else:
        x, y, z = state["pose"]
        print("/loc/pose: x=%.3f y=%.3f z=%.3f yaw=%.1fdeg" % (x, y, z, state["yaw"]))
    if state["health"]:
        print("/loc/health: %d拍 last=%s" % (len(state["health"]), state["health"][-1]))
    else:
        print("/loc/health: 20s 内未收到（每 ~12.7s 一拍，正常）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
