#!/usr/bin/env python3
"""live_probe.py — 巡逻栈数据通量探针（foxy-live 容器内跑）

订阅 /livox/lidar(雷达CustomMsg) 与 /go2/odom(狗里程计)，先空转 5s 让
WiFi 上的 DDS 发现任检完成，再计数 10s 打印速率。
用法:
  source /opt/ros/foxy/setup.bash && source ~/foxy_ws/install_x86/setup.bash
  timeout -s INT 25 python3 ~/foxy_ws/live_probe.py
"""
import time

import rclpy
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

WARMUP_S = 5.0   # 只发现不计数（跨 WiFi 发现要 5-15s）
COUNT_S = 10.0

TOPICS = {
    "/livox/lidar": (CustomMsg, "雷达CustomMsg"),
    "/go2/odom": (Odometry, "狗里程计"),
}


def main():
    rclpy.init()
    node = Node("live_probe")
    q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST, depth=10)
    cnt = {t: 0 for t in TOPICS}
    for t, (typ, _) in TOPICS.items():
        node.create_subscription(typ, t,
                                 lambda m, t=t: cnt.__setitem__(t, cnt[t] + 1), q)

    t0 = time.time()
    while time.time() - t0 < WARMUP_S:
        rclpy.spin_once(node, timeout_sec=0.5)
    cnt = {t: 0 for t in TOPICS}  # 预热期的计数清零，只统计正式窗口
    t1 = time.time()
    while time.time() - t1 < COUNT_S:
        rclpy.spin_once(node, timeout_sec=0.5)
    dt = time.time() - t1

    print("----PROBE----")
    for t, (_, label) in TOPICS.items():
        print("%-14s %-12s %4d条 / %.1fs = %.1fHz"
              % (t, label, cnt[t], dt, cnt[t] / dt))
    print("(狗里程计 0Hz 且雷达正常 = 狗趴着没唤醒，属正常)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
