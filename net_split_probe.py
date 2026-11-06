#!/usr/bin/env python3
"""net_split_probe.py — eno1 掉线后查数据链现状: 各关键话题速率+戳龄 + 毒包计数
(戳龄高≈WiFi路径重发现, 全静默≈狗/Orin失联)"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def age(stamp):
    return time.time() - stamp.sec - stamp.nanosec * 1e-9


class P(Node):
    def __init__(self):
        super().__init__("net_split_probe")
        self.c = {t: 0 for t in ("/Odometry", "/livox/points2", "/go2/odom")}
        self.a = {}
        self.h = "--"
        self.create_subscription(Odometry, "/Odometry", self.cb_o, 20)
        self.create_subscription(PointCloud2, "/livox/points2", self.cb_p, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/go2/odom", lambda m: self.cb("/go2/odom", m), qos_profile_sensor_data)
        self.create_subscription(String, "/loc/health", lambda m: setattr(self, "h", m.data), 10)

    def cb(self, t, m):
        self.c[t] += 1
        if m.header.stamp.sec:
            self.a[t] = age(m.header.stamp)

    def cb_o(self, m):
        self.cb("/Odometry", m)

    def cb_p(self, m):
        self.cb("/livox/points2", m)


def main():
    rclpy.init()
    n = P()
    t0 = time.time()
    while time.time() - t0 < 12:
        rclpy.spin_once(n, timeout_sec=0.3)
    dt = time.time() - t0
    print(f"[probe] /loc/health = {n.h}")
    for t, c in n.c.items():
        a = f" 戳龄={n.a[t]*1000:.0f}ms" if t in n.a else ""
        print(f"[probe] {t:<16} {c/dt:5.1f} Hz{a}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
