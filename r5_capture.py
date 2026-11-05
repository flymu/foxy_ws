#!/usr/bin/env python3
"""r5_capture.py — R5 回放回归数据采集（foxy-live 容器内跑）
订阅 /loc/pose /Odometry /go2/odom /cmd_vel_intent /loc/health，
每条一行追加写 argv[1]（行缓冲，host 可直接读共享卷）:
  <wall_ms> loc  x y z qz qw
  <wall_ms> lio  x y z qz qw
  <wall_ms> dog  x y z qz qw
  <wall_ms> cmd  vx vy vyaw
  <wall_ms> health <原串>
用法: python3 r5_capture.py /home/mu/foxy_ws/logs_r5/rg1_cap.txt
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Cap(Node):
    def __init__(self, out):
        super().__init__("r5_capture")
        self.f = open(out, "a", 1)  # 行缓冲
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/loc/pose",
                                 lambda m: self.odom("loc", m), 10)
        self.create_subscription(Odometry, "/Odometry",
                                 lambda m: self.odom("lio", m), be)
        self.create_subscription(Odometry, "/go2/odom",
                                 lambda m: self.odom("dog", m), be)
        self.create_subscription(Twist, "/cmd_vel_intent",
                                 lambda m: self.line("cmd", f"{m.linear.x:.4f} "
                                                       f"{m.linear.y:.4f} {m.angular.z:.4f}"), 10)
        self.create_subscription(String, "/loc/health",
                                 lambda m: self.line("health", m.data), 10)

    def line(self, tag, payload):
        self.f.write(f"{time.time() * 1000:.0f} {tag} {payload}\n")

    def odom(self, tag, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.line(tag, f"{p.x:.4f} {p.y:.4f} {p.z:.4f} {q.z:.4f} {q.w:.4f}")


def main():
    if len(sys.argv) < 2:
        raise SystemExit("用法: r5_capture.py <输出文件>")
    rclpy.init()
    n = Cap(sys.argv[1])
    print(f"[r5_capture] -> {sys.argv[1]}", flush=True)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
