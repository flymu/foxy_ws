#!/usr/bin/env python3
"""goal_loop.py — 让狗绕屋子自主走一圈（foxy-live 容器内跑）

依次向 /goal_pose 发航点（1Hz 持续重发，防发现竞态/丢目标），
订阅 /loc/pose 判到达：距目标 <0.6m 且稳定 3s，或走过 0.5m 后稳定 5s，
或单段超时 150s。SIGINT 时向 /go2/cmd 发 stop 硬停狗。
用法:
  source /opt/ros/foxy/setup.bash && source ~/foxy_ws/install_x86/setup.bash
  timeout -s INT 900 python3 ~/foxy_ws/goal_loop.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# 俯视图主可行带 y∈[-2,1.5] x∈[-4,5] 内的巡圈航点（map 系）
WAYPOINTS = [
    ("东侧", 1.5, 0.8),
    ("西北", -2.5, 0.7),
    ("西南", -3.0, -1.0),
    ("南侧", 0.5, -1.3),
    ("回起点", -1.3, -0.4),
]
GOAL_RADIUS = 0.6    # m，距目标视为到达
REACH_STABLE_S = 3.0
MOVED_MIN = 0.5      # m，本段至少走过这么多才允许"中途停下"判定
STOP_STABLE_S = 5.0
STABLE_MOVE = 0.15   # m，窗口内位移小于此=稳定
LEG_TIMEOUT = 150.0  # s
PUB_PERIOD = 1.0


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class GoalLoop(Node):
    def __init__(self):
        super().__init__("goal_loop")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pose = None
        self.create_subscription(Odometry, "/loc/pose", self.on_pose, q)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", q)
        self.stop_pub = self.create_publisher(String, "/go2/cmd", q)

    def on_pose(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y)

    def publish_goal(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        self.goal_pub.publish(m)

    def hard_stop(self):
        m = String()
        m.data = "stop"
        self.stop_pub.publish(m)
        time.sleep(0.3)
        self.stop_pub.publish(m)


def run_leg(node, name, gx, gy, yaw):
    t0 = time.time()
    last_pub = 0.0
    start = node.pose or (0.0, 0.0)
    moved = 0.0
    last = start
    stable_t0 = None
    while rclpy.ok() and time.time() - t0 < LEG_TIMEOUT:
        now = time.time()
        if now - last_pub >= PUB_PERIOD:
            node.publish_goal(gx, gy, yaw)
            last_pub = now
        rclpy.spin_once(node, timeout_sec=0.3)
        p = node.pose
        if p is None:
            continue
        moved = max(moved, dist(p, start))
        if dist(p, last) > STABLE_MOVE:
            stable_t0 = now
        elif stable_t0 is None:
            stable_t0 = now
        stable_for = now - stable_t0
        last = p
        if dist(p, (gx, gy)) < GOAL_RADIUS and stable_for >= REACH_STABLE_S:
            return "到达 (耗时%.0fs)" % (now - t0)
        if moved >= MOVED_MIN and stable_for >= STOP_STABLE_S:
            return "中途停稳 (距目标%.2fm, 耗时%.0fs)" % (dist(p, (gx, gy)), now - t0)
    p = node.pose or start
    return "超时 (距目标%.2fm)" % dist(p, (gx, gy))


def main():
    rclpy.init()
    node = GoalLoop()
    print("等待 /loc/pose ...", flush=True)
    t0 = time.time()
    while node.pose is None and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.pose is None:
        print("20s 未收到 /loc/pose，退出", flush=True)
        return
    print("起点 (%.2f, %.2f)，共 %d 个航点" % (node.pose[0], node.pose[1], len(WAYPOINTS)),
          flush=True)
    results = []
    try:
        for i, (name, gx, gy) in enumerate(WAYPOINTS):
            nxt = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
            yaw = math.atan2(nxt[2] - gy, nxt[1] - gx)
            print("[航点%d %s] 目标 (%.2f, %.2f) yaw=%.0f°" % (i + 1, name, gx, gy, math.degrees(yaw)),
                  flush=True)
            r = run_leg(node, name, gx, gy, yaw)
            results.append((name, r))
            print("  → %s  当前位姿 (%.2f, %.2f)" % (r, node.pose[0], node.pose[1]), flush=True)
            time.sleep(1.5)
    except KeyboardInterrupt:
        print("收到中断，硬停狗", flush=True)
    finally:
        try:
            node.hard_stop()
        except Exception:
            pass
        print("----巡圈汇总----", flush=True)
        for name, r in results:
            print("%s: %s" % (name, r), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
