#!/usr/bin/env python3
"""
dog_odom_sim.py — 狗航位推算看门狗的离线验证器（foxy 容器内跑，配合 bag 回放）
订阅回放的 /Odometry（Point-LIO），按场景把 LIO 轨迹改装成 /go2/odom 喂给定位节点，
时间戳沿用 /Odometry（与 LIO 同钟域，下游按各自流内部差分对窗口）。

场景（argv[1]，drift 的切换秒数 argv[2] 默认 30）：
  mirror     狗=LIO                 → 看门狗应始终 GOOD，与旧版行为一致（回归）
  drift  [N] 狗=LIO+0.5m/s 漂移，N 秒后偏移冻结 → 应 2-4s 切死推、N 秒后解除
  glitch     mirror，第 15s 瞬移 +5m → 应识别"狗里程计复位"，不误切死推
  latchtele  漂移 10s（先切死推）→ 瞬移 +5m（死推中复位）→ 偏移冻结（解除）
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

SCEN = sys.argv[1] if len(sys.argv) > 1 else "mirror"
SWITCH = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
DRIFT_RATE = 0.5  # m/s

rclpy.init()
node = Node("dog_odom_sim")
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
pub = node.create_publisher(Odometry, "/go2/odom", qos)
t0 = None
n = 0


def dog_offset(el):
    """狗相对 LIO 的 x 向偏移（含瞬移阶跃）"""
    if SCEN == "mirror":
        return 0.0
    if SCEN == "glitch":
        return 5.0 if el >= 15.0 else 0.0
    if SCEN == "latchtele":
        # 0-10s 漂移(0.5m/s)，10s 时再瞬移 +5m，之后偏移冻结
        return DRIFT_RATE * 10.0 + 5.0 if el >= 10.0 else DRIFT_RATE * el
    if SCEN == "drift":
        return DRIFT_RATE * el if el < SWITCH else DRIFT_RATE * SWITCH
    raise SystemExit("未知场景: " + SCEN)


def on_odom(m):
    global t0, n
    if t0 is None:
        t0 = time.time()
        node.get_logger().info("场景 %s 开始（switch=%ss）" % (SCEN, SWITCH))
    el = time.time() - t0
    d = Odometry()
    d.header.stamp = m.header.stamp
    d.header.frame_id = "dog_odom"
    d.child_frame_id = "go2_base"
    d.pose = m.pose
    d.twist = m.twist
    d.pose.pose.position.x += dog_offset(el)
    pub.publish(d)
    n += 1
    if n % 300 == 0:
        node.get_logger().info("%.0fs 已仿 %d 帧" % (el, n))


node.create_subscription(Odometry, "/Odometry", on_odom, qos)
node.get_logger().info("dog_odom_sim 就绪: %s" % SCEN)
rclpy.spin(node)
