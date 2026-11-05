#!/usr/bin/env python3
"""doctor.py — 巡检链路只读体检（foxy-live 容器内跑，全链路各级速率/健康一眼看全）
零侵入：只订阅不发布。采样 5s，输出每级判定，异常退出码 1。
覆盖 2026-11-05 D1+D2 架构：WS 侧 LIO/定位/规划/适配，WiFi 只过 point_lio 小报文
与 Orin 侧 veto/桥/狗航位推算状态（大流量点云不出狗）。
用法（宿主机）:
  sg docker -c 'docker exec foxy-live python3 /home/mu/foxy_ws/doctor.py'
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

SAMPLE_S = 5.0

# 话题 -> (订阅类型, 是否关键, 期望Hz下限, 说明)
SPEC = [
    ("/Odometry",       "odom", True,  8.0,  "LIO 里程计(WS)"),
    ("/loc/pose",       "odom", True,  8.0,  "定位输出(WS)"),
    ("/loc/health",     "str",  True,  0.5,  "定位健康(WS, 1Hz心跳)"),
    ("/cmd_vel_intent", "tw",   True, 20.0,  "规划意图(WS, 100Hz; 门控时零速也保持流)"),
    ("/cmd_vel_raw",    "tw",   True, 20.0,  "适配输出(WS→WiFi)"),
    ("/go2/cmd_vel",    "tw",   True, 10.0,  "veto 放行(Orin→狗)"),
    ("/veto/status",    "str",  False, 0.0,  "否决层状态(Orin)"),
    ("/go2/status",     "str",  False, 0.0,  "桥心跳(Orin, 1Hz)"),
    ("/go2/odom",       "odom", False, 5.0,  "狗航位推算(Orin)"),
    ("/plan",           "path", False, 0.0,  "规划轨迹(可无目标)"),
]


class Doctor(Node):
    def __init__(self):
        super().__init__("doctor")
        self.counts = {t: 0 for t, *_ in SPEC}
        self.last = {t: None for t, *_ in SPEC}
        self.nonzero = {t: 0 for t, k, _, _, _ in SPEC if k == "tw"}
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
        for topic, kind, _, _, _ in SPEC:
            if kind == "odom":
                self.create_subscription(Odometry, topic,
                                         lambda m, t=topic: self.on_odom(t, m), be)
            elif kind == "tw":
                self.create_subscription(Twist, topic,
                                         lambda m, t=topic: self.on_tw(t, m), 10)
            elif kind == "str":
                self.create_subscription(String, topic,
                                         lambda m, t=topic: self.on_str(t, m), 10)
            else:
                self.create_subscription(Path, topic,
                                         lambda m, t=topic: self.cb(t, m), 10)

    def cb(self, topic, _m):
        self.counts[topic] += 1

    def on_odom(self, topic, m):
        p = m.pose.pose.position
        self.cb(topic, m)
        self.last[topic] = f"({p.x:+.2f},{p.y:+.2f})"

    def on_tw(self, topic, m):
        self.cb(topic, m)
        nz = abs(m.linear.x) > 1e-6 or abs(m.linear.y) > 1e-6 or abs(m.angular.z) > 1e-6
        self.nonzero[topic] += nz
        if nz:
            self.last[topic] = f"vx={m.linear.x:+.2f} vyaw={m.angular.z:+.2f}"

    def on_str(self, topic, m):
        self.cb(topic, m)
        self.last[topic] = m.data


def main():
    rclpy.init()
    n = Doctor()
    t_end = time.time() + SAMPLE_S
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.3)
    n.destroy_node()
    rclpy.shutdown()

    print(f"==== doctor 体检（采样 {SAMPLE_S:.0f}s）====")
    print(f"{'话题':<18}{'实测Hz':>8}{'非零%':>6}  最新值/判定")
    bad = 0
    for topic, kind, critical, hz_min, desc in SPEC:
        hz = n.counts[topic] / SAMPLE_S
        note = n.last[topic] or "--"
        mark = "✓"
        if hz_min > 0 and hz < hz_min:
            mark = "✗ 无流/欠速"
            if critical:
                bad += 1
        elif hz_min > 0:
            mark = "✓"
        else:
            mark = "✓" if n.counts[topic] else "–(可无)"
        nz = ""
        if kind == "tw" and n.counts[topic]:
            nz = f"{100 * n.nonzero[topic] // n.counts[topic]:>4}%"
        print(f"{topic:<18}{hz:>7.1f} {nz:>5}  {mark}  {note}  [{desc}]")

    # 定位健康语义检查（R5 三态格式 STATE|reason|age）
    h = n.last.get("/loc/health") or ""
    health_ok = False
    if h:
        parts = h.split("|")
        if len(parts) == 3 and parts[0] == "OK":
            health_ok = True
    print(f"定位健康语义: {h or '无'} → {'✓ OK' if health_ok else '✗ 非 OK（门控将关闭，规划器零速）'}")
    if not health_ok:
        bad += 1
    print(f"==== 结论: {'链路健康' if bad == 0 else f'{bad} 项关键异常'} ====")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
