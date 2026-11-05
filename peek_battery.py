#!/usr/bin/env python3
"""peek_battery.py — 读狗电池（只读，跑在 Orin，cyclone/eth1 环境）
/lf/lowstate 的 bms_state.soc = 电量%，power_v/power_a = 电池电压/电流。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowState

qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5)

rclpy.init()
n = Node("battery_peek")
got = []


def on(m):
    got.append((m.bms_state.soc, m.power_v, m.power_a))


n.create_subscription(LowState, '/lf/lowstate', on, qb)

end = time.monotonic() + 6.0
while time.monotonic() < end and rclpy.ok() and len(got) < 20:
    rclpy.spin_once(n, timeout_sec=0.5)
n.destroy_node()
rclpy.shutdown()

if got:
    soc = got[-1][0]
    print('battery: soc=%s%% power_v=%.2fV power_a=%.2fA (%d samples)' % (
        soc, got[-1][1], got[-1][2], len(got)), flush=True)
else:
    print('battery: /lf/lowstate 6s 内没有消息', flush=True)