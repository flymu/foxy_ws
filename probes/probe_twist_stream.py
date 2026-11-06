#!/usr/bin/env python3
"""probe_twist_stream.py — 最小复现：rclpy↔rclpy 50Hz Twist 流 ~35s 静默断流

背景：veto 离线测试三轮都在 ~35s（≈1750 条）处 /cmd_vel_raw→/veto/cmd_out 链
静默断流（1Hz status 同期正常）。本探针去掉雷达/numpy/拦截逻辑，只留
"50Hz 定时器发 Twist + RELIABLE 订阅计数"，定位是 DDS/rclpy 层还是 veto 业务层。

用法（foxy-live 容器内，两个进程）:
  python3 probe_twist_stream.py writer 50      # 终端1
  python3 probe_twist_stream.py reader 50 60   # 终端2，60s 后打印时间线
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

role = sys.argv[1]
rate = int(sys.argv[2]) if len(sys.argv) > 2 else 50
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
SLOW = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
WSLOW = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

rclpy.init()
node = Node("probe_" + role)
qr = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=1)

if role == "writer":
    pub = node.create_publisher(Twist, "/probe/t", qr)
    n = [0]

    def tick():
        if time.monotonic() - t0 > dur:
            return
        time.sleep(WSLOW)
        m = Twist()
        m.linear.x = 0.25
        pub.publish(m)
        n[0] += 1

    t0 = time.monotonic()
    node.create_timer(1.0 / rate, tick)
    node.create_timer(5.0, lambda: print("writer 已发 %d" % n[0], flush=True))
    rclpy.spin(node)
else:
    got = []

    def on(m):
        time.sleep(SLOW)
        got.append(time.monotonic())

    node.create_subscription(Twist, "/probe/t", on, qr)
    node.create_timer(5.0, lambda: print(
        "reader 已收 %d (近5s: %d)" % (
            len(got),
            sum(1 for t in got if t > time.monotonic() - 5.0)), flush=True))
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    end = time.monotonic() + dur
    while time.monotonic() < end:
        time.sleep(0.5)
    # 断流检测：最后 10s 收到的比例
    tail = sum(1 for t in got if t > end - 10)
    print("==== 50Hz 流复现结果: 总收 %d, 期望 %d, 末10s %d (%.0f%%) ===="
          % (len(got), rate * dur, tail, 100.0 * tail / (10 * rate)))
node.destroy_node()
rclpy.shutdown()
