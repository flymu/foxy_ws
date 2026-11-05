#!/usr/bin/env python3
"""hz_any.py — 任意话题 BE 订阅测率+戳滞后（rclpy, 兼容 REL/BE 发布端）
用法: hz_any.py <topic> <msg_module> <msg_type> [时长s=20]"""
import importlib
import sys
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

topic = sys.argv[1]
Msg = getattr(importlib.import_module(sys.argv[2]), sys.argv[3])
DUR = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0

rclpy.init()
node = rclpy.create_node('hz_any')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=10)
got = []


def cb(m):
    got.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))


node.create_subscription(Msg, topic, cb, q)
t0 = time.time()
while not got and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.3)
if not got:
    print('%s: 30s 无数据' % topic)
    raise SystemExit(1)
print('%s: 发现 %.1fs' % (topic, time.time() - t0), flush=True)
got.clear()
while time.time() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.05)
if len(got) < 2:
    print('%s: 窗口内 %d 条' % (topic, len(got)))
    raise SystemExit(1)
span = got[-1][0] - got[0][0]
lags = sorted(w - s for w, s in got)
n = len(lags)
print('%s: 消息数=%d 时长=%.1fs 率=%.2f Hz 戳滞后 p10=%.3f p50=%.3f p90=%.3f s'
      % (topic, len(got), span, (len(got) - 1) / span,
         lags[int(0.1 * n)], lags[n // 2], lags[int(0.9 * n)]))
