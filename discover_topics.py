#!/usr/bin/env python3
"""discover_topics.py — 主板 DDS 总线侦察（跑在 Orin，cyclone/eth1 环境）
列出总线上的全部话题与类型，重点找 rt/api/*（RPC 服务注册面），
找出固件到底暴露了哪些可调服务（为绕开 7004 权限门做情报）。
"""
import time

import rclpy

rclpy.init()
n = rclpy.create_node("bus_scout")
time.sleep(6.0)  # SDSP 发现要几秒
names = sorted(n.get_topic_names_and_types())
print("=== %d topics ===" % len(names), flush=True)
for name, types in names:
    print("%-58s %s" % (name, ",".join(types)), flush=True)
n.destroy_node()
rclpy.shutdown()
