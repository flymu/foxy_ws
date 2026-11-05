#!/usr/bin/env python3
"""dual_probe.py — 同进程双通道：eth0 原始抓包 + /livox/lidar 订阅。
钉死三个问题：
  1. 网线上的包（udp_cnt 连续性、点率）——雷达侧有没有丢/重发
  2. ROS 消息（率、点数分布、戳滞后）——主机侧丢了多少
  3. 戳推进速率 vs 墙钟——10s 是恒定 FIFO 深度还是饥饿累积
用法（Orin，sudo）: python3 dual_probe.py
"""
import socket
import struct
import threading
import time

DUR = 15.0
wire = []   # (wall, udp_cnt, dot_num)
msgs = []   # (wall, stamp_s, npts)

# ---------- 通道1: ROS 订阅（先启动，等 DDS 发现） ----------
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node('dual_probe')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=10)


def on_cloud(m):
    msgs.append((time.time(),
                 m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                 len(m.points)))


node.create_subscription(CustomMsg, '/livox/lidar', on_cloud, q)
print('等 DDS 发现...', flush=True)
t0 = time.time()
while not msgs and time.time() - t0 < 40:
    rclpy.spin_once(node, timeout_sec=0.3)
if not msgs:
    print('40s 没收到云，放弃')
    raise SystemExit(1)
msgs.clear()
print('DDS 发现完成，开始 %.0fs 同步采样...' % DUR, flush=True)

# ---------- 通道2: 原始抓包线程 ----------


def sniff():
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(3))
    s.bind(('eth0', 0))
    s.settimeout(0.5)
    te = time.time() + DUR
    while time.time() < te:
        try:
            pkt, _ = s.recvfrom(2048)
        except socket.timeout:
            continue
        if len(pkt) < 42 or struct.unpack_from('!H', pkt, 12)[0] != 0x0800:
            continue
        ihl = (pkt[14] & 0x0F) * 4
        if pkt[14 + 9] != 17:
            continue
        udp = 14 + ihl
        if struct.unpack_from('!H', pkt, udp + 2)[0] != 56301:
            continue
        p = pkt[udp + 8:]
        if len(p) < 36:
            continue
        wire.append((time.time(), struct.unpack_from('<H', p, 7)[0],
                     struct.unpack_from('<H', p, 5)[0]))


th = threading.Thread(target=sniff)
th.start()
te = time.time() + DUR
while time.time() < te:
    rclpy.spin_once(node, timeout_sec=0.05)
th.join()
node.destroy_node()

# ---------- 分析 ----------
w = sorted(wire)
m = sorted(msgs)
print('\n=== 网线通道 (eth0 原始包) ===')
if w:
    span = w[-1][0] - w[0][0]
    dots = sum(d for _, _, d in w)
    cnts = [c for _, c, _ in w]
    dup = len(cnts) - len(set(cnts))
    gaps = [b - a for a, b in zip(cnts, cnts[1:]) if b - a != 1]
    print('包数=%d  时长=%.1fs  包率=%.0f/s' % (len(w), span, len(w) / span))
    print('点率=%.0f 点/s (dot_num 合计 %d)' % (dots / span, dots))
    print('udp_cnt: 首=%d 末=%d 应到=%d 实到=%d → 线上缺失%d'
          % (cnts[0], cnts[-1], cnts[-1] - cnts[0] + 1, len(cnts),
             cnts[-1] - cnts[0] + 1 - len(cnts)))
    print('重复cnt=%d  跳变(≠+1)次数=%d' % (dup, len(gaps)))
else:
    print('没抓到包!')

print('\n=== ROS 通道 (/livox/lidar) ===')
if m:
    span = m[-1][0] - m[0][0]
    npts = [n for _, _, n in m]
    lags = sorted(wl - st for wl, st, _ in m)
    stamp_rate = (m[-1][1] - m[0][1]) / (m[-1][0] - m[0][0])
    npts_sorted = sorted(npts)

    def pc(a, q_):
        return a[min(len(a) - 1, int(q_ / 100 * len(a)))]

    print('消息数=%d  时长=%.1fs  消息率=%.2f Hz' % (len(m), span, len(m) / span))
    print('点数/消息: p10=%d p50=%d p90=%d  → 点率=%.0f/s'
          % (pc(npts_sorted, 10), pc(npts_sorted, 50), pc(npts_sorted, 90),
             sum(npts) / span))
    print('戳滞后(墙钟-戳): p10=%.2f p50=%.2f p90=%.2f s'
          % (pc(lags, 10), pc(lags, 50), pc(lags, 90)))
    print('戳推进速率 = %.4f× 墙钟（1.0=恒定FIFO延迟, <1=饥饿累积）' % stamp_rate)
    if w:
        wspan = w[-1][0] - w[0][0]
        wrate = sum(d for _, _, d in w) / wspan
        trate = sum(npts) / span
        print('\n=== 对照 ===')
        print('线上点率 %.0f/s vs 话题点率 %.0f/s  → 话题保真率 %.1f%%'
              % (wrate, trate, trate / wrate * 100))
else:
    print('采样窗口内没收到消息!')