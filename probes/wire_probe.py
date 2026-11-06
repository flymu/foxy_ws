#!/usr/bin/env python3
"""wire_probe.py — 在 eth0 上直接嗅探雷达 UDP 原始包，解析包头内嵌时间戳。
回答一个问题：网线上现在到达的数据，其雷达时间戳是新鲜还是陈酿？
新鲜(<1s) → 延迟在主机侧 SDK/驱动流水线
陈酿(>5s) → 延迟在雷达内部（设备级问题）
用法（Orin 上，无需 ROS）: python3 wire_probe.py
"""
import socket
import struct
import time

DUR = 6.0
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003))
s.bind(('eth0', 0))
s.settimeout(1.0)

pts = []   # (wall, ts_ns, dot_num, time_type)
imus = []  # (wall, ts_ns)
ports = {}  # dport -> [count, first_payload_hex, first_wall, offsets_with_epoch_ts]
t0 = time.time()
while time.time() - t0 < DUR:
    try:
        pkt, _ = s.recvfrom(2048)
    except socket.timeout:
        continue
    if len(pkt) < 42 or struct.unpack_from('!H', pkt, 12)[0] != 0x0800:
        continue
    ihl = (pkt[14] & 0x0F) * 4
    if pkt[14 + 9] != 17:      # proto UDP
        continue
    udp = 14 + ihl
    dport = struct.unpack_from('!H', pkt, udp + 2)[0]
    payload = pkt[udp + 8:]
    ent = ports.setdefault(dport, [0, None, 0.0, None])
    ent[0] += 1
    if ent[1] is None:
        ent[1] = payload[:64].hex()
        ent[2] = time.time()
        offs = []
        for off in range(0, min(len(payload), 64) - 7):
            v = struct.unpack_from('<Q', payload, off)[0]
            if 1.7e18 < v < 1.9e18:
                offs.append((off, v))
        ent[3] = offs
    if dport == 56301 and len(payload) >= 36:      # 点云
        ts = struct.unpack_from('<Q', payload, 28)[0]
        if 1.7e18 < ts < 1.9e18:                   # epoch ns 合法域
            dot = struct.unpack_from('<H', payload, 5)[0]
            pts.append((time.time(), ts, dot, payload[11]))
    elif dport == 56401 and len(payload) >= 48:    # IMU
        ts = struct.unpack_from('<Q', payload, 40)[0]
        if 1.7e18 < ts < 1.9e18:
            imus.append((time.time(), ts))

def pct(sorted_lag, q):
    return sorted_lag[min(len(sorted_lag) - 1, int(q / 100 * len(sorted_lag)))]


def rep(name, arr):
    if not arr:
        print('%s: 0 包' % name)
        return
    lag = sorted(w - t / 1e9 for w, t in arr)
    print('%s: %d 包  滞后 p10=%.3f p50=%.3f p90=%.3f s'
          % (name, len(arr), pct(lag, 10), pct(lag, 50), pct(lag, 90)))


print('=== 抓包时长 %.1fs ===' % (time.time() - t0))
print('UDP端口直方图(目的端口):')
for dport, (cnt, hx, wall, offs) in sorted(ports.items(), key=lambda kv: -kv[1][0]):
    print('  dport=%d 包数=%d 首包载荷前64B=%s' % (dport, cnt, hx))
    if offs:
        print('    载荷内epoch-ns候选偏移: %s' % [(o, '%.3fs' % (wall - v / 1e9)) for o, v in offs])
rep('点云包(网线上)', [(w, t) for w, t, d, tt in pts])
if pts:
    tt = {t for _, _, _, t in pts}
    dots = sum(d for _, _, d, _ in pts)
    span = max(w for w, *_ in pts) - min(w for w, *_ in pts)
    print('  dot_num合计=%d → %.0f 点/s   time_type=%s   包间隔≈%.1fms'
          % (dots, dots / max(span, 1e-9), tt, span / max(len(pts) - 1, 1) * 1000))
rep('IMU包(网线上)', imus)