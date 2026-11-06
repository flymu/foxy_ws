#!/usr/bin/env python3
"""wire_logger.py — root 抓 eth0 点包，写 (wall, udp_cnt, dot_num, lidar_ts) 到日志
用法: python3 wire_logger.py [时长s，默认25]"""
import socket
import struct
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
OUT = '/tmp/wire_log.txt'
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(3))
s.bind(('eth0', 0))
s.settimeout(0.5)
n = 0
with open(OUT, 'w') as f:
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
        f.write('%.6f %d %d %d\n' % (time.time(),
                                     struct.unpack_from('<H', p, 7)[0],
                                     struct.unpack_from('<H', p, 5)[0],
                                     struct.unpack_from('<Q', p, 28)[0]))
        n += 1
        if n % 2000 == 0:
            f.flush()
print('wire_logger 完成: %d 包' % n, flush=True)
