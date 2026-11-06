#!/usr/bin/env python3
"""sport_stream_probe.py — 租约 + 连续流 Move 试探（跑在 Orin）

证据矩阵（狗是否平移）：
  无租约 + 单发   : 没反应（sport_probe）
  无租约 + 连续流 : 只打挺不平移（今天早些的桥/creep 全程 out=0.128 在流）
  有租约 + 单发   : 没反应（sport_lease_probe，租约秒批）
  有租约 + 连续流 : 本脚本 ← 缺的最后一格
Move 是 noreply 一次性速度指令，遥控器就是连续流驱动的；单发可能瞬间衰减。
租约 term=1s 由 LeaseClient 后台线程 0.3s 续约，全程有效。
狗若放行：BalanceStand 后以 0.10m/s 连发 3s → 向前 ~30cm → StopMove。
"""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, 'eth1')
sc = SportClient(enableLease=True)
sc.Init()

t0 = time.monotonic()
while not sc.GetLeaseId() and time.monotonic() - t0 < 8.0:
    time.sleep(0.2)
lid = sc.GetLeaseId()
print('lease id = %s（%.1fs）' % (lid, time.monotonic() - t0), flush=True)
if not lid:
    print('租约没拿到，收工', flush=True)
    raise SystemExit(1)

code = sc.BalanceStand()
print('BalanceStand code=%s' % code, flush=True)
time.sleep(2.0)

print('连发 Move(0.10) @20Hz × 3s ...', flush=True)
t1 = time.monotonic()
n = 0
while time.monotonic() - t1 < 3.0:
    sc.Move(0.10, 0.0, 0.0)
    n += 1
    time.sleep(0.05)
print('发了 %d 次 Move' % n, flush=True)

code = sc.StopMove()
print('StopMove code=%s' % code, flush=True)