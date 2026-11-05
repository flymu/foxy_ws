#!/usr/bin/env python3
"""sport_lease_probe.py — 租约版运动试探（跑在 Orin）

线索链（2026-11-03 撞权限门后的侦察结论）：
  1. 总线侦察发现 /api/sport_lease/request|response 服务；
  2. SDK 的 RequestHeader 带 lease 字段；LeaseClient 就是往 {svc}_lease
     发 LEASE_APPLY(101) 拿 {id,term}，后台线程按 term*0.3 续约(102)；
  3. SportClient(enableLease=False) 是默认——我们此前所有客户端 leaseId=0。
假设：固件对平移类(Move)要求有效租约，无租约静默丢弃；姿态类(BalanceStand)放行。
     （与实测吻合：BalanceStand 狗执行，Move 无反应且 noreply 不报错。）
验证：SportClient(enableLease=True) 申请租约 → BalanceStand → Move(0.10) 2.5s → StopMove。
租约被拒则不发 Move（狗不会动），把拒绝码打出来（3205=拒绝 3207=已被占）。
"""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, 'eth1')
sc = SportClient(enableLease=True)
sc.Init()

t0 = time.monotonic()
lease_id = None
while time.monotonic() - t0 < 8.0:
    lid = sc.GetLeaseId()
    if lid:
        lease_id = lid
        break
    time.sleep(0.2)
print('lease id = %s（等待 %.1fs）' % (lease_id, time.monotonic() - t0), flush=True)

code = sc.BalanceStand()
print('BalanceStand code=%s  <- 有应答RPC，狗应维持站立' % code, flush=True)
time.sleep(2.0)

if lease_id:
    code = sc.Move(0.10, 0.0, 0.0)
    print('Move(0.10,0,0) code=%s  <- noreply，狗若放行会向前挪 ~25cm' % code, flush=True)
    time.sleep(2.5)
    code = sc.StopMove()
    print('StopMove code=%s' % code, flush=True)
    time.sleep(1.0)
else:
    print('!!! 租约没拿到，不发 Move（狗不动）。看上面 LeaseClient 的报错码。', flush=True)