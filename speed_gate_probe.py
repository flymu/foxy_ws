#!/usr/bin/env python3
"""speed_gate_probe.py — 速度门决定性一测（跑在 Orin，狗前方留 2m 净空）

背景（2026-11-03 课程包审查结论）：课程讲义实锤 Go2 平台硬约束
「linear.x < 0.2 m/s 时原地不动」("极具迷惑性的假死")；而我们全部被吞
探针恰好都是 Move(0.10)。断电重启可能只是巧合——速度门才是真凶。

流程（每段之间停 2.5s 便于观察区分）：
  BalanceStand → 段1 vx=0.25 @20Hz×2s → StopMove →
  段2 0.20 → 段3 0.15 → 段4 0.10 → StopMove 收尾。
段1 狗走 = 破案（0.2 阈值实锤）；段1 都不走 = 真有权限门，
回到 api_snoop/SwitchJoystick/useRemoteCommandFromApi 序列。
无租约（课程就是无租约直发）。狗总位移 ≤1m。
"""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

SPEEDS = (0.25, 0.20, 0.15, 0.10)
SEG_S = 2.0      # 每段时长
RATE = 20        # Hz

ChannelFactoryInitialize(0, 'eth1')

sc = SportClient()
sc.SetTimeout(10.0)
sc.Init()

code = sc.BalanceStand()
print('BalanceStand code=%s — 狗应站起，等 2.5s' % code, flush=True)
time.sleep(2.5)

for vx in SPEEDS:
    print('>>> 段 %s: Move(%.2f) @%dHz ×%.1fs — 预期位移 %s m（<0.2 应假死）'
          % (SPEEDS.index(vx) + 1, vx, RATE, SEG_S, round(vx * SEG_S, 2)),
          flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < SEG_S:
        sc.Move(vx, 0.0, 0.0)
        time.sleep(1.0 / RATE)
    sc.StopMove()
    print('    StopMove，停 2.5s 观察狗是否真动了', flush=True)
    time.sleep(2.5)

print('=== 全段完成：哪几段狗物理走了？走=对应速度过阈值 ===', flush=True)
