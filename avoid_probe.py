#!/usr/bin/env python3
"""avoid_probe.py — 狗自带避障开关试探（跑在 Orin）

用户假设：狗下颌 L1 激光的避障栈掐掉了 SDK 平移（姿态放行）。
本脚本一把梭：
  1. 只读侦察：robot_state ServiceList（服务运行状态清单）
                 + obstacles_avoid SwitchGet（避障当前开还是关）
  2. SwitchSet(False) 关掉狗自带避障，复读确认
  3. 段A：sport.Move 连发 0.10m/s×3s（带租约）——标准 SDK 路径
  4. 段B：obstacles_avoid.Move 连发 0.10m/s×3s——mcf/AI 模式运动路径
  5. SwitchSet(True) 恢复狗自带避障（留场如初）
注意：段A/段B 狗若响应会各向前 ~30cm，请人工看着。
"""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import \
    ObstaclesAvoidClient
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, 'eth1')

# ---------- 1. 只读侦察 ----------
rs = RobotStateClient()
rs.Init()
code, lst = rs.ServiceList()
print('[侦察] ServiceList code=%s, %s services' % (
    code, 0 if lst is None else len(lst)), flush=True)
if lst:
    for s in sorted(lst, key=lambda x: (x.status, x.name)):
        flag = {0: 'STOP', 1: 'RUN '}.get(s.status, str(s.status))
        print('  %-28s %s protect=%s' % (s.name, flag, s.protect), flush=True)

oa = ObstaclesAvoidClient()
oa.Init()
code, en = oa.SwitchGet()
print('[侦察] 避障开关当前: code=%s enable=%s' % (code, en), flush=True)

# ---------- 2. 关掉狗自带避障 ----------
code = oa.SwitchSet(False)
code2, en2 = oa.SwitchGet()
print('[操作] SwitchSet(False) code=%s，复读 enable=%s' % (code, en2), flush=True)
time.sleep(1.0)

# ---------- 3. 段A：sport.Move（租约） ----------
sc = SportClient(enableLease=True)
sc.Init()
t0 = time.monotonic()
while not sc.GetLeaseId() and time.monotonic() - t0 < 8.0:
    time.sleep(0.2)
print('[租约] sport lease id=%s' % sc.GetLeaseId(), flush=True)

sc.BalanceStand()
time.sleep(2.0)
print('>>> 段A开始: sport.Move(0.10) @20Hz ×3s — 狗若响应向前~30cm <<<', flush=True)
t1 = time.monotonic()
while time.monotonic() - t1 < 3.0:
    sc.Move(0.10, 0.0, 0.0)
    time.sleep(0.05)
sc.StopMove()
print('>>> 段A结束，歇 3s <<<', flush=True)
time.sleep(3.0)

# ---------- 4. 段B：obstacles_avoid.Move（AI 路径） ----------
print('>>> 段B开始: obstacles_avoid.Move(0.10) @20Hz ×3s <<<', flush=True)
t2 = time.monotonic()
while time.monotonic() - t2 < 3.0:
    oa.Move(0.10, 0.0, 0.0)
    time.sleep(0.05)
sc.StopMove()
print('>>> 段B结束 <<<', flush=True)
time.sleep(1.0)

# ---------- 5. 恢复狗自带避障 ----------
code = oa.SwitchSet(True)
code2, en2 = oa.SwitchGet()
print('[收尾] 已恢复避障: SwitchSet(True) code=%s，复读 enable=%s' % (
    code, en2), flush=True)