#!/usr/bin/env python3
"""stack_switch_probe.py — 拉起停着的运动栈再切模式（跑在 Orin）

推理链（2026-11-03 侦察结论）：
  ServiceList 语义实为 status 0=运行 1=停止（robot_state 自己 status=0 还在应答）。
  实测快照：mcf 运行中（CheckMode=mcf 吻合），sport_mode/ai_sport/advanced_sport
  三个运动栈服务**全停** → SelectMode('sport_mode') 7004 的真因：目标栈没在跑。
  robot_state api 1001 ServiceSwitch 可以启停服务，sport_mode protect=0 可开关。
实验：
  1. ServiceSwitch('sport_mode', True) 拉起 SDK 运动栈
  2. SelectMode('sport_mode')（失败再试 'normal'）→ CheckMode 确认
  3. 切换成功 → 租约 BalanceStand + Move(0.10) 连发 3s（狗向前 ~30cm）→ StopMove
注意：运动栈切换时狗可能有姿态起伏；切到 SDK 栈后狗自带 AI 避障旁路
（11-03 事故的前提条件）——本次移动量极小，人工盯着。
"""
import json
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.rpc.client import Client

MOTION_SERVICES = ('mcf', 'sport_mode', 'ai_sport', 'advanced_sport', 'ota_box')


class MS(Client):
    def __init__(self):
        super().__init__('motion_switcher', False)

    def Init(self):
        self._SetApiVerson('1.0.0.1')
        for api in (1001, 1002, 1003):
            self._RegistApi(api, 0)

    def CheckMode(self):
        return self._Call(1001, '{}')

    def SelectMode(self, name):
        return self._Call(1002, json.dumps({'name': name}))


def show_services(rs, tag):
    code, lst = rs.ServiceList()
    print('[%s] ServiceList code=%s' % (tag, code), flush=True)
    if lst:
        for s in sorted(lst, key=lambda x: x.name):
            if s.name in MOTION_SERVICES:
                print('  %-16s status=%s protect=%s' % (s.name, s.status, s.protect),
                      flush=True)


ChannelFactoryInitialize(0, 'eth1')

rs = RobotStateClient()
rs.Init()
show_services(rs, '切换前')

# ---------- 1. 拉起 sport_mode 服务 ----------
code = rs.ServiceSwitch('sport_mode', True)
print('ServiceSwitch(sport_mode,True) code=%s' % code, flush=True)
time.sleep(3.0)
show_services(rs, '切换后')

# ---------- 2. 切模式 ----------
ms = MS()
ms.Init()
switched = None
for name in ('sport_mode', 'normal'):
    code, data = ms.SelectMode(name)
    print('SelectMode(%r) code=%s data=%s' % (name, code, data), flush=True)
    if code == 0:
        switched = name
        break
    time.sleep(0.5)

time.sleep(1.0)
code, data = ms.CheckMode()
print('CheckMode: code=%s data=%s' % (code, data), flush=True)

# ---------- 3. 切换成功才测 Move ----------
if switched:
    print('>>> 模式已切到 %r，租约 Move 测试（狗向前 ~30cm）<<<' % switched, flush=True)
    sc = SportClient(enableLease=True)
    sc.Init()
    t0 = time.monotonic()
    while not sc.GetLeaseId() and time.monotonic() - t0 < 8.0:
        time.sleep(0.2)
    print('lease id=%s' % sc.GetLeaseId(), flush=True)
    sc.BalanceStand()
    time.sleep(2.0)
    print('>>> Move(0.10) @20Hz ×3s <<<', flush=True)
    t1 = time.monotonic()
    while time.monotonic() - t1 < 3.0:
        sc.Move(0.10, 0.0, 0.0)
        time.sleep(0.05)
    sc.StopMove()
    print('>>> StopMove 完成 <<<', flush=True)
else:
    print('>>> 模式没切过去，不发 Move。', flush=True)
    # 收尾：把拉起的服务停回去，狗回到原状
    code = rs.ServiceSwitch('sport_mode', False)
    print('收尾 ServiceSwitch(sport_mode,False) code=%s' % code, flush=True)