#!/usr/bin/env python3
"""recover_mcf.py — 紧急恢复：把 mcf 栈拉回来（跑在 Orin）
ReleaseMode 后狗趴着且无模式（CheckMode name=''，sport RPC 3104 超时）。
7002 语义已明确 = 目标栈没在跑。恢复路径：
  ServiceSwitch('mcf', True) 重启 mcf 服务 → SelectMode('ai'/'mcf') → CheckMode。
都不行的话，遥控器手动恢复（用户实测过遥控能蹲下/站起/走）。
"""
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import \
    MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

ChannelFactoryInitialize(0, 'eth1')

rs = RobotStateClient()
rs.Init()
code = rs.ServiceSwitch('mcf', True)
print('ServiceSwitch(mcf,True) code=%s' % code, flush=True)
time.sleep(5.0)

code, lst = rs.ServiceList()
if lst:
    for s in sorted(lst, key=lambda x: x.name):
        if s.name in ('mcf', 'sport_mode', 'ai_sport', 'advanced_sport'):
            print('  %-16s status=%s' % (s.name, s.status), flush=True)

ms = MotionSwitcherClient()
ms.SetTimeout(5.0)
ms.Init()
for name in ('ai', 'mcf'):
    code, _ = ms.SelectMode(name)
    print('SelectMode(%r) code=%s' % (name, code), flush=True)
    if code == 0:
        break
    time.sleep(0.5)

time.sleep(1.0)
code, mode = ms.CheckMode()
print('CheckMode: code=%s name=%s' % (code, mode), flush=True)