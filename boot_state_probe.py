#!/usr/bin/env python3
"""boot_state_probe.py — 开机后全套只读状态（跑在 Orin）
一次读三样：robot_state ServiceList / motion_switcher CheckMode /
obstacles_avoid SwitchGet。全部只读，狗不会有任何反应。
"""
import json

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import \
    ObstaclesAvoidClient
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.rpc.client import Client


class MS(Client):
    def __init__(self):
        super().__init__('motion_switcher', False)

    def Init(self):
        self._SetApiVerson('1.0.0.1')
        for api in (1001, 1002, 1003):
            self._RegistApi(api, 0)

    def CheckMode(self):
        return self._Call(1001, '{}')


ChannelFactoryInitialize(0, 'eth1')

rs = RobotStateClient()
rs.Init()
code, lst = rs.ServiceList()
print('ServiceList code=%s, %s services' % (
    code, 0 if lst is None else len(lst)), flush=True)
if lst:
    for s in sorted(lst, key=lambda x: (x.status, x.name)):
        flag = {0: 'STOP', 1: 'RUN '}.get(s.status, str(s.status))
        print('  %-28s %s protect=%s' % (s.name, flag, s.protect), flush=True)

ms = MS()
ms.Init()
code, data = ms.CheckMode()
print('CheckMode: code=%s data=%s' % (code, data), flush=True)

oa = ObstaclesAvoidClient()
oa.Init()
code, en = oa.SwitchGet()
print('避障开关: code=%s enable=%s' % (code, en), flush=True)