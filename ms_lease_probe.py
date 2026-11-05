#!/usr/bin/env python3
"""ms_lease_probe.py — motion_switcher 带租约试探（跑在 Orin）

事实链更新：sport 租约固件秒批，但带租约 Move 仍不执行 → 门在**模式层**：
当前 mcf（App/AI 控制栈）模式下，平移指令被路由给 AI 栈而非底层运动执行器；
姿态类放行。出口是把模式切回 normal/sport_mode，但 SelectMode 此前一直 7004（拒）
——而那时我们从没申请过 motion_switcher 的租约。本脚本补上这块拼图：
  MS(enableLease=True) → SelectMode('normal') → 成功则立刻
  SportClient(enableLease=True) BalanceStand + Move(0.10) 2.5s + StopMove。
切模式本身不应产生运动（狗保持站立）；Move 放行则向前 ~25cm。
用法: python3 ms_lease_probe.py [模式名]   默认 normal
"""
import json
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.rpc.client import Client

MODE = sys.argv[1] if len(sys.argv) > 1 else 'normal'


class MS(Client):
    def __init__(self):
        super().__init__('motion_switcher', True)  # 关键：带租约

    def Init(self):
        self._SetApiVerson('1.0.0.1')
        for api in (1001, 1002, 1003):
            self._RegistApi(api, 0)

    def CheckMode(self):
        return self._Call(1001, '{}')

    def SelectMode(self, name):
        return self._Call(1002, json.dumps({'name': name}))


def wait_lease(cli, tag, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        lid = cli.GetLeaseId()
        if lid:
            print('%s lease id = %s（%.1fs）' % (tag, lid, time.monotonic() - t0), flush=True)
            return lid
        time.sleep(0.2)
    print('%s 租约没拿到！' % tag, flush=True)
    return None


ChannelFactoryInitialize(0, 'eth1')

ms = MS()
ms.Init()
if not wait_lease(ms, 'motion_switcher'):
    sys.exit(1)

code, data = ms.CheckMode()
print('CheckMode(带租约): code=%s data=%s' % (code, data), flush=True)

code, data = ms.SelectMode(MODE)
print('SelectMode(%r) 带租约: code=%s data=%s' % (MODE, code, data), flush=True)
time.sleep(1.0)
code, data = ms.CheckMode()
print('CheckMode after: code=%s data=%s' % (code, data), flush=True)

if code == 0 and '"name":"%s"' % MODE in (data or '').replace(' ', ''):
    print('>>> 模式已切换！立刻带租约测 Move（狗向前 ~25cm）', flush=True)
    sc = SportClient(enableLease=True)
    sc.Init()
    if wait_lease(sc, 'sport'):
        code = sc.BalanceStand()
        print('BalanceStand code=%s' % code, flush=True)
        time.sleep(2.0)
        code = sc.Move(0.10, 0.0, 0.0)
        print('Move(0.10,0,0) code=%s' % code, flush=True)
        time.sleep(2.5)
        code = sc.StopMove()
        print('StopMove code=%s' % code, flush=True)
else:
    print('>>> 模式没切过去（7004=仍被拒）。下一步才轮到 ReleaseMode（需人工确认）。', flush=True)