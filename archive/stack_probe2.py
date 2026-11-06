#!/usr/bin/env python3
"""stack_probe2.py — 正确别名 + 服务拉起 + 优先级三合一试探（跑在 Orin）

新证据（SDK 官方示例 example/motionSwitcher）：
  SelectMode 的参数是**别名**："ai" / "normal" / "advanced" / "ai-w"(轮式)，
  不是服务名！normal = 开发者 SDK 运动栈（对应 sport_mode 服务），我们
  之前一直用错名字。且 SelectMode('mcf') code=0 证明 API 对我们开放。
假说：
  a) 需要先把 sport_mode/advanced_sport 服务拉起来再切（上次 ServiceSwitch
     立即返回 5201，可能异步起服务需要时间——这次等 8s 看状态）；
  b) 优先级：RequestPolicy.priority，我们一直用 0，内部调用方可能用更高
     优先级才被允许切栈——逐档试 0/1/2/5。
流程：拉起 sport_mode + advanced_sport → 等 8s → ServiceList →
  SelectMode('normal') 逐优先级试 → 不行再 SelectMode('advanced') →
  CheckMode 确认 → 切成功 → 租约 BalanceStand + Move(0.10)×3s（狗向前~30cm）→ StopMove。
切栈时狗可能有姿态起伏，人工盯着；切换成功即测试 Move。
"""
import json
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import \
    MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

MOTION_SERVICES = ('mcf', 'sport_mode', 'ai_sport', 'advanced_sport', 'ota_box')

ChannelFactoryInitialize(0, 'eth1')

rs = RobotStateClient()
rs.Init()


def show_services(tag):
    code, lst = rs.ServiceList()
    print('[%s] ServiceList code=%s' % (tag, code), flush=True)
    if lst:
        for s in sorted(lst, key=lambda x: x.name):
            if s.name in MOTION_SERVICES:
                print('  %-16s status=%s protect=%s' % (s.name, s.status, s.protect),
                      flush=True)


show_services('起点')

# ---------- 1. 拉起两个停着的运动栈服务 ----------
for name in ('sport_mode', 'advanced_sport'):
    code = rs.ServiceSwitch(name, True)
    print('ServiceSwitch(%s,True) code=%s' % (name, code), flush=True)
    time.sleep(1.0)
print('等 8s 让服务起来...', flush=True)
time.sleep(8.0)
show_services('拉起后')

# ---------- 2. SelectMode 逐别名逐优先级 ----------
ms = MotionSwitcherClient()
ms.SetTimeout(5.0)
ms.Init()

switched = None
for alias in ('normal', 'advanced'):
    for prio in (0, 1, 2, 5):
        ms._RegistApi(1002, prio)
        code, _ = ms.SelectMode(alias)
        print('SelectMode(%r) prio=%s -> code=%s' % (alias, prio, code), flush=True)
        if code == 0:
            switched = alias
            break
        time.sleep(0.5)
    if switched:
        break

time.sleep(1.0)
code, mode = ms.CheckMode()
print('CheckMode: code=%s name=%s' % (code, mode), flush=True)

# ---------- 3. 切成功 → Move 实测 ----------
if switched:
    print('>>> 已切到 %r！租约 Move 测试（狗向前 ~30cm）<<<' % switched, flush=True)
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
    print('>>> 还是没切过去。收尾：停掉拉起的服务。', flush=True)
    for name in ('sport_mode', 'advanced_sport'):
        code = rs.ServiceSwitch(name, False)
        print('收尾 ServiceSwitch(%s,False) code=%s' % (name, code), flush=True)
    show_services('收尾后')