#!/usr/bin/env python3
"""release_switch_probe.py — ReleaseMode → SelectMode('normal') 之舞（跑在 Orin）

证据收敛（SelectMode 之谜）：
  - SelectMode('mcf') code=0：API 开放，重选已持有的模式=允许；
  - SelectMode('normal'/'sport_mode') prio=0 → 7004：往别的栈切被拒；
  - prio>0 → 7002（更早被拒）；ServiceSwitch 拉服务 → 5201 起不来；
  - 官方示例的别名名（normal/advanced/ai）确认无误。
  → 7004 的机制指向：**当前栈被占用**，须先 ReleaseMode(1003) 释放再选
  （这正是 ReleaseMode 存在的意义，也是社区经验：App/遥控占着 motion 栈时
  先 release 再 select）。
流程：CheckMode → ReleaseMode（狗会趴下！）→ 2s → SelectMode('normal') →
  成功：CheckMode 确认 → BalanceStand 站起 → 租约 Move(0.10)×3s（狗向前~30cm）
  → StopMove → 报捷。
  失败：SelectMode('mcf') 把狗切回原模式 → BalanceStand 站起 → 报败。
"""
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import \
    MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, 'eth1')

ms = MotionSwitcherClient()
ms.SetTimeout(5.0)
ms.Init()

code, mode = ms.CheckMode()
print('CheckMode 起点: code=%s name=%s' % (code, mode), flush=True)

# ---------- 1. 释放当前持有的运动栈（狗会趴下） ----------
code, _ = ms.ReleaseMode()
print('ReleaseMode: code=%s  <- 狗应该趴下了' % code, flush=True)
time.sleep(2.0)

# ---------- 2. 切到 normal（SDK 开发者栈） ----------
code, _ = ms.SelectMode('normal')
print("SelectMode('normal'): code=%s" % code, flush=True)

if code == 0:
    time.sleep(1.0)
    code, mode = ms.CheckMode()
    print('CheckMode: code=%s name=%s' % (code, mode), flush=True)
    print('>>> 切换成功！BalanceStand 站起 + 租约 Move 测试 <<<', flush=True)
    sc = SportClient(enableLease=True)
    sc.Init()
    t0 = time.monotonic()
    while not sc.GetLeaseId() and time.monotonic() - t0 < 8.0:
        time.sleep(0.2)
    print('lease id=%s' % sc.GetLeaseId(), flush=True)
    sc.BalanceStand()
    print('BalanceStand 发出，狗站起，等 2.5s', flush=True)
    time.sleep(2.5)
    print('>>> Move(0.10) @20Hz ×3s — 狗向前 ~30cm <<<', flush=True)
    t1 = time.monotonic()
    while time.monotonic() - t1 < 3.0:
        sc.Move(0.10, 0.0, 0.0)
        time.sleep(0.05)
    sc.StopMove()
    print('>>> StopMove 完成 <<<', flush=True)
else:
    # 失败兜底：切回 mcf，狗别裸奔在无模式态
    code2, _ = ms.SelectMode('mcf')
    print("兜底 SelectMode('mcf'): code=%s" % code2, flush=True)
    time.sleep(1.0)
    code3, mode = ms.CheckMode()
    print('CheckMode 兜底后: code=%s name=%s' % (code3, mode), flush=True)
    sc = SportClient()
    sc.Init()
    code4 = sc.BalanceStand()
    print('BalanceStand(无租约姿态指令) code=%s — 用遥控器也可站起' % code4,
          flush=True)