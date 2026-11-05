#!/usr/bin/env python3
"""sport_probe.py — 一次性：绕开桥，unitree_sdk2py 直呼 sport 服务（跑在 Orin）
看三件事：BalanceStand(有应答 RPC) 结果码 + 狗有没有姿态反应；Move noreply；StopMove。
"""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, 'eth1')
sc = SportClient()
sc.Init()

code = sc.BalanceStand()
print('BalanceStand code=%s   <- 有应答RPC：-1=拒收 0=接受' % code, flush=True)
time.sleep(2.5)

code = sc.Move(0.10, 0.0, 0.0)
print('Move(0.10,0,0) code=%s  <- noreply，0只代表已发' % code, flush=True)
time.sleep(3.0)

code = sc.StopMove()
print('StopMove code=%s' % code, flush=True)
