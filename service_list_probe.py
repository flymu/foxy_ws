#!/usr/bin/env python3
"""service_list_probe.py — robot_state 服务清单查询（只读，跑在 Orin）

robot_state api: 1001=ServiceSwitch 1002=ReportFreq 1003=ServiceList。
ServiceList 列出固件里全部服务 {name, status, protect}：
  status: 0=停止 1=运行（2=保护态？以实读为准）  protect: True=受保护不可开关
目的：搞清 SelectMode 7004 的真因——目标运动服务是不是根本没在跑。
纯查询，不发任何控制指令，狗不会有反应。
用法: python3 service_list_probe.py
"""
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

ChannelFactoryInitialize(0, 'eth1')
rs = RobotStateClient()
rs.Init()

code, lst = rs.ServiceList()
print('ServiceList code=%s, %d services' % (code, 0 if lst is None else len(lst)),
      flush=True)
if lst:
    running = stopped = 0
    for s in sorted(lst, key=lambda x: (x.status, x.name)):
        flag = {0: 'STOP', 1: 'RUN ', 2: 'PROT?'}.get(s.status, str(s.status))
        print('%-28s %s protect=%s' % (s.name, flag, s.protect), flush=True)
        running += s.status == 1
        stopped += s.status == 0
    print('--- 运行中 %d 个 / 停止 %d 个 ---' % (running, stopped), flush=True)