#!/usr/bin/env python3
"""
go2_avoid_switch.py — 查询/开关 Go2 自带避障（obstacles_avoid 服务，跑在 Orin 上）

用法（Orin 上，无需 source ROS）:
  python3 go2_avoid_switch.py get   # 只查询当前开关
  python3 go2_avoid_switch.py off   # 关闭自带避障
  python3 go2_avoid_switch.py on    # 重新开启

环境与 go2_bridge_start.sh 同款（PYTHONPATH=SDK源码树, LD_LIBRARY_PATH=cyclonedds）。

背景（11-05 遥控不动排查）:
  - 该固件遥控/APP 指令由 obstacles_avoid 服务转发（api 1004
    UseRemoteCommandFromApi 即证据），自带避障开着=朝障碍方向杆量被吃掉，
    表现为"遥控不动"。
  - 巡检平台上安全由 Mid360+veto 层负责；自带避障必须关闭，否则固件
    在障碍前截停会污染蠕行测试/导航测量（狗停哪=固件说了算，不是 veto）。
  - 遥控机动时 veto 不在线（Orin 重启/栈未起），此时狗无任何避障——
    人就是避障器，注意留余量。
  - 开关是否跨狗断电持久化未验证：每次实弹前 get 确认一遍。
"""
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import (
    ObstaclesAvoidClient,
)


def main():
    op = sys.argv[1] if len(sys.argv) > 1 else "get"
    if op not in ("get", "on", "off"):
        print("用法: go2_avoid_switch.py [get|on|off]")
        return 2
    ChannelFactoryInitialize(0, "eth1")
    cli = ObstaclesAvoidClient()
    cli.Init()
    try:
        code, enable = cli.SwitchGet()
    except Exception as e:  # noqa: BLE001
        print("SwitchGet 异常: %s（狗没开？eth1 不通？）" % e)
        return 1
    if code != 0:
        print("SwitchGet 失败 code=%d（服务不在线/模式不受理）" % code)
        return 1
    print("当前自带避障: %s" % ("开" if enable else "关"))
    if op == "get" or enable == (op == "on"):
        return 0
    want = (op == "on")
    try:
        code = cli.SwitchSet(want)
    except Exception as e:  # noqa: BLE001
        print("SwitchSet 异常: %s" % e)
        return 1
    print("SwitchSet(%s) code=%d" % (want, code))
    code, enable = cli.SwitchGet()
    ok = (code == 0 and enable == want)
    print("变更后自带避障: %s (code=%d)" % ("开" if enable else "关", code))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
