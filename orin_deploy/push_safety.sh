#!/bin/bash
# push_safety.sh — 把工作站上的安全栈最新副本推到 Orin（2026-11-05 G1 起）
# 用法: bash orin_deploy/push_safety.sh
# 背景：Orin 上 /home/unitree/ 的运行副本靠手工 scp 维护，狗断电时改完本地
# 容易忘推——下次实弹跑的还是旧逻辑。跑本脚本对齐后再上电测试。
# 铁律不变：Orin 侧只许 unitree 用户跑这些文件（root 跑 DDS 毒化 SHM 发现）。
set -e
cd "$(dirname "$0")/.."
FILES="obstacle_veto.py veto_core.py veto_viz.py cloud_sparsify.py \
go2_bridge.py bridge_core.py go2_avoid_switch.py veto_status_snoop.py tf_snoop.py \
creep_test.py livox_watchdog.sh"
echo "== 推送: $FILES"
scp -q $FILES go2_wifi:/home/unitree/
echo "== 完成。下次实弹前的开机检查单："
echo "  详见 orin_deploy/POWERON_CHECKLIST.md，顺序概要："
echo "  1. ssh go2_wifi 'bash -s' < start_livox.sh                     # 雷达"
echo "  2. ssh go2_wifi 'setsid nohup bash /home/unitree/livox_watchdog.sh >/dev/null 2>&1 < /dev/null &'"
echo "  3. ssh go2_wifi 'python3 /home/unitree/go2_avoid_switch.py get'  # 自带避障必须 off"
echo "  4. ssh go2_wifi 'bash /home/unitree/patrol_safety_start.sh [viz]' # relay+veto(G1)"
echo "  5. 桥: ssh go2_wifi 'bash /home/unitree/go2_bridge_start.sh'      # v4 停车锁存版"
echo "  6. 狗唤醒后 known-start/续接 重定位（point_lio 位姿已脏）"