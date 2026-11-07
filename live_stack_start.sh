#!/bin/bash
# live_stack_start.sh — 工作站实时巡逻栈一键启动（在 foxy-live 容器内跑）
# 宿主机用法:
#   sg docker -c 'docker exec foxy-live bash /home/mu/foxy_ws/live_stack_start.sh'
# 一键停:    live_stack_stop.sh
# 前提（Orin 侧）:
#   常驻: livox 驱动、go2_bridge.py(控制桥)、go2_odom_bridge.py(四足航位推算桥)
#   安全栈(2026-11-05 D1+D2 起迁移驻 Orin，起本脚本前要另起):
#     ssh go2_wifi 'bash /home/unitree/patrol_safety_start.sh [viz]'
#     (livox_relay 中继 + obstacle_veto 否决层；/livox/lidar 的 WiFi 订阅者只剩 point_lio)
#   开始实验时把狗唤醒、牵到建图起点附近再起本脚本（known-start 初始化依赖起点）。
# 相机链路的 republish 若不在跑会顺手补一个（foxy-live 容器重启后需要）。
MAP=/home/mu/maps/lio_1792684457_global_map.pcd
LOG=/tmp/live_stack
mkdir -p $LOG

pkill -f "[m]ap_localization_node" || true
pkill -f "[p]oint_lio_node" || true
pkill -f "[s]can_planner_node" || true
pkill -f "[s]peed_adapter" || true
pkill -f "[o]bstacle_veto" || true   # 只清本机走丢的旧副本（veto 已驻 Orin，勿在本机起）
pkill -f "[v]iz_helper" || true
pkill -f "[g]o2_camera_republish" || true  # 11-06: 不杀它就 rm SHM 会毒化它（Orin 同型事故）
sleep 3  # 11-06 实弹：上代 LIO 数GB IKD 树要时间释放，杀完 1s 就起新栈会 bad_alloc
# SHM 清理（默认开：CLI 恢复流程用，清掉崩死节点留下的段/信号量）。
# ⚠️ 控制台在场时必须跳过（LIVE_STACK_KEEP_SHM=1，控制台的 stacks.yaml 已配）：
#    11-07 实测 rm 中途删 /dev/shm/fastrtps_port* 会把新旧参与者的 DDS 发现整个劈成
#    两代——先起的参与者（控制台/探针）再也发现不了后起的节点（控制台看不到
#    规划器、点云、健康，互斥门失效）。控制台流程起停都干净（SIGINT），
#    段用不着重建；栈崩了要恢复就 CLI 直跑本脚本（不带环境变量，rm 会执行）。
if [ "$LIVE_STACK_KEEP_SHM" != "1" ]; then
  rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
fi

source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash

nohup /home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2/point_lio_node \
    --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $LOG/lio.log 2>&1 &
nohup /home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2/map_localization_node \
    --ros-args -p map_file:=$MAP > $LOG/loc.log 2>&1 &
# G1(2026-11-05): 规划器输出改发 /cmd_vel_intent，经 speed_adapter（起步地板/
# 停步死区——狗物理上只有 ≥0.25 或 0 两档）适配后才成为 /cmd_vel_raw 给 veto。
nohup /home/mu/foxy_ws/install_x86/scan_planner_ros2/lib/scan_planner_ros2/scan_planner_node \
    --ros-args -r /cmd_vel:=/cmd_vel_intent -p max_vx:=0.4 > $LOG/planner.log 2>&1 &
nohup python3 /home/mu/foxy_ws/speed_adapter.py > $LOG/adapter.log 2>&1 &
# P0-S 独立避障否决层已驻 Orin（D1+D2，2026-11-05）：/cmd_vel_raw 经 WiFi 小报文
# → Orin veto → /go2/cmd_vel 本地给桥。本机不再起 veto（也不再订雷达 WiFi 大流）。
#   起法: ssh go2_wifi 'bash /home/unitree/patrol_safety_start.sh [viz]'
nohup python3 /home/mu/foxy_ws/viz_helper.py > $LOG/viz.log 2>&1 &
if ! pgrep -f "[g]o2_camera_republish" > /dev/null; then
    # setsid 让它搬出本脚本的进程组：stop.sh 刻意不杀它（"没有才补"的常驻助手），
    # 留在栈的组里会把停止流程吊死——监护进程等不到"组清空"，退化成 SIGKILL 连坐。
    nohup setsid python3 /home/mu/foxy_ws/go2_camera_republish.py > $LOG/camera.log 2>&1 &
fi

sleep 2
echo "存活节点数: $(ps aux | grep -cE '[p]oint_lio_node|[m]ap_localization_node|[s]can_planner_node|[s]peed_adapter|[v]iz_helper')/5"
echo "veto 健康(驻Orin,跨DDS可见): timeout -s INT 4 ros2 topic echo /veto/status --once"
echo "=== live stack up, logs in $LOG ==="
echo "健康状态: timeout -s INT 8 ros2 topic echo /loc/health"
echo "（狗唤醒后 Orin go2_odom_bridge 才有 /go2/odom 数据，看门狗随之激活）"
