#!/bin/bash
# live_stack_start.sh — 工作站实时巡逻栈一键启动（在 foxy-live 容器内跑）
# 宿主机用法:
#   sg docker -c 'docker exec foxy-live bash /home/mu/foxy_ws/live_stack_start.sh'
# 一键停:    live_stack_stop.sh
# 前提（Orin 侧，均已开机自启/常驻）:
#   livox 驱动、go2_bridge.py(控制桥)、go2_odom_bridge.py(四足航位推算桥)
#   开始实验时把狗唤醒、牵到建图起点附近再起本脚本（known-start 初始化依赖起点）。
# 相机链路的 republish 若不在跑会顺手补一个（foxy-live 容器重启后需要）。
MAP=/home/mu/maps/lio_1792684457_global_map.pcd
LOG=/tmp/live_stack
mkdir -p $LOG

pkill -f "[m]ap_localization_node" || true
pkill -f "[p]oint_lio_node" || true
pkill -f "[s]can_planner_node" || true
pkill -f "[v]iz_helper" || true
sleep 1
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*

source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash

nohup /home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2/point_lio_node \
    --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $LOG/lio.log 2>&1 &
nohup /home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2/map_localization_node \
    --ros-args -p map_file:=$MAP > $LOG/loc.log 2>&1 &
nohup /home/mu/foxy_ws/install_x86/scan_planner_ros2/lib/scan_planner_ros2/scan_planner_node \
    --ros-args -r /cmd_vel:=/go2/cmd_vel -p max_vx:=0.4 > $LOG/planner.log 2>&1 &
nohup python3 /home/mu/foxy_ws/viz_helper.py > $LOG/viz.log 2>&1 &
if ! pgrep -f "[g]o2_camera_republish" > /dev/null; then
    nohup python3 /home/mu/foxy_ws/go2_camera_republish.py > $LOG/camera.log 2>&1 &
fi

sleep 2
echo "存活节点数: $(ps aux | grep -cE '[p]oint_lio_node|[m]ap_localization_node|[s]can_planner_node|[v]iz_helper')/4"
echo "=== live stack up, logs in $LOG ==="
echo "健康状态: timeout -s INT 8 ros2 topic echo /loc/health"
echo "（狗唤醒后 Orin go2_odom_bridge 才有 /go2/odom 数据，看门狗随之激活）"
