#!/bin/bash
# live_stack_stop.sh — 一键停巡逻栈（foxy-live 容器内跑；SIGINT 干净退出）
# 宿主机用法:
#   sg docker -c 'docker exec foxy-live bash /home/mu/foxy_ws/live_stack_stop.sh'
pkill -INT -f "[s]can_planner_node" || true
pkill -INT -f "[s]peed_adapter" || true
pkill -INT -f "[o]bstacle_veto" || true   # 仅清本机走丢的旧副本；Orin veto 用 patrol_safety_stop.sh
sleep 0.5
pkill -INT -f "[m]ap_localization_node" || true
pkill -INT -f "[p]oint_lio_node" || true
pkill -INT -f "[v]iz_helper" || true
sleep 2
# veto 是 python 节点，SIGINT 偶有不退的（viz_helper 同款病），兜底 kill -9
pkill -9 -f "[o]bstacle_veto" 2>/dev/null || true   # 同上，只清本机旧副本
pkill -9 -f "[v]iz_helper" 2>/dev/null || true
pkill -9 -f "[s]peed_adapter" 2>/dev/null || true
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
echo "存活: $(ps aux | grep -cE '[p]oint_lio_node|[m]ap_localization_node|[s]can_planner_node|[s]peed_adapter|[v]iz_helper') (僵尸进程不计)"
