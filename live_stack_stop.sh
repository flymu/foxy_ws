#!/bin/bash
# live_stack_stop.sh — 一键停巡逻栈（foxy-live 容器内跑；SIGINT 干净退出）
# 宿主机用法:
#   sg docker -c 'docker exec foxy-live bash /home/mu/foxy_ws/live_stack_stop.sh'
pkill -INT -f "[s]can_planner_node" || true
sleep 0.5
pkill -INT -f "[m]ap_localization_node" || true
pkill -INT -f "[p]oint_lio_node" || true
pkill -INT -f "[v]iz_helper" || true
sleep 2
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
echo "存活: $(ps aux | grep -cE '[p]oint_lio_node|[m]ap_localization_node|[s]can_planner_node|[v]iz_helper') (僵尸进程不计)"
