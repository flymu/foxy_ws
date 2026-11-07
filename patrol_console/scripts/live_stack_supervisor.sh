#!/bin/bash
# live_stack_supervisor.sh — 巡检栈在控制台里的看护进程（stacks.yaml 里 live_stack 的 cmd）。
# 为什么需要它：live_stack_start.sh 起完节点自己就退了，直接当栈命令跑会让控制台
#   状态立刻变 exited（节点还在）。本脚本跑完它之后原地等节点退场：
#   脚本里 nohup 不换进程组，节点全都落在本进程的进程组里，所以这里按名字等得到，
#   控制台的按组信号也够得着（兜底路径）。
# 停止走 live_stack_stop.sh（stacks.yaml 里的 stop_cmd）：按节点名有序 pkill，
#   节点没了本循环自然退出，控制台状态 running → exited。
bash /home/mu/foxy_ws/live_stack_start.sh
status=$?
# 等节点退场。不能直接 pgrep -f：容器 PID1 不收割（老病），僵尸进程 cmdline 为空，
# pgrep 会退回按进程名匹配——9 个 11-06 留下的 point_lio 僵尸就能把本循环卡死，
# 停止退化成对进程组的 SIGKILL 连坐。用 ps 的状态列把 Z 滤掉。
# 模式与 live_stack_start.sh 的存活检查同一套；go2_camera_republish 刻意不在里面
#（起栈脚本对它就是"没有才补"的常驻助手，栈停了它继续活着）。
stack_alive() {
  ps -eo stat=,args= | awk '
    $1 !~ /^Z/ && /[p]oint_lio_node|[m]ap_localization_node|[s]can_planner_node|[s]peed_adapter|[v]iz_helper/ { found = 1 }
    END { exit !found }'
}
while stack_alive; do
  sleep 2
done
exit $status
