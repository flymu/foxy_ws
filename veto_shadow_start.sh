#!/bin/bash
# veto_shadow_start.sh — C++ 影子否决层在 Orin 上一键启动（2026-11-06 影子阶段）
# 前提: veto_ws 已构建（colcon build），Python veto 由 patrol_safety_start.sh 照常运行。
# 输出全部 remap 到 *_shadow 话题，不碰生产链；节点名改 obstacle_veto_shadow。
# ⚠ patrol_safety_start.sh 里的 pkill "[o]bstacle_veto" 会连坐本影子（重跑它
#   前先看完本阶段）；反过来本脚本只杀 shadow，不动 Python 生产版。
# 用法: ssh go2_lan 'bash /home/unitree/veto_shadow_start.sh'
# 停止: ssh go2_lan 'pkill -f "[o]bstacle_veto_shadow"'
pkill -f "[o]bstacle_veto_shadow" 2>/dev/null
sleep 1
source /opt/ros/foxy/setup.bash
source /home/unitree/veto_ws/install/setup.bash
cd /home/unitree
setsid nohup ros2 run obstacle_veto_cpp obstacle_veto_node --ros-args \
  -r /go2/cmd_vel:=/go2/cmd_vel_shadow \
  -r /veto/status:=/veto/status_shadow \
  -r /veto/cmd:=/veto/cmd_shadow \
  -r __node:=obstacle_veto_shadow \
  > /tmp/obstacle_veto_shadow.log 2>&1 < /dev/null &
sleep 2
echo "存活: $(pgrep -c -f '[o]bstacle_veto_shadow' || echo 0) 个"
tail -n 5 /tmp/obstacle_veto_shadow.log
