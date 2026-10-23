#!/bin/bash
# start_livox.sh — 启动 Orin 上的 livox 驱动（后台，日志 /tmp/livox_driver.log）
# 当前 launch 配置 xfer_format=1（CustomMsg，Point-LIO 需要）
pgrep -f livox_ros_driver2 >/dev/null && { echo "driver already running"; exit 0; }
nohup bash -c "source /opt/ros/foxy/setup.bash && source ~/livox_ws/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py" >/tmp/livox_driver.log 2>&1 &
sleep 3
pgrep -f livox_ros_driver2 >/dev/null && echo "livox driver started (pid $(pgrep -f livox_ros_driver2 | head -1))" || { echo "FAILED — 看 /tmp/livox_driver.log"; tail -3 /tmp/livox_driver.log; }
