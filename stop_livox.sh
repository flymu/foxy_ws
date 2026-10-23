#!/bin/bash
# stop_livox.sh — 停掉 Orin 上的 livox 驱动（bag 回放离线调试前必须停，
# 否则驱动与 bag 在 /livox/lidar 上双路互灌、时间戳错乱，LIO 必炸）
pkill -f "ros2 launch livox_ros_driver2" 2>/dev/null
pkill -f livox_ros_driver2_node 2>/dev/null
sleep 1
if pgrep -f livox_ros_driver2 >/dev/null; then
  echo "仍残留:"; pgrep -af livox_ros_driver2 | head -3
else
  echo "livox driver stopped"
fi
