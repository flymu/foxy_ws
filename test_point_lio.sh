#!/bin/bash
# test_point_lio.sh — 离线回放 bag 验 Point-LIO 节点（一键：停驱动→重启节点→回放→出报告）
# 用法: bash ~/test_point_lio.sh <bag_dir>        例: bash ~/test_point_lio.sh ~/bags/test15s
# 注意: 必须 source livox_ws，否则 rosbag2 因缺 livox_ros_driver2 类型定义
#       会静默丢弃 /livox/lidar（上次只放 IMU 就是这个原因）
BAG="${1:?用法: $0 <bag_dir>}"
source /opt/ros/foxy/setup.bash
source ~/livox_ws/install/setup.bash

bash ~/stop_livox.sh
bash ~/point_lio_start.sh
sleep 2

DUR=$(ros2 bag info "$BAG" 2>/dev/null | grep -oP 'Duration: \K[0-9]+(\.[0-9]+)?' | head -1)
SECS=${DUR%.*}; SECS=${SECS:-15}
echo "== 回放 $BAG (${DUR}s) =="
timeout -s INT $((SECS + 8)) ros2 bag play "$BAG" 2>&1 | grep -v bad_alloc
sleep 1

LOG=/tmp/point_lio.log
N=$(grep -c '^Frame' "$LOG")
LAG=$(grep -c 'lag' "$LOG")
echo "== 处理 $N 帧 / 预期约 $((SECS * 10)) 帧, 丢帧标记 ${LAG} 次 =="
grep -E '^\[Init\]' "$LOG"
grep '^Frame' "$LOG" | head -4
grep '^Frame' "$LOG" | tail -4
