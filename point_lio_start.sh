#!/bin/bash
# point_lio_start.sh — 启动/重启 Point-LIO ROS2 节点（Orin 上，后台）
# 日志: /tmp/point_lio.log   停止: pkill -INT -f point_lio_node （触发退出保存地图）
pkill -f point_lio_node 2>/dev/null
sleep 0.5

source /opt/ros/foxy/setup.bash
source ~/livox_ws/install/setup.bash
source ~/point_lio_ws/install/setup.bash

nohup ~/point_lio_ws/build/point_lio_ros2/point_lio_node --ros-args -p config_file:=/home/unitree/Point-LIO/config/mid360.yaml >/tmp/point_lio.log 2>&1 &
echo "point_lio pid: $!"
