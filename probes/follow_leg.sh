#!/bin/bash
# follow_leg.sh — 返程跟踪（容器内跑）: 每~9s 打印 health+位姿, 共约4分钟
source /opt/ros/foxy/setup.bash
for i in $(seq 1 26); do
  printf "[%02d] " "$i"
  timeout 6 python3 /home/mu/foxy_ws/probes/health_pose_probe.py
  sleep 4
done
