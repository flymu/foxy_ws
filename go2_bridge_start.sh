#!/bin/bash
# 启动/重启 go2_bridge（Orin 上）— 后台运行, 日志在 /tmp/go2_bridge.log
pkill -f go2_bridge.py 2>/dev/null
sleep 0.3

source /opt/ros/foxy/setup.bash
# unitree_api/unitree_go 等消息包（/api/sport/request 需要）
source /home/unitree/unitree_ros2/install/setup.bash
# SDK2 python 包取源码树（install 里那份与本机 cyclonedds 版本不匹配）; libddsc 0.10.2 在 cyclonedds_ws
export PYTHONPATH=/home/unitree/unitree_ros2/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}
export LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

nohup python3 /home/unitree/go2_bridge.py >/tmp/go2_bridge.log 2>&1 &
echo "bridge pid: $!"
