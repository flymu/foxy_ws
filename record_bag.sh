#!/bin/bash
# record_bag.sh — Orin 上录制 Mid360 雷达(CustomMsg)+IMU 的 rosbag2
# 前提: livox 驱动运行中且 xfer_format=1（/livox/lidar 类型必须是 livox_ros_driver2/msg/CustomMsg）
# 用法:
#   record_bag.sh start <名字> [秒数]   无秒数=手动 stop
#   record_bag.sh stop
#   record_bag.sh info [名字]           默认最近一个
source /opt/ros/foxy/setup.bash
source ~/livox_ws/install/setup.bash

BAGS=~/bags
mkdir -p "$BAGS"
PIDF=/tmp/bag_rec.pid

cmd=$1
case $cmd in
start)
  name=${2:-bag_$(date +%m%d_%H%M%S)}
  secs=${3:-0}
  if [ -f "$PIDF" ] && pgrep -f "ros2 bag record" >/dev/null; then
    echo "已有录制任务在跑 (pid $(cat $PIDF))，先 record_bag.sh stop"; exit 1
  fi
  # 发现竞态: 重试几次。Foxy rosbag2 拒录"多类型"话题——工作站 RViz 等用
  # PointCloud2 订阅 /livox/lidar 也会在 DDS 层注册第二个类型描述符，必须先关
  t=""
  for i in 1 2 3 4; do t=$(timeout 10 ros2 topic type /livox/lidar 2>/dev/null); [ -n "$t" ] && break; sleep 2; done
  if [ "$t" != "livox_ros_driver2/msg/CustomMsg" ]; then
    echo "错误: /livox/lidar 类型='$t'。若同时列出 sensor_msgs/msg/PointCloud2，"
    echo "说明有 PointCloud2 订阅者还活着（多半是工作站 RViz 容器），先关掉它再录"
    echo "错误: /livox/lidar 类型='$t'，需要 CustomMsg (xfer_format=1)。修复:"
    echo "  sed -i 's/^xfer_format   = 0/xfer_format   = 1/' ~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py ~/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
    echo "  然后重启驱动"; exit 2
  fi
  for i in 1 2 3 4; do timeout 10 ros2 topic list 2>/dev/null | grep -q /livox/imu && break; sleep 2; done
  timeout 10 ros2 topic list 2>/dev/null | grep -q /livox/imu || { echo "错误: /livox/imu 不存在"; exit 2; }
  free_gb=$(df -BG /home | awk 'NR==2{print $4}' | tr -d G)
  [ "$free_gb" -lt 5 ] && echo "警告: 剩余磁盘仅 ${free_gb}G"

  dir="$BAGS/$name"
  if [ "$secs" -gt 0 ]; then
    echo ">>> 录制 ${secs}s -> $dir"
    timeout -s INT "$secs" ros2 bag record -o "$dir" /livox/lidar /livox/imu
    sleep 1
    echo ">>> 完成，bag 信息:"
    ros2 bag info "$dir"
  else
    nohup ros2 bag record -o "$dir" /livox/lidar /livox/imu >/tmp/bag_rec.log 2>&1 &
    echo $! > "$PIDF"
    echo ">>> 录制中 pid=$(cat $PIDF) -> $dir   (record_bag.sh stop 结束)"
  fi
  ;;
stop)
  if ! pgrep -f "ros2 bag record" >/dev/null; then echo "无录制任务"; rm -f "$PIDF"; exit 1; fi
  pkill -INT -f "ros2 bag record"
  for i in 1 2 3 4 5; do pgrep -f "ros2 bag record" >/dev/null || break; sleep 1; done
  pkill -TERM -f "ros2 bag record" 2>/dev/null
  rm -f "$PIDF"
  echo ">>> 已停止"
  ;;
info)
  name=${2:-$(ls -t "$BAGS" 2>/dev/null | head -1)}
  [ -z "$name" ] && { echo "没有 bag"; exit 1; }
  ros2 bag info "$BAGS/$name"
  ;;
*)
  echo "用法: $0 start <名字> [秒数] | stop | info [名字]"
  ;;
esac
