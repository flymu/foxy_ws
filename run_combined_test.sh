#!/bin/bash
# run_combined_test.sh — LIO + map_localization + scan_planner 联合回放测试
# 在 foxy-map 容器内运行: docker exec foxy-map bash /home/mu/foxy_ws/run_combined_test.sh
L=/home/mu/foxy_ws/logs
B=/home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2
P=/home/mu/foxy_ws/install_x86/scan_planner_ros2/lib/scan_planner_ros2

# 1) 清理残留（模式加括号防 pkill 自杀）
pkill -9 -f "[r]os2 bag play" 2>/dev/null
pkill -9 -f "[p]oint_lio_node" 2>/dev/null
pkill -9 -f "[m]ap_localization_node" 2>/dev/null
pkill -9 -f "[s]can_planner_node" 2>/dev/null
pkill -9 -f "[t]opic echo" 2>/dev/null
sleep 1

# 2) 清掉泄漏的 FastDDS SHM 端口段（被 SIGTERM 的节点会泄漏，毁掉后续节点 discovery）
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*

# 3) 环境
source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash

# 4) 起三个节点
$B/point_lio_node --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml -p map_save_path:=/tmp > $L/lio_re2.log 2>&1 &
LIO=$!
$B/map_localization_node --ros-args -p map_file:=/home/mu/maps/lio_1792684457_global_map.pcd > $L/loc_test2.log 2>&1 &
LOC=$!
$P/scan_planner_node --ros-args -p goal_x:=2.0 -p goal_y:=-0.5 > $L/planner_test.log 2>&1 &
PLN=$!
echo "nodes up: LIO=$LIO LOC=$LOC PLN=$PLN"

sleep 4
# 全程回显 /loc/pose（226s bag → 245s 上限）
timeout -s INT 245 ros2 topic echo /loc/pose > $L/loc_echo2.txt 2>&1 &
ECHO=$!
# 播放中探针: t+12s 查 /Odometry 与 /loc/pose 的 pub/sub 匹配数
( sleep 12; ros2 topic info /Odometry -v > $L/diag_odom_info.txt 2>&1; ros2 topic info /loc/pose -v > $L/diag_loc_info.txt 2>&1 ) &

# 5) 回放（前台，226s）
ros2 bag play /home/mu/bags/campus_map > $L/play2.log 2>&1
echo "play done rc=$?"

# 6) 收尾: INT 干净退出（避免 SHM 再泄漏）
kill -INT $PLN $LOC $LIO $ECHO 2>/dev/null
for p in $PLN $LOC $LIO $ECHO; do
  n=0
  while kill -0 $p 2>/dev/null && [ $n -lt 40 ]; do
    st=$(ps -p $p -o stat= 2>/dev/null)
    [ "${st:0:1}" = "Z" ] && break
    sleep 0.5; n=$((n+1))
  done
  kill -0 $p 2>/dev/null && kill -9 $p 2>/dev/null
done
wait 2>/dev/null

# 7) 摘要
echo "=== /Odometry 探针 ==="
grep -E "Publisher count|Subscription count" $L/diag_odom_info.txt 2>/dev/null
echo "=== /loc/pose 探针 ==="
grep -E "Publisher count|Subscription count" $L/diag_loc_info.txt 2>/dev/null
echo "=== loc 节点日志 ==="
grep -v bad_alloc $L/loc_test2.log | head -12
echo "=== planner 日志 ==="
grep -v bad_alloc $L/planner_test.log | head -25
echo "=== LIO Frame 行数 ==="
grep -c "Frame" $L/lio_re2.log
echo "=== echo /loc/pose 行数 ==="
wc -l < $L/loc_echo2.txt
echo "===combined-test2-done==="
