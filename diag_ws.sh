#!/bin/bash
# diag_ws.sh — WS 侧一揽子体检（foxy-live 容器内跑，宿主: docker exec foxy-live bash 本文件）
# 只读诊断：存活/日志/跨链话题可见性/小大报文收包率/bad_alloc 字符串出处
source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash 2>/dev/null

echo "=== [1] 存活节点 ==="
for p in point_lio_node map_localization_node scan_planner_node speed_adapter viz_helper; do
  n=$(pgrep -fc "[${p:0:1}]${p:1}")
  echo "$p: $n"
done

echo "=== [2] lio.log 概况 ==="
echo "bad_alloc 总数: $(grep -c bad_alloc /tmp/live_stack/lio.log 2>/dev/null)"
tail -n 6 /tmp/live_stack/lio.log 2>/dev/null

echo "=== [3] 跨链话题可见性（8s 发现窗）==="
timeout 8 ros2 topic list 2>/dev/null | sort | head -40

echo "=== [4] 小报文跨链 /go2/odom hz (6s) ==="
timeout -s INT 6 ros2 topic hz /go2/odom --window 20 2>&1 | tail -n 3

echo "=== [5] 大报文跨链 /livox/points2 hz (8s) ==="
timeout -s INT 8 ros2 topic hz /livox/points2 --window 20 2>&1 | tail -n 3

echo "=== [6] bad_alloc 字符串出处 ==="
grep -rl "bad_alloc caught" /opt/ros/foxy/lib/ 2>/dev/null | head -5
grep -rl "bad_alloc caught" /home/mu/foxy_ws/install_x86/ 2>/dev/null | head -5

echo "=== [7] /loc/health (6s) ==="
timeout -s INT 6 ros2 topic echo /loc/health 2>&1 | head -n 6
echo "=== diag done ==="
