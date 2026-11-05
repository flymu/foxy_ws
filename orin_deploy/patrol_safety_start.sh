#!/bin/bash
# patrol_safety_start.sh — Orin 侧安全栈一键启动（D1+D2 落地 2026-11-05）
#   livox_relay    C++ 本地中继: /livox/lidar(CustomMsg) -> /livox/points2(PointCloud2)
#   obstacle_veto  独立否决层: 吃 points2(np.frombuffer 零拷贝) + /livox/imu,
#                  /cmd_vel_raw(WS规划器,经WiFi小报文) -> /go2/cmd_vel(本地给桥)
#   veto_viz       诊断可视化(可选, 传 "viz" 参数启动)
# 用法:  ssh go2_wifi 'bash /home/unitree/patrol_safety_start.sh [viz]'
# 停止:  ssh go2_wifi 'bash /home/unitree/patrol_safety_stop.sh'
# 铁律: 只许 unitree 用户跑——root 跑 DDS 会毒化 Orin 本地 SHM 发现
#      (后续本地订阅者全瞎, 救治=重启驱动+清shm+daemon stop)。
# 注意: 不能 set -u——foxy 的 setup.bash 引用未绑定变量(AMENT_TRACE_SETUP_FILES)
#      会在 source 时炸；本脚本也不用未定义变量，放弃 -u 换取可 source。
pkill -f "[l]ivox_relay" 2>/dev/null
pkill -f "[o]bstacle_veto" 2>/dev/null
pkill -f "[v]eto_viz" 2>/dev/null
sleep 1
source /opt/ros/foxy/setup.bash
source /home/unitree/livox_ws/install/setup.bash
cd /home/unitree
# Orin 上 numpy(OpenBLAS) 默认开满 6 核 spin-wait：veto+viz 各烧 1.5~2.2 核，
# 6 核被吃掉 ~3.7 核 → livox 驱动线程饿死 → 发布塌到 0.2Hz、戳龄 +1s/s 棘轮
# （11-05 实弹定位）。单线程 BLAS 对 2 万点小矩阵反而更快（省派发/spin）。
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export GOTO_NUM_THREADS=1
export OPENBLAS_MAIN_FREE=1
setsid nohup ros2 run livox_relay relay_node > /tmp/livox_relay.log 2>&1 < /dev/null &
setsid nohup python3 /home/unitree/obstacle_veto.py > /tmp/obstacle_veto.log 2>&1 < /dev/null &
if [ "${1:-}" = "viz" ]; then
  setsid nohup python3 /home/unitree/veto_viz.py > /tmp/veto_viz.log 2>&1 < /dev/null &
fi
sleep 2
echo "存活: relay=$(pgrep -c -f "[l]ivox_relay" || true) veto=$(pgrep -c -f "[o]bstacle_veto" || true) viz=$(pgrep -c -f "[v]eto_viz" || true)"
echo "--- relay 日志 ---"; tail -n 3 /tmp/livox_relay.log 2>/dev/null
echo "--- veto 日志 ---";  tail -n 3 /tmp/obstacle_veto.log 2>/dev/null
