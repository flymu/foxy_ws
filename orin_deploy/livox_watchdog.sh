#!/bin/bash
# livox_watchdog.sh — Orin 上 livox 驱动长跑发病的自愈看门狗（部署件，跑在狗上）
# 部署: scp 或in_deploy/livox_watchdog.sh go2_wifi:/home/unitree/   (push_safety.sh 已含)
# 启动: ssh go2_wifi 'setsid nohup bash /home/unitree/livox_watchdog.sh >/dev/null 2>&1 < /dev/null &'
# 停止: ssh go2_wifi 'pkill -f "[l]ivox_watchdog"'
#
# 病史（2026-11）：驱动 22-48min 长跑后发布塌方（livox 驱动线程被饿/自锁），
# relay 输入断流 → veto 失明 → 巡检失去唯一安全防线。人工重启才能恢复。
# 看门狗语义：
#   * 每 15s 探一次 /livox/points2（relay 输出，Orin 本地零拷贝小开销）的 5s 窗速率
#   * 进程在而速率 <5Hz（正常 10Hz）连续 2 次 → SIGINT 驱动 + 清 FastDDS SHM 泄漏段
#     + setsid 重启驱动（照 start_livox.sh 的启动方式）
#   * 驱动进程不在 → 只告警不拉起（尊重 stop_livox.sh 人工停；看门狗启动时驱动
#     就不在则直接退出——防误当"自动开机雷达"用）
#   * 重启后 grace 60s 不判病（驱动初始化慢）
# 铁律：unitree 用户跑，绝不 sudo（root 跑 DDS 毒化 SHM 发现）。
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GOTO_NUM_THREADS=1
source /opt/ros/foxy/setup.bash
source ~/livox_ws/install/setup.bash 2>/dev/null
LOG=/tmp/livox_watchdog.log
MIN_HZ=5.0
GRACE_S=60
CYCLE_S=15
SICK_NEEDED=2

log() { echo "$(date '+%m-%d %H:%M:%S') $*" >> $LOG; }

if ! pgrep -f "[l]ivox_ros_driver2" >/dev/null; then
  log "启动时驱动不在——退出（不自动拉起；先跑 start_livox.sh 再起看门狗）"
  echo "驱动不在，看门狗不启动（见 $LOG）"
  exit 1
fi

start_driver() {
  setsid nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
      >> /tmp/livox_driver.log 2>&1 < /dev/null &
}

sick=0
last_restart=0
log "看门狗启动 (pid $$): 驱动 pid=$(pgrep -f '[l]ivox_ros_driver2' | head -1)"
while true; do
  sleep $CYCLE_S
  # 驱动进程没了：人工停或崩溃——只告警
  if ! pgrep -f "[l]ivox_ros_driver2" >/dev/null; then
    log "!! 驱动进程消失（人工停或崩溃）——只告警，不拉起"
    sick=0
    continue
  fi
  now=$(date +%s)
  [ $((now - last_restart)) -lt $GRACE_S ] && { log "grace 期跳过"; continue; }
  # 5s 窗速率探测（ros2 topic hz 是纯 rclpy，无 numpy/OpenBLAS 烧核问题）
  hz=$(timeout -s INT 9 ros2 topic hz /livox/points2 --window 40 2>/dev/null \
       | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}')
  if [ -z "$hz" ]; then
    sick=$((sick + 1))
    log "探针无数据 (sick=$sick)"
  elif awk "BEGIN{exit !($hz < $MIN_HZ)}"; then
    sick=$((sick + 1))
    log "速率塌方 ${hz}Hz (sick=$sick)"
  else
    [ $sick -gt 0 ] && log "恢复 ${hz}Hz"
    sick=0
  fi
  if [ $sick -ge $SICK_NEEDED ]; then
    log "== 重启驱动: SIGINT → 清SHM → 重拉 =="
    pkill -INT -f "[l]ivox_ros_driver2" 2>/dev/null
    sleep 3
    pkill -9 -f "[l]ivox_ros_driver2" 2>/dev/null
    rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
    start_driver
    sleep 3
    if pgrep -f "[l]ivox_ros_driver2" >/dev/null; then
      log "重启成功 pid=$(pgrep -f '[l]ivox_ros_driver2' | head -1)"
    else
      log "!! 重启失败 — 看 /tmp/livox_driver.log"
    fi
    last_restart=$(date +%s)
    sick=0
  fi
done
