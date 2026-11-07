#!/bin/bash
# livox_watchdog.sh — Orin 上 livox 驱动长跑发病的自愈看门狗（部署件，跑在狗上）
# 部署: scp 或in_deploy/livox_watchdog.sh go2_wifi:/home/unitree/   (push_safety.sh 已含)
# 启动: ssh go2_wifi 'setsid nohup bash /home/unitree/livox_watchdog.sh >/dev/null 2>&1 < /dev/null &'
# 停止: ssh go2_wifi 'pkill -f "[l]ivox_watchdog"'
#
# 病史（2026-11）：驱动 22-48min 长跑后发布塌方（livox 驱动线程被饿/自锁），
# relay 输入断流 → veto 失明 → 巡检失去唯一安全防线。人工重启才能恢复。
# 2026-11-06 实弹教训（两处修订，见下行 ★）：
#   * 塌方首发 ~19min（早于 22-48min 既往窗口）；首发形态=戳龄冻结 12→33s+速率 2Hz，
#     IMU 仍 200Hz——探 points2 是对的（IMU 探不出点云 SHM 写侧自锁）。
#   * ★旧版 MIN_HZ=5.0 撞上 rclpy 读 points2 的 ~5.3Hz 天花板（D2），负载高时会
#     假阳触发重启连环拳；降 3.0（真塌方实测 0.5~2.5Hz，仍全覆盖）。
#   * ★旧版重启时 rm /dev/shm/fastrtps_port* 把活着的 relay/republish 的 SHM 端口
#     一起删了——驱动重启成功、relay 变僵尸、veto 失明、/go2/odom 断供，越治越病。
#     删掉 rm；驱动死进程的陈旧端口无害（root 毒化 SHM 才是要防的，见铁律）。
#   * ★探针 timeout 无 -k，rclpy 关停可卡死整个看门狗循环 → 加 kill 兜底。
# 看门狗语义：
#   * 每 15s 探一次 /livox/points2（relay 输出，Orin 本地零拷贝小开销）的 5s 窗速率
#   * 进程在而速率 <MIN_HZ 连续 2 次 → SIGINT 驱动 + setsid 重启驱动
#     （不清 SHM！驱动重启后 relay 靠 DDS 自动重发现恢复订阅）
#   * 驱动进程不在 → 只告警不拉起（尊重 stop_livox.sh 人工停；看门狗启动时驱动
#     就不在则直接退出——防误当"自动开机雷达"用）
#   * 重启后 grace 期不判病（驱动初始化慢；11-06 实测传感器重连最长 ~3.5min）
# 2026-11-07 实弹教训：
#   * ★GRACE_S=60 < 传感器重连最长 3.5min → 重启后还在重连的驱动被提前处决
#     （当晚 19:54 重启 #1 仅活 93s 就被 sick=2 杀掉，连环拳放大）。升 240s：
#     真再塌也最多多等 3min，而假阳处决每次白费一次传感器断电重连。
# 铁律：unitree 用户跑，绝不 sudo（root 跑 DDS 毒化 SHM 发现）。
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GOTO_NUM_THREADS=1
source /opt/ros/foxy/setup.bash
source ~/livox_ws/install/setup.bash 2>/dev/null
LOG=/tmp/livox_watchdog.log
MIN_HZ=3.0
GRACE_S=240
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
  # -k 2：SIGINT 后 rclpy 关停若卡死，2s 强杀，防看门狗循环被拖死（11-06 实弹）
  hz=$(timeout -k 2 -s INT 9 ros2 topic hz /livox/points2 --window 40 2>/dev/null \
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
    log "== 重启驱动: SIGINT → 重拉（不清SHM，见文件头）=="
    pkill -INT -f "[l]ivox_ros_driver2" 2>/dev/null
    sleep 3
    pkill -9 -f "[l]ivox_ros_driver2" 2>/dev/null
    # 不 rm /dev/shm/fastrtps_port*：11-06 实弹证明会毒死活着的 relay/republish
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
