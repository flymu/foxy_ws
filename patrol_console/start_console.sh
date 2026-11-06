#!/bin/bash
# 宿主机一键启动容器里的网页控制台。UI: http://<本机 IP>:8000
#
# 两个坑都按记忆里的规矩来：
#  - 必须 docker exec -d + setsid + 完整重定向：普通 exec 的后台树会被会话 SIGTERM
#    连坐杀（E2E 回归踩过）；不重定向 fd 的话管道缓冲写满会卡死进程（CLEAN 连坐杀
#    live 栈事故）。
#  - 日志落 patrol_console/data/console.log（容器与宿主机共享 /home/mu/foxy_ws 挂载）。
set -e
DATA=/home/mu/foxy_ws/patrol_console/data
mkdir -p "$DATA"
LOG="$DATA/console.log"
# 容器里没有 ss，用 curl 探活（容器里 curl 可用，host 网络端口直达）
alive() { docker exec foxy-live curl -s -m 2 http://127.0.0.1:8000/api/status >/dev/null 2>&1; }
if alive; then
  echo "控制台已经在跑：http://$(hostname -I | awk '{print $1}'):8000  (日志 $LOG)"
  exit 0
fi
docker exec -d foxy-live bash -c \
  "setsid bash /home/mu/foxy_ws/patrol_console/console_in_container.sh >> '$LOG' 2>&1 < /dev/null"
sleep 2
alive \
  && echo "控制台已启动: http://$(hostname -I | awk '{print $1}'):8000  (日志 $LOG)" \
  || { echo "启动失败，看日志: $LOG"; tail -20 "$LOG"; exit 1; }
