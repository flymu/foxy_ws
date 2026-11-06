#!/bin/bash
# 控制台本体：在 foxy-live 容器里跑（和 Point-LIO 同容器，订阅点云零额外网络流量）。
# 本脚本在容器里执行；宿主机入口是 start_console.sh。
set -e
source /opt/ros/foxy/setup.bash
# Orin 六核教训（D3）：OpenBLAS 默认满核 spin-wait 会饿死别的节点。这是 WS 侧，
# 但体素处理也用 numpy，规矩统一按 1 线程。
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH=/home/mu/foxy_ws/patrol_console${PYTHONPATH:+:$PYTHONPATH}
exec python3 -m patrol_console.main --config /home/mu/foxy_ws/patrol_console/config/console.yaml
