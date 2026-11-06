#!/bin/bash
# 给控制台装 Python 依赖（fastapi/uvicorn），装到容器可见的 pydeps/ 平铺目录里。
# 在宿主机上直接跑本脚本；容器是 UID 1000 非 root，没有 ensurepip 建不了真 venv，
# 所以用 pip --target（python3 -m venv 的替代）。
set -e
docker exec foxy-live bash -c '
set -e
source /opt/ros/foxy/setup.bash
cd /home/mu/foxy_ws/patrol_console
rm -rf pydeps
pip3 install --quiet --target pydeps --upgrade fastapi "uvicorn[standard]"
PYTHONPATH=/home/mu/foxy_ws/patrol_console/pydeps python3 -c "import fastapi, uvicorn, websockets; print(\"pydeps OK: fastapi\", fastapi.__version__, \"+ uvicorn\", uvicorn.__version__)"
'
