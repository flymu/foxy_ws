#!/bin/bash
# go2_odom_start.sh — Orin 上四足航位推算两段接力一键启停
#   第1段 go2_odom_bridge:    rmw_cyclonedds + CYCLONEDDS_URI 钉 eth1 收主板
#                             /lf/sportmodestate → 发 /go2/odom_src
#   第2段 go2_odom_republish: 默认 FastDDS 栈把 /go2/odom_src 透传 /go2/odom
#                             给工作站（wlan0 网段）订阅
# 为什么两段、为什么钉 eth1：主板 DDS 是 Cyclone 品格且只认 123.x locator，
# FastDDS 参与者怎么配都完不成与主板的发现（publisher count 恒 0）；
# 而钉死 eth1 的 cyclone 参与者工作站又发现不了，须本机跨栈接力一次。
# 用法: ssh go2_wifi 'bash /home/unitree/go2_odom_start.sh'   （跑之前先把狗唤醒）
CDDS=/home/unitree/cyclonedds_ws/cyclonedds.xml

pkill -f "[g]o2_odom_bridge" 2>/dev/null
pkill -f "[g]o2_odom_republish" 2>/dev/null
sleep 1

source /opt/ros/foxy/setup.bash
export PYTHONPATH=/home/unitree/unitree_ros2/install/unitree_hg/lib/python3.8/site-packages:/home/unitree/unitree_ros2/install/unitree_go/lib/python3.8/site-packages:/home/unitree/unitree_ros2/install/unitree_api/lib/python3.8/site-packages:$PYTHONPATH
BASE_LD="$LD_LIBRARY_PATH:/home/unitree/unitree_ros2/install/unitree_hg/lib:/home/unitree/unitree_ros2/install/unitree_go/lib:/home/unitree/unitree_ros2/install/unitree_api/lib:/home/unitree/livox_ws/install/livox_ros_driver2/lib"

# 第1段: cyclone 钉 eth1（主板通路）
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI=file://$CDDS \
LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/rmw_cyclonedds_cpp/lib:/home/unitree/cyclonedds_ws/install/cyclonedds/lib:/home/unitree/unitree_ros2/install/cyclonedds/lib:$BASE_LD \
nohup python3 /home/unitree/go2_odom_bridge.py > /tmp/go2_odom_bridge.log 2>&1 &

# 第2段: 默认 FastDDS（工作站通路）
LD_LIBRARY_PATH=$BASE_LD \
nohup python3 /home/unitree/go2_odom_republish.py > /tmp/go2_odom_republish.log 2>&1 &

sleep 1
echo "存活: $(pgrep -fc 'go2_odom_bridge|go2_odom_republish')/2  日志: /tmp/go2_odom_bridge.log /tmp/go2_odom_republish.log"
[ -f "$CDDS" ] || echo "警告: $CDDS 不存在（unitree 自带 cyclone 配置丢了？第1段收不到主板数据）"
