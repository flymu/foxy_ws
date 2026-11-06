#!/usr/bin/env python3
# motion_test.py — Go2 移动控制测试（工作站 → /go2/cmd_vel → Orin 桥 → 狗）
# 序列: 前进0.15 → 暂停(看门狗) → 原地左转0.3 → 暂停 → 右平移0.1 → 暂停 → 显式stop
# 全程监听 /go2/status (1Hz: mode/moving/结果码)，输出与动作交错对照
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MotionTest(Node):
    def __init__(self):
        super().__init__('motion_test')
        self.pub_vel = self.create_publisher(Twist, '/go2/cmd_vel', 10)
        self.pub_cmd = self.create_publisher(String, '/go2/cmd', 10)
        self.create_subscription(String, '/go2/status', self.on_status, 10)

    def on_status(self, m):
        t = time.time() - self.t0
        print(f"    [{t:5.1f}s] status: {m.data}", flush=True)


def vel(n, vx=0.0, vy=0.0, vz=0.0, dur=3.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, vz
    end = time.time() + dur
    while time.time() < end:
        n.pub_vel.publish(t)
        time.sleep(0.1)  # 10Hz > 看门狗 0.5s


rclpy.init()
n = MotionTest()
n.t0 = time.time()
print('等待 DDS 发现(8s)...', flush=True)
time.sleep(8)  # pub 与桥的 sub 建立匹配
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

seq = [('前进 0.15 m/s x4s', dict(vx=0.15), 4.0),
       ('暂停 2s(看门狗应停车)', dict(), 2.0),
       ('原地左转 0.3 rad/s x3s', dict(vz=0.3), 3.0),
       ('暂停 2s', dict(), 2.0),
       ('右平移 0.1 m/s x3s', dict(vy=-0.1), 3.0),
       ('暂停 2s', dict(), 2.0),
       ('显式 stop 命令', None, 2.0)]
for name, v, dur in seq:
    print(f'--- {name} ---', flush=True)
    if v is None:
        s = String()
        s.data = 'stop'
        n.pub_cmd.publish(s)
        time.sleep(dur)
    else:
        vel(n, dur=dur, **v)

print('=== motion_test 完成 ===', flush=True)
time.sleep(1)
