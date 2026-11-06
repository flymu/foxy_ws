#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

rclpy.init()
n = Node("probe_cmdvel")
got = {}
n.create_subscription(Twist, "/cmd_vel_raw", lambda m: got.update(raw=got.get("raw", 0) + 1), qos_profile_sensor_data)
n.create_subscription(Twist, "/go2/cmd_vel", lambda m: got.update(dog=got.get("dog", 0) + 1), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.3)
print(f"5s 采样: {got if got else '全部无流'}")
