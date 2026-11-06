"""合成数据发布器（无狗冒烟测试用）：向 /cloud_registered 和 /Odometry 发 10 Hz 假数据。

模拟 Point-LIO 的输出特征：点云在 camera_init 系、随时间平移（每帧长出新体素），
里程计绕圈走。真实栈未运行时用它在浏览器里也能看到点云流动。
用法（容器内，已 source ROS）：
    python3 /home/mu/foxy_ws/patrol_console/tests/synth_publisher.py [时长秒，默认一直跑]
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else None
    rclpy.init()
    node = Node('synth_pub')
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', 10)
    odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
    fields = [PointField(name=f, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, f in enumerate('xyz')]
    rng = np.random.default_rng(7)
    t0 = time.time()
    n = 1500
    # 一面沿 y 方向平移的"墙"：保证每帧都出现新体素，体素地图持续增长
    base = rng.uniform(-2, 2, (n, 1)).astype(np.float32)
    z = rng.uniform(0, 2, (n, 1)).astype(np.float32)
    noise = rng.uniform(-0.02, 0.02, (n, 3)).astype(np.float32)

    def tick():
        t = time.time() - t0
        y = (rng.uniform(0, 0.1, (n, 1)) + t * 0.3).astype(np.float32)   # 0.3 m/s 向前推
        xyz = np.concatenate([base, y, z], axis=1) + noise
        msg = PointCloud2()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_init'
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = xyz.astype(np.float32).tobytes()
        cloud_pub.publish(msg)
        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = 'camera_init'
        o.child_frame_id = 'body'
        o.pose.pose.position.x = 0.5 * math.cos(0.2 * t)
        o.pose.pose.position.y = 0.5 * math.sin(0.2 * t)
        o.pose.pose.orientation = Quaternion(z=math.sin(0.1 * t), w=math.cos(0.1 * t))
        odom_pub.publish(o)

    node.create_timer(0.1, tick)
    if duration:
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    else:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
