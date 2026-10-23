#!/usr/bin/env python3
# viz_helper.py — RViz 可视化辅助（foxy-live 容器内运行，纯 stdlib + rclpy）
#   /global_map : 降采样全局地图 PointCloud2 (0.5Hz, reliable)
#   /loc/path   : /loc/pose 累积历史轨迹 Path (2Hz)（RViz 重启不丢）
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField

BLOB = '/home/mu/maps/global_map_viz.f32'
MAX_PATH = 4000


class VizHelper(Node):
    def __init__(self):
        super().__init__('viz_helper')

        # ---------- 全局地图 ----------
        with open(BLOB, 'rb') as f:
            raw = f.read()
        n = len(raw) // 16  # 4 float32/点
        self.map_msg = PointCloud2()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.height = 1
        self.map_msg.width = n
        self.map_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.map_msg.point_step = 16
        self.map_msg.row_step = 16 * n
        self.map_msg.data = raw  # blob 本身就是交错布局
        self.map_msg.is_dense = True
        self.pub_map = self.create_publisher(PointCloud2, '/global_map', 5)
        self.t_map = self.create_timer(2.0, self.pub_map_now)  # 0.5Hz 重发, 后加入的订阅者也能拿到
        self.pub_map_now()
        self.get_logger().info(f'global_map: {n} 点, 0.5Hz 发布')

        # ---------- 历史轨迹 ----------
        self.poses = []
        self.last_xy = None
        self.pub_path = self.create_publisher(Path, '/loc/path', 5)
        self.create_subscription(Odometry, '/loc/pose', self.on_loc, 20)
        self.t_path = self.create_timer(0.5, self.pub_path_now)

    def pub_map_now(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self.map_msg)

    def on_loc(self, m):
        p = m.pose.pose.position
        if self.last_xy is not None:
            if math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1]) < 0.05:
                return  # 站着不动不记录
        self.last_xy = (p.x, p.y)
        ps = PoseStamped()
        ps.header = m.header  # 已是 map 系
        ps.pose = m.pose.pose
        self.poses.append(ps)
        if len(self.poses) > MAX_PATH:
            self.poses = self.poses[-MAX_PATH:]

    def pub_path_now(self):
        if not self.poses:
            return
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self.poses
        self.pub_path.publish(msg)
        self.get_logger().info(f'loc/path: {len(self.poses)} 点', throttle_duration_sec=30.0)


rclpy.init()
rclpy.spin(VizHelper())
