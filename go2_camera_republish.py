#!/usr/bin/env python3
"""
go2_camera_republish.py — 工作站侧相机中继（foxy-live 容器里跑）
订阅 /go2/camera/front/compressed (JPEG 720p, Orin 发来)
→ 解码 → 降采样 640x360 → /go2/camera/front (raw Image, bgr8)
给 RViz Image 显示。降采样+限频控制 UDP 分片带宽。
"""
import cv2  # noqa: E402  (容器里有 cv2)
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

PUB_EVERY = 2  # 约 3fps
OUT_W, OUT_H = 640, 360


class CameraRepublish(Node):
    def __init__(self):
        super().__init__("go2_camera_republish")
        qos = QoSProfile(  # 视频链路全 best_effort：丢帧无所谓，绝不被分片重传卡死
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            CompressedImage, "/go2/camera/front/compressed", self.on_frame, qos)
        self.pub = self.create_publisher(Image, "/go2/camera/front", qos)
        self.bridge = CvBridge()
        self.n = 0
        self.last_log = rclpy.clock.Clock().now()
        self.get_logger().info("相机中继已启动（compressed → raw 640x360）")

    def on_frame(self, msg):
        self.n += 1
        if self.n % PUB_EVERY != 0:
            return
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn("解码失败: %s" % e)
            return
        if frame is None:
            return
        small = cv2.resize(frame, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)
        out = self.bridge.cv2_to_imgmsg(small, "bgr8")
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "go2_camera"
        self.pub.publish(out)
        now = rclpy.clock.Clock().now()
        if (now - self.last_log).nanoseconds > 10e9:
            self.get_logger().info("中继中（%d 帧）" % self.n)
            self.last_log = now


def main():
    rclpy.init()
    node = CameraRepublish()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
