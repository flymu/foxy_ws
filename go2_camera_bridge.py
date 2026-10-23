#!/usr/bin/env python3
"""
go2_camera_bridge.py — Go2 前摄像头 → ROS2 桥（Orin 上跑）
狗主板的 RTSP 流 rtsp://192.168.123.161:8551/video (H.264 1280x720@23fps)
  → OpenCV 拉流 → JPEG 压缩 → sensor_msgs/CompressedImage 发 /go2/camera/front/compressed
工作站侧用 image_transport republish 解压成 raw 给 RViz Image 显示。

纯 cv2 + rclpy（FastDDS），不走 SDK2，与 LIO 主栈零冲突（独立话题/帧名）。
断流自动重连；JPEG 质量 70，隔帧发布 ~11fps，WiFi 占用 ~0.5MB/s。
"""
import os
import threading
import time

import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
import rclpy
from sensor_msgs.msg import CompressedImage

RTSP_URL = "rtsp://192.168.123.161:8551/video"
PUB_TOPIC = "/go2/camera/front/compressed"
JPEG_QUALITY = 70
PUB_EVERY = 2  # 源 23fps，隔 1 帧发 1 帧 ≈ 11fps


class Go2CameraBridge(Node):
    def __init__(self):
        super().__init__("go2_camera_bridge")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 视频流丢帧可容忍；
            history=HistoryPolicy.KEEP_LAST,            # reliable 大消息走 WiFi UDP
            depth=1,                                    # 分片重传曾把订阅方卡死
        )
        self.pub = self.create_publisher(CompressedImage, PUB_TOPIC, qos)
        self.stopped = threading.Event()
        self.cap = None
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info("go2_camera_bridge 已启动（RTSP → %s）" % PUB_TOPIC)

    def open_cap(self):
        cap = cv2.VideoCapture(RTSP_URL)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        return cap

    def capture_loop(self):
        frame_idx = 0
        pub_count = 0
        fail = 0
        last_log = time.time()
        while not self.stopped.is_set():
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().info("连接 RTSP ...")
                self.cap = self.open_cap()
                if not self.cap.isOpened():
                    time.sleep(3.0)
                    continue
            ok, frame = self.cap.read()
            if not ok:
                fail += 1
                if fail >= 30:  # 连续读不到帧 → 断流重连
                    self.get_logger().warn("RTSP 断流，重连 ...")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                    fail = 0
                    time.sleep(1.0)
                continue
            fail = 0
            frame_idx += 1
            if frame_idx % PUB_EVERY != 0:
                continue

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "go2_camera"
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            self.pub.publish(msg)
            pub_count += 1

            now = time.time()
            if now - last_log > 10.0:
                self.get_logger().info(
                    "发布中：源 %d 帧，已发 %d 帧（%.1f fps）"
                    % (frame_idx, pub_count, pub_count / (now - last_log)))
                pub_count = 0
                last_log = now

    def destroy_node(self):
        self.stopped.set()
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        super().destroy_node()


def main():
    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")
    rclpy.init()
    node = Go2CameraBridge()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
