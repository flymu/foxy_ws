#!/usr/bin/env python3
"""RTSP 拉流测试：cv2 从 rtsp://192.168.123.161:8551/video 读帧，测 fps + 存一帧"""
import os
import time

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

import cv2  # noqa: E402

URL = "rtsp://192.168.123.161:8551/video"

cap = cv2.VideoCapture(URL)
print("opened:", cap.isOpened())
t0 = time.time()
n = 0
last = None
first_dt = None
while n < 60:
    ok, fr = cap.read()
    if ok:
        n += 1
        if first_dt is None:
            first_dt = time.time() - t0
            print("首帧 %.1fs 后到达, 形状 %s" % (first_dt, fr.shape))
        last = fr
    if time.time() - t0 > 25:
        break
dt = time.time() - t0
print("共 %d 帧 / %.1fs = %.1f fps" % (n, dt, n / max(dt, 0.1)))
if last is not None:
    cv2.imwrite("/tmp/go2_frame.jpg", last)
    print("已存 /tmp/go2_frame.jpg")
cap.release()
