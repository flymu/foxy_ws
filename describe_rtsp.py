#!/usr/bin/env python3
"""RTSP DESCRIBE 枚举 Go2 相机流的路径"""
import socket

H = "192.168.123.161"
P = 8551
PATHS = ["front", "front_camera", "video", "video0", "cam", "camera", "cam0",
         "0", "1", "main", "live", "go2", "go2_front", "h265", "wide",
         "rear", "mouth", "left", "right", "top", "channel0", "front_video",
         "frontcam", "rgb", "ir", "depth"]

import sys  # 命令行可覆盖候选路径
if len(sys.argv) > 1:
    PATHS = sys.argv[1:]


def describe(path):
    s = socket.socket()
    s.settimeout(2)
    try:
        s.connect((H, P))
        req = (f"DESCRIBE rtsp://{H}:{P}/{path} RTSP/1.0\r\nCSeq: 2\r\n"
               f"Accept: application/sdp\r\n\r\n").encode()
        s.sendall(req)
        data = b""
        while True:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            data += chunk
            if len(data) > 4000:
                break
        head = data.split(b"\r\n\r\n")[0].decode(errors="replace")
        body = data.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in data else b""
        print(f"=== /{path}: {head.splitlines()[0] if head else 'NO RESPONSE'}")
        if b"200" in head.encode():
            print(body.decode(errors="replace")[:800])
    except Exception as e:
        print(f"=== /{path}: {e}")
    finally:
        s.close()


for p in PATHS:
    describe(p)
print("done")
