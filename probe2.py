#!/usr/bin/env python3
"""二轮侦察：9991 banner 抓取 + RTSP 路径再枚举"""
import socket

H = "192.168.123.161"


def banner(port, name):
    s = socket.socket()
    s.settimeout(2)
    try:
        s.connect((H, port))
        try:
            data = s.recv(512)  # 先听：服务器是否主动发 banner
            print(f"--- {name} 先收: {data[:200]!r}")
        except socket.timeout:
            print(f"--- {name} 服务器不主动说话")
        for probe, tag in [
            (b"SSH-2.0-probe\r\n", "SSH探针"),
            (b"GET / HTTP/1.0\r\n\r\n", "HTTP"),
            (b"\x00\x00\x00\x08hello", "二进制"),
        ]:
            try:
                s.sendall(probe)
                data = s.recv(512)
                print(f"--- {name} {tag} 回应: {data[:200]!r}")
                if not data:
                    print(f"--- {name} {tag} 连接被关闭")
                    break
            except Exception as e:
                print(f"--- {name} {tag}: {e}")
                break
    except Exception as e:
        print(f"--- {name}: {e}")
    finally:
        s.close()


def describe(path):
    s = socket.socket()
    s.settimeout(2.5)
    try:
        s.connect((H, 8551))
        req = (f"DESCRIBE rtsp://{H}:8551/{path} RTSP/1.0\r\nCSeq: 9\r\n"
               f"Accept: application/sdp\r\n\r\n").encode()
        s.sendall(req)
        data = b""
        while len(data) < 3000:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            data += chunk
        first = data.split(b"\r\n", 1)[0].decode(errors="replace")
        if "200" not in first:
            return
        print(f"*** /{path}: {first}")
        body = data.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in data else b""
        print(body.decode(errors="replace")[:1000])
    except Exception as e:
        print(f"=== /{path}: {e}")
    finally:
        s.close()


banner(9991, "9991")

for p in ["", "test", "stream", "streams", "ch0", "fronts", "front_stream",
          "front-video", "frontVideo", "video1", "go2_camera", "unitree",
          "robot", "camera0", "camera1", "h264", "front.h265", "front1",
          "cam_front", "f", "camera/front", "streams/front", "media/front"]:
    describe(p)
print("done")
