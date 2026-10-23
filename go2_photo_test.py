#!/usr/bin/env python3
"""go2 拍照测试：SDK2 DDS RPC 调固件 videohub.GetImageSample (api 1001)
B2 的 FrontVideoClient 同款模式；返回的 binary 应该就是 JPEG。"""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.go2.video.video_api import (
    VIDEO_SERVICE_NAME, VIDEO_API_VERSION, VIDEO_API_ID_GETIMAGESAMPLE)


class VideoHubClient(Client):
    def __init__(self):
        super().__init__(VIDEO_SERVICE_NAME, False)

    def Init(self):
        self._SetApiVerson(VIDEO_API_VERSION)
        self._RegistApi(VIDEO_API_ID_GETIMAGESAMPLE, 0)

    def GetImageSample(self):
        return self._CallBinary(VIDEO_API_ID_GETIMAGESAMPLE, [])


def main():
    ChannelFactoryInitialize(0, "eth1")
    cli = VideoHubClient()
    cli.Init()
    print("调用 GetImageSample ...")
    t0 = time.time()
    code, data = cli.GetImageSample()
    dt = time.time() - t0
    if data is not None:
        raw = bytes(data)
        print("code: %s  耗时 %.2fs  数据 %d 字节  头: %s"
              % (code, dt, len(raw), raw[:16].hex()))
        with open("/tmp/go2_photo.jpg", "wb") as f:
            f.write(raw)
        print("已存 /tmp/go2_photo.jpg")
    else:
        print("无数据返回，code =", code, " 耗时 %.2fs" % dt)


main()
