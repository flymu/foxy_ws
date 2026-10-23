#!/usr/bin/env python3
"""videohub 深挖：照片是否实时 + 枚举更多 api id"""
import hashlib
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
        for api_id in range(1002, 1008):
            self._RegistApi(api_id, 0)

    def GetImageSample(self):
        return self._CallBinary(VIDEO_API_ID_GETIMAGESAMPLE, [])

    def TryApi(self, api_id):
        return self._CallBinary(api_id, [])


def md5(b):
    return hashlib.md5(b).hexdigest()[:12]


def main():
    ChannelFactoryInitialize(0, "eth1")
    cli = VideoHubClient()
    cli.Init()

    code, d1 = cli.GetImageSample()
    print("photo1: code=%s len=%s md5=%s" % (code, len(d1 or b""), md5(bytes(d1 or b""))))
    time.sleep(6)
    code, d2 = cli.GetImageSample()
    print("photo2: code=%s len=%s md5=%s" % (code, len(d2 or b""), md5(bytes(d2 or b""))))
    print("两帧相同？" , bytes(d1 or b"") == bytes(d2 or b""))

    with open("/tmp/go2_photo2.jpg", "wb") as f:
        f.write(bytes(d2 or b""))

    for api_id in range(1002, 1008):
        t0 = time.time()
        try:
            code, data = cli.TryApi(api_id)
            dl = len(data) if data else 0
            print("api %d: code=%s len=%d 耗时%.2f" % (api_id, code, dl, time.time() - t0))
        except Exception as e:
            print("api %d: 异常 %s" % (api_id, e))


main()
