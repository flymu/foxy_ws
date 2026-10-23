#!/usr/bin/env python3
"""探一下 con_notify：看固件加密版本 data2（1=明文, 2=静态密钥可用, 3=要云端密钥）"""
import base64
import json
import urllib.request

url = "http://192.168.123.161:9991/con_notify"
req = urllib.request.Request(url, data=b"", method="POST")
resp = urllib.request.urlopen(req, timeout=5)
raw = resp.read().decode()
print("HTTP", resp.status, "raw_len", len(raw))
decoded = json.loads(base64.b64decode(raw).decode("utf-8"))
data1 = decoded.get("data1", "")
data2 = decoded.get("data2")
print("data2 =", data2, "| data1 len =", len(data1))
if data2 in (1, None):
    print("data1 前 80 字符:", data1[:80])
