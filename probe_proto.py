#!/usr/bin/env python3
"""探测 .161 上 8551/9991 两个端口说的什么协议"""
import socket

H = "192.168.123.161"


def probe(port, payload, name):
    try:
        s = socket.socket()
        s.settimeout(2)
        s.connect((H, port))
        s.sendall(payload)
        data = s.recv(4096)
        print(f"--- {name} port {port} ---")
        print(repr(data[:500]))
        s.close()
    except Exception as e:
        print(f"--- {name} port {port}: {e} ---")


probe(8551, b"OPTIONS * RTSP/1.0\r\nCSeq: 1\r\n\r\n", "RTSP-OPTIONS")
probe(8551, b"GET / HTTP/1.0\r\nHost: dog\r\n\r\n", "HTTP-GET")
probe(9991, b"GET / HTTP/1.0\r\nHost: dog\r\n\r\n", "HTTP-GET")
probe(9991, b"\x00\x01\x00\x02", "RAW-HEX")
