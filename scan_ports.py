#!/usr/bin/env python3
"""TCP 端口扫描：Go2 内部网段各主机，找 WebRTC/HTTP 服务"""
import socket
import sys
from concurrent.futures import ThreadPoolExecutor

HOSTS = ["192.168.123.161", "192.168.123.220",
         "192.168.123.13", "192.168.123.14", "192.168.123.12"]
PORTS = range(1, 65536)
TIMEOUT = 0.4
WORKERS = 800


def probe(args):
    host, port = args
    s = socket.socket()
    s.settimeout(TIMEOUT)
    try:
        s.connect((host, port))
        return (host, port)
    except Exception:
        return None
    finally:
        s.close()


def main():
    # 先试连一个随机端口筛活主机：拒绝=活着, 超时=不可达
    alive = []
    for h in HOSTS:
        s = socket.socket()
        s.settimeout(1.0)
        try:
            s.connect((h, 12345))
            print(f"{h}: 活着(12345 居然开着)", flush=True)
            alive.append(h)
        except ConnectionRefusedError:
            print(f"{h}: 活着(拒绝连接)", flush=True)
            alive.append(h)
        except socket.timeout:
            print(f"{h}: 不可达", flush=True)
        except OSError as e:
            print(f"{h}: 活着? {e}", flush=True)
            alive.append(h)
        finally:
            s.close()

    tasks = [(h, p) for h in alive for p in PORTS]
    print(f"扫描 {alive} 全端口...", flush=True)
    with ThreadPoolExecutor(WORKERS) as ex:
        for r in ex.map(probe, tasks):
            if r:
                print(f"OPEN {r[0]}:{r[1]}", flush=True)
    print("扫描完成", flush=True)


if __name__ == "__main__":
    main()
