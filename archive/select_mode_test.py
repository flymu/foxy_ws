#!/usr/bin/env python3
"""select_mode_test.py — 一次性：motion_switcher SelectMode 逐名试探（跑在 Orin）
用法: python3 select_mode_test.py [名字...]   默认试 sport_mode ai_sport sport
"""
import json
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client


class MS(Client):
    def __init__(self):
        super().__init__('motion_switcher', False)

    def Init(self):
        self._SetApiVerson('1.0.0.1')
        for api in (1001, 1002, 1003):
            self._RegistApi(api, 0)

    def CheckMode(self):
        return self._Call(1001, '{}')

    def SelectMode(self, name):
        return self._Call(1002, json.dumps({'name': name}))


ChannelFactoryInitialize(0, 'eth1')
ms = MS()
ms.Init()
code, data = ms.CheckMode()
print('CheckMode: code=%s data=%s' % (code, data), flush=True)
names = [a for a in sys.argv[1:] if not a.startswith('-')] or \
    ['normal', 'sport_mode', 'ai_sport', 'sport']
for name in names:
    code, data = ms.SelectMode(name)
    print('SelectMode(%r): code=%s data=%s' % (name, code, data), flush=True)
    if code == 0:
        break  # 成功即停，别把已进的模式又切走
if code != 0 and '--release' in sys.argv:
    # 社区经验：别的客户端(App/遥控)占着 motion 栈时先 ReleaseMode 再选。
    # 注意：release 后狗可能瞬间 damp(趴下)，人要在旁边看着
    code, data = ms._Call(1003, '{}')
    print('ReleaseMode: code=%s data=%s' % (code, data), flush=True)
    code, data = ms.SelectMode('normal')
    print('SelectMode(normal) after release: code=%s data=%s' % (code, data), flush=True)
code, data = ms.CheckMode()
print('CheckMode after: code=%s data=%s' % (code, data), flush=True)
