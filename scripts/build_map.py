#!/usr/bin/env python3
"""兼容入口：建图已挪到功能包 src/drone_mapping（rosrun drone_mapping build_map），这里把命令行原样转过去，
用法见 src/drone_mapping/README.md。其他脚本要用 read_pcd / floor_fit 等函数，改成 from drone_mapping import mapio。"""
import os
import sys

ENTRY = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'src/drone_mapping/scripts/build_map')

if __name__ == '__main__':
    os.execv(sys.executable, [sys.executable, ENTRY] + sys.argv[1:])
