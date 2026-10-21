#!/usr/bin/env python3
"""兼容入口；参数原样交给 rosrun drone_relocalization relocalize 对应脚本。"""
import os
import sys
entry=os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),'src/drone_relocalization/scripts/relocalize')
os.execv(sys.executable,[sys.executable,entry]+sys.argv[1:])
