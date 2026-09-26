#!/usr/bin/env python3
"""测量 RViz 主窗口的左侧停靠面板布局。

输出一行，空格分隔：

    <面板宽> <窗高> <分隔条 x> <分隔条 y>

后两个坐标相对主窗口左上角；定位不到分隔条时输出 -1 -1（面板宽与窗高仍会输出）。
测量失败时向 stderr 说明原因并以非 0 退出。

分隔条位置靠**相邻窗口边界**求出：Qt 为每个停靠面板建了原生 X 子窗口（都从 x=0 开始、
窄于主窗口），面板区右边界右侧是分隔条手柄，再往右才是中央 3D 区域。因此
「面板右边界」与「中央区域左边界」之间那道窄缝就是手柄，取其中点即可。

这里刻意**不**做两件事：不假设 3D 区域一定落在窗口右半边（面板收窄后这个假设不成立），
也不在没有实测边界时猜一个坐标——定位不到就明确报 -1，由调用方决定是否放弃。
"""

import re
import subprocess
import sys
from dataclasses import dataclass

# 手柄宽度上限：Qt 的分隔条一般 4~6px，这里放宽到 16px 仍算「相邻」，
# 再宽就可能是别的东西（例如被隐藏的面板占位），宁可判为定位失败。
MAX_HANDLE_GAP = 16
MIN_HANDLE_GAP = 2
# 手柄与面板至少要有这么高的重叠才算同一个分隔条，避免点到别处的缝。
MIN_OVERLAP = 32


@dataclass(frozen=True)
class Child:
    wid: str
    width: int
    height: int
    x: int
    y: int


def xwininfo(*args):
    return subprocess.check_output(
        ["xwininfo", *args], text=True, stderr=subprocess.DEVNULL, timeout=5
    )


def parse_children(tree):
    """只取直接子窗口：xwininfo -tree 里直接子窗口缩进 5 个空格，孙窗口更深，
    而孙窗口的坐标是相对父窗口的，混进来会算错。"""
    pattern = r"^ {5}(0x[0-9a-fA-F]+).*?\s(\d+)x(\d+)([+-]\d+)([+-]\d+)\s"
    return [
        Child(m[0], *(int(v) for v in m[1:]))
        for m in re.findall(pattern, tree, re.MULTILINE)
    ]


def measure(width, height, children):
    """返回 (面板宽, 窗高, 分隔条 x, 分隔条 y)。"""
    # 左侧停靠面板：从 x=0 开始、窄于主窗口（排除菜单栏/工具栏/状态栏这类整宽子窗口），
    # 且竖直方向落在主窗口内。同一停靠区里的面板等宽，取最宽者。
    left = [
        c
        for c in children
        if c.x == 0 and 0 < c.width < width and c.height > 0 and c.y >= 0
        and c.y + c.height <= height
    ]
    if not left:
        raise ValueError("没有可见的左侧停靠面板（面板可能被隐藏了）")
    panel = max(c.width for c in left)

    candidates = []
    for dock in left:
        if dock.width != panel:
            continue
        for other in children:
            gap = other.x - panel
            if not MIN_HANDLE_GAP <= gap <= MAX_HANDLE_GAP:
                continue
            top = max(dock.y, other.y)
            bottom = min(dock.y + dock.height, other.y + other.height)
            if bottom - top < MIN_OVERLAP:
                continue
            # 缝隙最窄、重叠最高的那个最可信
            candidates.append((gap, -(bottom - top), panel + gap // 2, (top + bottom) // 2))

    if not candidates:
        return panel, height, -1, -1
    candidates.sort()
    return panel, height, candidates[0][2], candidates[0][3]


def probe(wid):
    info = xwininfo("-id", wid)
    if "Map State: IsViewable" not in info:
        raise ValueError("RViz 主窗口不可见或已关闭")
    width = int(re.search(r"^\s+Width:\s+(\d+)", info, re.MULTILINE)[1])
    height = int(re.search(r"^\s+Height:\s+(\d+)", info, re.MULTILINE)[1])
    visible = []
    for child in parse_children(xwininfo("-tree", "-id", wid)):
        if child.width >= width or child.x < 0 or child.height <= 0:
            continue
        if "Map State: IsViewable" in xwininfo("-id", child.wid):
            visible.append(child)
    return measure(width, height, visible)


if __name__ == "__main__":
    try:
        wid = sys.argv[1]
    except IndexError:
        print("用法: rviz_window_geometry.py <RViz 主窗口 id>", file=sys.stderr)
        sys.exit(2)
    try:
        print(*probe(wid))
    except (ValueError, TypeError, subprocess.SubprocessError) as exc:
        print(f"无法测量 RViz 窗口 {wid}: {exc}", file=sys.stderr)
        sys.exit(1)
