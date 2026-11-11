#!/usr/bin/env python3
"""test8：test7 的同一条全局路线，但每一段都先把机头转到前进方向再飞（前进 → 转 → 前进 → 转）。

和 test7 的区别只在航向：
  test7 每段把目标点第 4 个数（航向）原样发下去，执行器在平移期间锁住"进入该目标时的航向"，
        到点后才原地转到目标航向 —— 所以会出现侧飞。
  test8 忽略第 4 个数，把每个拐点拆成两个目标：
    1) 前进：下一拐点的位置 + "这一段的连线方向"作为航向。执行器在平移期间保持该航向，
       于是机头与前进方向一致（不会再侧飞）。
    2) 原地转向：与刚完成的那一段终点完全相同的位置 + 下一段方向。任务层只放行
       "重复同一终点"（位置 1 μm 内、实际误差 ≤0.15 m、速度 ≤0.1 m/s）的原地转向，
       执行器按导航配置的 yaw_rate_deg 逐 tick 步进（test8 默认用 outdoor_nav_yaw45.yaml：
       45°/s，转 90° 约 2 s）。
  于是实际节奏是：前进 → 原地转 → 前进 → 原地转 → …（最后一点不再转）。

默认路线与 test7 相同（drone_map 绝对坐标，只用位置；括号里是 test8 自动算出的航向）：
  (0,0,1) [起点] → (2,0,1) 机头 0° → (2,-2,1) 机头 -90° → (0,-2,1) 机头 180° → (0,0,1) 机头 +90°

注意：
  - 第一段的航向改不了（任务层不接受首目标的原地转向），所以起飞后机头要大致对着首段方向
    （默认路线是 map +x，即建图时的机头方向）；偏一点只会让第一段轻微侧飞。
  - 航点帧 drone_map（z=0 = 飞机着地时的机体高度），与 test7 相同；起飞前要升到 0.70 m 以上。
  - 原地转向在隔离测试里验证过（test5 的 90° 原地转向、漂移 7.7 mm），但还没在实飞里用过；
    test8 会是第一次。转向期间位置设定值压在原点，实际漂移要看实飞数据。

用法：
  nav_turn.py [--point x,y,z,yaw_deg ...] [--hold 0] [--delay 3]
              [--max-start-offset 2] [--check-segments] [--preview] [--out DIR]
"""
import math

import numpy as np

from nav_forward import say  # noqa: E402  同目录
from nav_global import GlobalRunner, build_parser, parse_args, serve  # noqa: E402
from nav_square import Stopped  # noqa: E402


class TurnRunner(GlobalRunner):
    test_name = 'test8'
    route_name = '全局航点 + 段前转向（机头朝前进方向）'
    record_prefix = 'test8'

    def make_route_map(self):
        """把航点展开成"前进 / 原地转向"两类目标（都是在 drone_map 下的绝对坐标）。"""
        pos, yaw, names = super().make_route_map()      # 复用：打印位置、首点距离、边界提示
        pos = np.asarray(pos, float)
        rate = float(self.cfg.get('yaw_rate_deg', 10.0))
        say('原地转向速率 yaw_rate_deg=%.0f°/s（导航配置），转 90° 约 %.1f s' % (rate, 90.0 / rate))
        if len(pos) < 2:
            return pos, yaw, names
        goals, yaws, labels, kinds = [], [], [], []
        for i in range(len(pos) - 1):
            d = pos[i + 1][:2] - pos[i][:2]
            yaw_leg = math.atan2(d[1], d[0])
            goals.append(pos[i + 1])
            yaws.append(yaw_leg)
            labels.append('前进 #%d→#%d，机头 %.0f°' % (i + 1, i + 2, math.degrees(yaw_leg)))
            kinds.append('move')
            if i >= len(pos) - 2:
                continue
            d2 = pos[i + 2][:2] - pos[i + 1][:2]
            yaw_next = math.atan2(d2[1], d2[0])
            delta = math.atan2(math.sin(yaw_next - yaw_leg), math.cos(yaw_next - yaw_leg))
            if abs(delta) > 1e-9:
                goals.append(pos[i + 1])                  # 同一位置：任务层的"重复同一终点"
                yaws.append(yaw_next)
                labels.append('原地转向到 %.0f°（%+.0f°）' % (math.degrees(yaw_next), math.degrees(delta)))
                kinds.append('turn')
        self.kinds = kinds
        return np.array(goals), np.array(yaws), tuple(labels)

    def skip_leg(self, i, goal, yaw):
        # 原地转向就是"位置不动、只换航向"，不能像 test7 那样按"已在点上"跳过
        return False

    def hold_for(self, i):
        # 转向段不额外保持（转完直接飞），前进段按 --hold（默认 0）
        return 0.0 if self.kinds[i] == 'turn' else self.hold_seconds


def main():
    ap = build_parser(description=__doc__, hold_default=0.0, owner='nav_test8.sh')
    return serve('nav_turn', parse_args(ap), TurnRunner)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        say('自动发目标已停；定位链路保留，请人工切回定点')
        raise SystemExit(130)
    except Stopped as error:
        say(str(error))
        raise SystemExit(1)
