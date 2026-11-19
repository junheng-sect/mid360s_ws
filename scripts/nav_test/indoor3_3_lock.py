"""Evidence collected by the OV9281 lock gate during one indoor3_3 search."""
from collections import Counter


MAX_SPEED_MPS = .05
STABLE_S = .5
WINDOW_S = .5
SEARCH_S = 5.
MAX_RMS_PX = 2.
ODOM_GAP_S = .20
MIN_SAMPLES = 3

LABELS = {
    'wrong_frame': '检测坐标系不符', 'wrong_family': '标签家族不符',
    'zero_stamp': '无源时间戳', 'stale_stamp': '检测消息过期', 'future_stamp': '时间戳超前',
    'before_search': '源帧早于本轮', 'before_hover': '早于最近一次超速重置',
    'expired': '已超出最近 0.5 秒', 'not_evaluated': '未进入判定窗口',
    'no_tag': '未见 Tag 1', 'no_pose': '检测到 ID 但无位姿',
    'duplicate_pose': '同帧有多个 ID 1 位姿', 'reprojection': '重投影误差无效或超过 2 px',
    'odom_sync': '里程计未能按帧时间插值', 'invalid_pose': '位姿或变换数值无效',
    'behind_camera': 'Tag 深度非正', 'back_facing': '纸面未朝向相机',
    'normal_tilt': '纸面法线偏离水平超过 15°', 'tag_up': 'Tag 上方向偏离竖直超过 20°',
    'position_spread': '位置离中位数超过 0.10 m', 'normal_spread': '法线离平均方向超过 10°',
    'too_few': '同窗有效帧不足 3 个', 'candidate': '通过单帧筛选', 'locked': '已锁定',
}


class LockRejection(ValueError):
    def __init__(self, code, value=None):
        super().__init__(LABELS[code])
        self.code = code
        self.value = value


class LockTrace:
    def __init__(self, start_ns):
        self.start_ns = start_ns
        self.frames = {}
        self.dropped = Counter()
        self.duplicates = 0
        self.windows = []
        self.max_stable = 0.
        self.reset_count = 0
        self.was_fast = False

    def receive(self, msg, problem):
        stamp = msg.header.stamp.to_nsec()
        if problem or stamp < self.start_ns:
            self.dropped[problem or 'before_search'] += 1
            return
        ids = list(msg.ids)
        pose_count = sum(t.id == 1 for t in msg.tags)
        if stamp in self.frames:
            self.duplicates += 1
            row = self.frames[stamp]
            row['ids'] = sorted(set(row['ids']).union(ids))
            row['seen'] = 1 in row['ids']
            row['poses'] = max(row['poses'], pose_count)
            return
        self.frames[stamp] = dict(stamp_ns=stamp, ids=ids, seen=1 in ids, poses=pose_count,
                                  evaluated=False, result='not_evaluated')

    def hover(self, fast, stable_s):
        self.reset_count += int(fast and not self.was_fast)
        self.was_fast = fast
        self.max_stable = max(self.max_stable, stable_s)

    def finish(self, end_ns, outcome, stop_reason):
        frames = list(self.frames.values())
        evaluated = [f for f in frames if f['evaluated']]
        frame_counts = Counter(f['result'] for f in frames)
        window_counts = Counter(w['result'] for w in self.windows)
        peak = max((len(w['stamps_ns']) for w in self.windows), default=0)
        seen = sum(f['seen'] for f in frames)
        solved = sum(f['poses'] > 0 for f in frames)
        if outcome == 'locked':
            reason = '已锁定 Tag 1'
        elif outcome == 'stream_fault':
            reason = '搜索结束时没有新鲜的 OV9281 检测帧'
            if self.dropped:
                reason += '（' + '、'.join(LABELS[k] for k in self.dropped) + '）'
        elif outcome != 'timeout':
            reason = stop_reason
        elif not self.windows:
            reason = '连续低速最长 %.2f s，未满 %.2f s' % (self.max_stable, STABLE_S)
        elif not seen:
            reason = '检测帧中未见 Tag 1'
        elif not solved:
            reason = '看到了 Tag 1，检测器未给出其位姿'
        else:
            reason = '单窗最多 %d 个有效位姿（需要 %d 个）' % (peak, MIN_SAMPLES)
            failures = [k for k in window_counts if k not in ('locked', 'too_few')]
            failures += [k for k in frame_counts if k != 'candidate']
            if failures:
                reason += '；' + '、'.join(LABELS[k] for k in failures)
        return dict(outcome=outcome, reason=reason, start_ns=self.start_ns, end_ns=end_ns,
                    survey_frames=len(frames), tag1_frames=seen, pose_frames=solved,
                    evaluated_frames=len(evaluated), candidate_frames=frame_counts['candidate'],
                    max_window_frames=peak, max_stable_s=round(self.max_stable, 4),
                    speed_resets=self.reset_count, dropped_messages=dict(self.dropped),
                    duplicate_messages=self.duplicates, frame_counts=dict(frame_counts),
                    window_counts=dict(window_counts), frames=frames, windows=self.windows)
