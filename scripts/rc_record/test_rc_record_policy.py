#!/usr/bin/env python3
"""RcRecordSession 与 SequenceAllocator 的离线测试（不需要 ROS、不写 bag）。

    python3 test_rc_record_policy.py
"""
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_record_policy import RcRecordSession  # noqa: E402
from sequence_allocator import SequenceAllocator  # noqa: E402

FAILURES = []


def check(name, condition, detail=''):
    if condition:
        print('  PASS %s' % name)
    else:
        print('  FAIL %s %s' % (name, detail))
        FAILURES.append(name)


def feed(session, samples, start=0.0, dt=0.1):
    """samples: [(valid, active), ...] -> 返回 action 列表。"""
    actions = []
    now = start
    for valid, active in samples:
        action = session.on_rc(now, valid, active)
        if action:
            actions.append(action)
        now += dt
    return actions, now


def test_open_on_rising_edge():
    print('test_open_on_rising_edge')
    s = RcRecordSession(confirm_samples=3, release_tail_s=1.0)
    actions, _ = feed(s, [(True, False)] * 3)
    check('未触发时不动作', actions == [], actions)
    actions, _ = feed(s, [(True, True)] * 3)
    check('拨到触发位 -> open', actions == [RcRecordSession.OPEN], actions)
    s.bag_opened()
    check('recording() 为真', s.recording())


def test_debounce():
    print('test_debounce')
    s = RcRecordSession(confirm_samples=3, release_tail_s=1.0)
    feed(s, [(True, False)] * 3)
    actions, _ = feed(s, [(True, True)] * 2)
    check('只来 2 拍（确认 3 拍）不开包', actions == [], actions)
    actions, _ = feed(s, [(True, False)] * 2)
    check('抖回非触发也不动作', actions == [], actions)
    actions, _ = feed(s, [(True, True)] * 3)
    check('稳定 3 拍后开包', actions == [RcRecordSession.OPEN], actions)


def test_tail_close():
    print('test_tail_close')
    s = RcRecordSession(confirm_samples=2, release_tail_s=1.0)
    feed(s, [(True, False)] * 2)
    actions, now = feed(s, [(True, True)] * 2)
    check('开包', actions == [RcRecordSession.OPEN], actions)
    s.bag_opened()
    actions, now = feed(s, [(True, False)] * 2, start=now)
    check('松手 -> pending_close', actions == [RcRecordSession.PENDING_CLOSE], actions)
    check('尾巴未到不关', s.tick(now + 0.5) is None)
    check('尾巴到期 -> close', s.tick(now + 1.05) == RcRecordSession.CLOSE)


def test_retrigger_during_tail_keeps_same_bag():
    print('test_retrigger_during_tail_keeps_same_bag')
    s = RcRecordSession(confirm_samples=2, release_tail_s=1.0)
    feed(s, [(True, False)] * 2)
    actions, now = feed(s, [(True, True)] * 2)
    s.bag_opened()
    actions, now = feed(s, [(True, False)] * 2, start=now)
    check('进入尾巴', actions == [RcRecordSession.PENDING_CLOSE], actions)
    actions, now = feed(s, [(True, True)] * 2, start=now + 0.3)
    check('尾巴内重新触发不再开新包', actions == [], actions)
    check('尾巴被取消', s.close_due_at is None)
    check('到期也不关（触发位仍在）', s.tick(now + 5.0) is None)


def test_rc_loss_does_not_truncate():
    print('test_rc_loss_does_not_truncate')
    s = RcRecordSession(confirm_samples=2, release_tail_s=1.0)
    feed(s, [(True, False)] * 2)
    actions, now = feed(s, [(True, True)] * 2)
    s.bag_opened()
    actions, now = feed(s, [(False, False)] * 10, start=now)
    check('RC 丢失不产生关包动作', actions == [], actions)
    check('仍在录制', s.recording())
    actions, now = feed(s, [(True, False)] * 2, start=now)
    check('恢复后松手照常关包', actions == [RcRecordSession.PENDING_CLOSE], actions)


def test_respawn_mid_trigger_does_not_open():
    print('test_respawn_mid_trigger_does_not_open')
    s = RcRecordSession(confirm_samples=2, release_tail_s=1.0)
    actions, _ = feed(s, [(True, True)] * 5)
    check('一上来就在触发位 -> 不开包（避免半截包）', actions == [], actions)
    actions, _ = feed(s, [(True, False)] * 2)
    check('先见到非触发位', actions == [], actions)
    actions, _ = feed(s, [(True, True)] * 2)
    check('再拨下才开包', actions == [RcRecordSession.OPEN], actions)


def test_fail_is_sticky():
    print('test_fail_is_sticky')
    s = RcRecordSession(confirm_samples=2, release_tail_s=1.0)
    feed(s, [(True, False)] * 2)
    s.fail()
    actions, _ = feed(s, [(True, True)] * 4)
    check('fail 后不再开包', actions == [], actions)
    check('healthy() 为假', not s.healthy())


def test_zero_tail_closes_immediately():
    print('test_zero_tail_closes_immediately')
    s = RcRecordSession(confirm_samples=2, release_tail_s=0.0)
    feed(s, [(True, False)] * 2)
    actions, _ = feed(s, [(True, True)] * 2)
    s.bag_opened()
    actions, _ = feed(s, [(True, False)] * 2)
    check('尾巴 0 -> 直接 close', actions == [RcRecordSession.CLOSE], actions)


def test_second_bag_after_close():
    print('test_second_bag_after_close')
    s = RcRecordSession(confirm_samples=2, release_tail_s=0.0)
    feed(s, [(True, False)] * 2)
    actions, _ = feed(s, [(True, True)] * 2)
    s.bag_opened()
    actions, _ = feed(s, [(True, False)] * 2)
    s.bag_closed()
    actions, _ = feed(s, [(True, True)] * 2)
    check('关闭后再拨下 -> 新开一个包', actions == [RcRecordSession.OPEN], actions)


def test_sequence_allocator():
    print('test_sequence_allocator')
    tmp = tempfile.mkdtemp(prefix='rc_seq_test.')
    try:
        allocator = SequenceAllocator(tmp, 'testrc')
        first = allocator.allocate()
        second = allocator.allocate()
        check('序号递增', os.path.basename(first) == 'testrc_000001.bag'
              and os.path.basename(second) == 'testrc_000002.bag',
              (first, second))
        open(os.path.join(tmp, 'testrc_000010.bag'), 'w').close()
        third = allocator.allocate()
        check('跳过已存在的编号', os.path.basename(third) == 'testrc_000011.bag', third)
        open(os.path.join(tmp, 'testrc_000020.bag.active'), 'w').close()
        fourth = allocator.allocate()
        check('跳过 .bag.active', os.path.basename(fourth) == 'testrc_000021.bag', fourth)
        fresh = SequenceAllocator(tmp, 'other')
        check('不同前缀互不影响',
              os.path.basename(fresh.allocate()) == 'other_000001.bag')
        check('状态文件按前缀分开', os.path.exists(os.path.join(tmp, '.testrc_sequence'))
              and os.path.exists(os.path.join(tmp, '.other_sequence')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    for func in (test_open_on_rising_edge, test_debounce, test_tail_close,
                 test_retrigger_during_tail_keeps_same_bag,
                 test_rc_loss_does_not_truncate,
                 test_respawn_mid_trigger_does_not_open, test_fail_is_sticky,
                 test_zero_tail_closes_immediately, test_second_bag_after_close,
                 test_sequence_allocator):
        func()
    print()
    if FAILURES:
        print('FAILED: %d 项 -> %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
