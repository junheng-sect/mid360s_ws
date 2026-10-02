"""标定录包的纯逻辑单测：提频/恢复决策、关包体检、开关策略、序号分配（不需要 ROS Master）。"""
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

from calib_bag_recorder import FCU_IMU_TOPIC, bag_health, fcu_rate_request  # noqa: E402
from rc_record_policy import RcRecordSession  # noqa: E402
from sequence_allocator import SequenceAllocator  # noqa: E402

BOOST = 400.0


# ---------------------------------------------------------------- 提频 / 恢复决策 ----
def test_boost_requested_when_recording_and_connected():
    assert fcu_rate_request(True, False, True, False, BOOST) == BOOST


def test_nothing_to_do_once_boost_applied():
    assert fcu_rate_request(True, True, True, False, BOOST) is None


def test_boost_reapplied_after_reconnect():
    assert fcu_rate_request(True, True, True, True, BOOST) == BOOST


def test_restore_default_after_recording():
    assert fcu_rate_request(False, True, True, False, BOOST) == 0.0


def test_idle_without_boost_does_nothing():
    assert fcu_rate_request(False, False, True, False, BOOST) is None


def test_no_call_while_fcu_disconnected():
    assert fcu_rate_request(True, False, False, False, BOOST) is None
    assert fcu_rate_request(False, True, False, False, BOOST) is None


def test_boost_disabled_never_calls():
    for want in (True, False):
        for applied in (True, False):
            assert fcu_rate_request(want, applied, True, True, 0.0) is None


# ---------------------------------------------------------------------- 关包体检 ----
def _counts(fcu_imu=1900):
    return {'/livox/lidar': 100, '/livox/imu': 2000, FCU_IMU_TOPIC: fcu_imu,
            '/mavros/timesync_status': 100}


def test_health_ok_and_rates():
    rates, problems = bag_health(_counts(), 10.0, 150.0)
    assert problems == []
    assert abs(rates[FCU_IMU_TOPIC] - 190.0) < 1e-9
    assert abs(rates['/livox/lidar'] - 10.0) < 1e-9


def test_health_reports_missing_topic():
    counts = _counts()
    counts['/livox/lidar'] = 0
    _, problems = bag_health(counts, 10.0, 150.0)
    assert len(problems) == 1 and '/livox/lidar' in problems[0]


def test_health_reports_low_fcu_imu_rate():
    _, problems = bag_health(_counts(fcu_imu=500), 10.0, 150.0)
    assert len(problems) == 1 and FCU_IMU_TOPIC in problems[0] and '50.0 Hz' in problems[0]


def test_health_rate_check_disabled():
    _, problems = bag_health(_counts(fcu_imu=500), 10.0, 0.0)
    assert problems == []


def test_health_zero_duration_does_not_crash():
    rates, _ = bag_health(_counts(), 0.0, 150.0)
    assert all(rate == 0.0 for rate in rates.values())


# ---------------------------------------------------------------------- 开关策略 ----
def _feed(session, t, active, n):
    actions = []
    for i in range(n):
        actions.append(session.on_rc(t + 0.05 * i, True, active))
    return actions


def test_no_bag_if_switch_already_on_at_startup():
    session = RcRecordSession(confirm_samples=3, release_tail_s=1.0)
    assert RcRecordSession.OPEN not in _feed(session, 0.0, True, 10)


def test_open_after_debounce_then_close_after_tail():
    session = RcRecordSession(confirm_samples=3, release_tail_s=1.0)
    assert RcRecordSession.OPEN not in _feed(session, 0.0, False, 3)
    actions = _feed(session, 1.0, True, 3)
    assert actions == [None, None, RcRecordSession.OPEN]
    session.bag_opened()
    assert _feed(session, 2.0, False, 3)[-1] == RcRecordSession.PENDING_CLOSE
    assert session.tick(2.5) is None
    assert session.tick(3.2) == RcRecordSession.CLOSE


# ---------------------------------------------------------------------- 序号分配 ----
def test_sequence_allocator_increments():
    directory = tempfile.mkdtemp()
    try:
        allocator = SequenceAllocator(directory, 'calib')
        first = allocator.allocate()
        open(first, 'w').close()
        second = allocator.allocate()
        assert os.path.basename(first) == 'calib_000001.bag'
        assert os.path.basename(second) == 'calib_000002.bag'
    finally:
        shutil.rmtree(directory)
