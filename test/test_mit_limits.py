"""Measured limit latch and recovery policy; no hardware or ROS nodes."""

import math
from pathlib import Path

import pytest

from dual_sharpa_wave.control_model import load_limits
from dual_sharpa_wave.joint_names import joint_names
from dual_sharpa_wave.mit_limits import MitLimits, RecoveryDiverged
from dual_sharpa_wave.sharpa_sdk_hand import SharpaSdkHand
from dual_sharpa_wave.hand_interface import BackendError


@pytest.fixture
def guard():
    return MitLimits(load_limits(Path(__file__).parents[1])['right'], joint_names('right'))


@pytest.mark.parametrize('joint,bound,sign', [(17, 1, 1), (7, 0, -1)])
def test_measured_five_degree_threshold_and_latch(guard, joint, bound, sign):
    raw = [0.] * 22
    raw[joint] = guard.limits[joint][bound] + sign * math.radians(5.)
    guard.observe(raw, 0.)
    assert guard.state == 'normal'
    raw[joint] += sign * math.radians(.001)
    guard.observe(raw, .1)
    assert guard.state == 'limit_locked'
    guard.observe([0.] * 22, .2)
    assert guard.state == 'limit_locked'  # Actual returning alone never rearms control.


def test_clip_does_not_change_raw_or_hide_overshoot(guard):
    raw = [100.] * 22
    copy = raw.copy()
    command = guard.clip(raw)
    assert command == [hi for lo, hi in guard.limits]
    assert raw == copy
    assert guard.state == 'normal'  # A request alone does not lock.
    guard.observe(raw, 0)
    assert guard.state == 'limit_locked'


def test_recovery_only_toward_zero_cancel_releases(guard):
    raw = [0.] * 22
    raw[17] = .32
    raw[7] = -.02
    guard.observe(raw, 0)
    guard.begin(raw)
    assert guard.recovery_target(raw)[17] == .2618
    target = [0.] * 22
    target[17] = .2
    assert guard.recovery_target(target) == target
    target[17] = .21
    with pytest.raises(ValueError, match='monotonically'):
        guard.recovery_target(target)
    guard.cancel(raw, .1)
    assert guard.state == 'normal'
    with pytest.raises(RuntimeError):
        guard.recovery_target([0.] * 22)
    guard.begin([0.] * 22)
    guard.finish([0.] * 22)
    assert guard.state == 'normal'


@pytest.mark.parametrize('excess_deg,expected', [(0., 'normal'), (5., 'normal'),
                                               (5.001, 'limit_locked')])
def test_cancel_recovery_only_locks_on_measured_overshoot(guard, excess_deg, expected):
    current = [0.] * 22
    current[17] = guard.limits[17][1] + math.radians(excess_deg)
    guard.begin(current)
    guard.cancel(current, .1)
    assert guard.state == expected
    if expected == 'limit_locked':
        assert 'measured' in guard.reason
    else:
        assert not guard.reason


def test_adapter_cancel_read_failure_remains_sdk_fault(fake_sdk, guard):
    fake_sdk.degrees = [0.] * 22
    hand = SharpaSdkHand('LEFT-SERIAL', .3, .6, False, control_mode='mit',
                         sdk_factory=lambda: fake_sdk, clock=fake_sdk.clock,
                         joint_limits=guard.limits)
    hand.start()
    hand.begin_recovery()
    fake_sdk.fail = 'read_status'
    with pytest.raises(BackendError):
        hand.cancel_recovery('Canceled')
    assert hand.mit_limits.state == 'faulted'
    assert not hand._started


def test_recovery_worsening_requires_persistent_same_joint(guard):
    raw = [0.] * 22
    raw[17] = .32
    guard.begin(raw)
    worse = raw.copy()
    worse[17] += math.radians(.6)
    guard.observe(worse, 0.)
    guard.observe(raw, .1)  # A brief spike clears the debounce.
    guard.observe(worse, .2)
    guard.observe(worse, .39)
    with pytest.raises(RecoveryDiverged):
        guard.observe(worse, .41)


def test_progress_cannot_be_lost_after_initial_return(guard):
    raw = [0.] * 22
    raw[17] = .32
    guard.begin(raw)
    guard.observe([0.] * 22, 0.)
    guard.observe(raw, .1)
    with pytest.raises(RecoveryDiverged):
        guard.observe(raw, .31)


def test_cannot_finish_outside_measured_tolerance(guard):
    raw = [0.] * 22
    raw[7] = -math.radians(5.1)
    guard.begin(raw)
    with pytest.raises(RuntimeError):
        guard.finish(raw)


def test_adapter_start_locked_keeps_feedback_and_explicit_recovery_can_unlock(fake_sdk, guard):
    fake_sdk.degrees = [0.] * 22
    fake_sdk.degrees[17] = 21.
    hand = SharpaSdkHand('LEFT-SERIAL', .3, .6, False, control_mode='mit',
                         sdk_factory=lambda: fake_sdk, clock=fake_sdk.clock,
                         joint_limits=guard.limits)
    hand.start()
    try:
        assert hand.mit_limits.state == 'limit_locked'
        assert hand.get_joint_positions()[17] == math.radians(21.)
        with pytest.raises(ValueError, match='limit_locked'):
            hand.set_joint_positions([0.] * 22)
        assert not any(c[0] in ('mit_write', 'stop', 'disconnect') for c in fake_sdk.calls)
        current = hand.begin_recovery()
        hand.set_recovery_positions(current)
        assert fake_sdk.calls[-1][1][17] == .2618
        with pytest.raises(ValueError, match='recovering'):
            hand.set_joint_positions([0.] * 22)
        hand.set_recovery_positions([0.] * 22)
        fake_sdk.degrees = [0.] * 22
        hand.finish_recovery()
        hand.set_joint_positions([0.] * 22)
        assert hand.mit_limits.state == 'normal'
    finally:
        hand.stop()


def test_adapter_recovery_worsening_trips_sdk(fake_sdk, guard):
    fake_sdk.degrees = [0.] * 22
    fake_sdk.degrees[17] = 21.
    hand = SharpaSdkHand('LEFT-SERIAL', .3, .6, False, control_mode='mit',
                         sdk_factory=lambda: fake_sdk, clock=fake_sdk.clock, joint_limits=guard.limits)
    hand.start()
    hand.begin_recovery()
    fake_sdk.degrees[17] = 21.6
    hand.get_joint_positions()
    fake_sdk.now += .21
    with pytest.raises(BackendError, match='worsened'):
        hand.get_joint_positions()
    assert hand.mit_limits.state == 'faulted'
    assert fake_sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]
