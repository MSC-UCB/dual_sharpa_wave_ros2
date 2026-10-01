"""Offline MIT contract tests. No native SDK or hardware is used."""

import json
from unittest.mock import Mock

import pytest

from dual_sharpa_wave.hand_interface import BackendError
from dual_sharpa_wave.mit_control import read_mit_settings
from dual_sharpa_wave import sharpa_sdk_hand as module


@pytest.fixture
def mit_hand(fake_sdk):
    fake_sdk.degrees = [0.0] * 22
    hand = module.SharpaSdkHand(
        'LEFT-SERIAL', 0.3, 0.6, False, sdk_factory=lambda: fake_sdk,
        clock=fake_sdk.clock, sleep=fake_sdk.sleep, control_mode='mit')
    yield hand
    hand.stop()


def test_saved_gains_read_before_mode_change_no_automatic_targets(mit_hand, fake_sdk):
    before = json.dumps(fake_sdk.parameters)
    mit_hand.start()
    assert ('mode', 'MIT') in fake_sdk.calls
    assert ('read_mode',) in fake_sdk.calls
    assert fake_sdk.calls.index(('parameters', ['mit_kp', 'mit_kd'])) < fake_sdk.calls.index(('mode', 'MIT'))
    assert mit_hand.mit_settings['mit_kp'] == [8.0] * 22
    assert json.dumps(fake_sdk.parameters) == before
    assert not any(c[0] in ('write', 'mit_write') for c in fake_sdk.calls)


def test_first_target_uses_measured_pose_then_forwards_targets_unchanged(mit_hand, fake_sdk):
    mit_hand.start()
    fake_sdk.degrees = [1.0] * 22  # Changed after startup: takeover must re-read.
    mit_hand.set_joint_positions([0.05] * 22)
    first = fake_sdk.calls[-1]
    assert first[0] == 'mit_write'
    assert first[1] == pytest.approx([module.math.radians(1.0)] * 22)
    assert first[2:] == ([0.0] * 22, [0.0] * 22)
    fake_sdk.now += 0.01
    mit_hand.set_joint_positions([0.1] * 22)
    assert fake_sdk.calls[-1][1] == [0.1] * 22
    fake_sdk.now += 0.4
    mit_hand.set_joint_positions([0.0] * 22)
    assert fake_sdk.calls[-1][1] == [0.0] * 22
    assert not any(c[0] == 'write' for c in fake_sdk.calls)


def test_nonidentity_mit_mapping(mit_hand, fake_sdk, monkeypatch):
    mapping = tuple(reversed(range(22)))
    monkeypatch.setattr(module, 'SDK_TO_URDF_INDEX', mapping)
    monkeypatch.setattr(module, 'URDF_TO_SDK_INDEX', mapping)
    fake_sdk.degrees = [i / 10 for i in range(22)]
    mit_hand.start()
    current = mit_hand.get_joint_positions()
    mit_hand.set_joint_positions(current)
    assert fake_sdk.calls[-1][1] == pytest.approx([module.math.radians(i / 10) for i in range(22)])


@pytest.mark.parametrize('resume', [False, True])
@pytest.mark.parametrize('joint,actual', [
    (17, .26405816452245695), (7, -.002), (18, -.1745329300574517),
])
def test_targets_are_clipped_but_actual_feedback_is_unchanged(mit_hand, fake_sdk, resume, joint, actual):
    mit_hand.start()
    if resume:
        mit_hand.set_joint_positions([0.0] * 22)
        fake_sdk.now += 1.0
    fake_sdk.degrees[joint] = module.math.degrees(actual)
    raw = mit_hand.get_joint_positions()
    mit_hand.set_joint_positions(raw)
    expected = raw.copy()
    lo, hi = mit_hand.mit_limits.limits[joint]
    expected[joint] = min(hi, max(lo, actual))
    assert fake_sdk.calls[-1][1] == expected
    assert mit_hand.get_joint_positions() == raw
    fake_sdk.now += .01
    target = raw.copy()
    target[joint] = actual * 1.01
    mit_hand.set_joint_positions(target)
    assert fake_sdk.calls[-1][1] == mit_hand.mit_limits.clip(target)
    assert mit_hand.get_joint_positions() == raw


def test_first_target_distance_still_checked_against_actual_pose(mit_hand, fake_sdk):
    mit_hand.start()
    fake_sdk.degrees[17] = module.math.degrees(.20)
    target = [0.0] * 22
    target[17] = .05
    with pytest.raises(ValueError, match='load current pose'):
        mit_hand.set_joint_positions(target)
    assert not any(c[0] == 'mit_write' for c in fake_sdk.calls)
    assert mit_hand._mit_last_command_at is None


def test_arbitrary_outside_requests_clip_without_locking(mit_hand, fake_sdk):
    mit_hand.start()
    mit_hand.set_joint_positions([0.0] * 22)
    for requested, bound in ((99., 1), (-99., 0)):
        mit_hand.set_joint_positions([requested] * 22)
        assert fake_sdk.calls[-1][1] == [limits[bound] for limits in mit_hand.mit_limits.limits]
        assert mit_hand.mit_limits.state == 'normal'
        assert not mit_hand._faulted


@pytest.mark.parametrize('bad', [[0.0] * 21, [float('nan')] * 22, [float('inf')] * 22])
def test_invalid_positions_do_not_write_or_refresh_timeout(mit_hand, fake_sdk, bad):
    mit_hand.start()
    mit_hand.set_joint_positions([0.0] * 22)
    accepted_at = mit_hand._mit_last_command_at
    fake_sdk.now += .01
    with pytest.raises(ValueError):
        mit_hand.set_joint_positions(bad)
    assert sum(c[0] == 'mit_write' for c in fake_sdk.calls) == 1
    assert mit_hand._mit_last_command_at == accepted_at


@pytest.mark.parametrize('timer_first', [False, True])
def test_resume_after_idle_reacquires_live_pose_without_disconnect(mit_hand, fake_sdk, timer_first):
    mit_hand.start()
    mit_hand.set_joint_positions([0.0] * 22)
    for _ in range(2):
        fake_sdk.now += 60.0
        if timer_first:
            calls = list(fake_sdk.calls)
            mit_hand.on_command_timeout()
            mit_hand.on_command_timeout()
            assert fake_sdk.calls == calls  # Idle never sends a target or closes SDK.
        fake_sdk.degrees = [2.0] * 22
        mit_hand.set_joint_positions([0.05] * 22)
        assert fake_sdk.calls[-1][1] == pytest.approx([module.math.radians(2.0)] * 22)
        fake_sdk.now += 0.01
        mit_hand.set_joint_positions([0.05] * 22)
        assert fake_sdk.calls[-1][1] == [0.05] * 22
    assert not any(c[0] in ('stop', 'disconnect') for c in fake_sdk.calls)
    assert not mit_hand._faulted


def test_far_resume_target_rejected_but_current_pose_can_resume(mit_hand, fake_sdk):
    mit_hand.start()
    mit_hand.set_joint_positions([0.0] * 22)
    fake_sdk.now += 1.0
    with pytest.raises(ValueError, match='load current pose'):
        mit_hand.set_joint_positions([0.2] * 22)
    assert mit_hand._mit_last_command_at is None
    assert sum(c[0] == 'mit_write' for c in fake_sdk.calls) == 1
    mit_hand.set_joint_positions(mit_hand.get_joint_positions())
    assert sum(c[0] == 'mit_write' for c in fake_sdk.calls) == 2
    assert not any(c[0] in ('stop', 'disconnect') for c in fake_sdk.calls)


def test_idle_does_not_hide_feedback_fault(mit_hand, fake_sdk):
    mit_hand.start()
    mit_hand.set_joint_positions([0.0] * 22)
    mit_hand.on_command_timeout()
    fake_sdk.fail = 'read_status'
    with pytest.raises(BackendError):
        mit_hand.set_joint_positions([0.0] * 22)
    assert mit_hand._faulted
    assert fake_sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]
    with pytest.raises(BackendError):
        mit_hand.on_command_timeout()
    with pytest.raises(BackendError):
        mit_hand.set_joint_positions([0.0] * 22)


@pytest.mark.parametrize('failure', ['parameters_status', 'read_mode_status', 'mit_write_status'])
def test_mit_failures_latch_and_disconnect(mit_hand, fake_sdk, failure):
    if failure == 'mit_write_status':
        mit_hand.start()
    fake_sdk.fail = failure
    with pytest.raises(BackendError):
        if failure == 'mit_write_status':
            mit_hand.set_joint_positions([0.0] * 22)
        else:
            mit_hand.start()
    assert ('disconnect', 'LEFT-SERIAL') in fake_sdk.calls
    with pytest.raises(BackendError):
        mit_hand.start()
    if failure == 'parameters_status':
        assert not any(c[0] == 'mode' for c in fake_sdk.calls)


def test_mode_readback_mismatch(mit_hand, fake_sdk, monkeypatch):
    monkeypatch.setattr(fake_sdk, 'get_control_mode', lambda: (fake_sdk.status('read_mode'), 'POSITION'))
    with pytest.raises(BackendError, match='mismatch'):
        mit_hand.start()
    assert not any(c[0] in ('write', 'mit_write', 'start') for c in fake_sdk.calls)


@pytest.mark.parametrize('length', [10, 22])
def test_sensor_gains_and_legacy_source_are_preserved(fake_sdk, length):
    fake_sdk.parameters = {'torque_source': [1] * 22, 'mit_kp': 8.0, 'mit_kd': 0.1,
                           'mit_kp_fs': [9.0] * length, 'mit_kd_fs': [0.2] * length}
    settings = read_mit_settings(fake_sdk)
    assert settings == {'source_key': 'torque_source', 'torque_source': 1,
                        **{k: v for k, v in fake_sdk.parameters.items() if k != 'torque_source'}}


@pytest.mark.parametrize('key,value', [
    ('mit_kp', None), ('mit_kp', [1.0] * 21), ('mit_kd', -0.1),
    ('mit_kd', float('nan')), ('mit_kp', True), ('mit_kp', '8'),
    ('force_feedback_source', []), ('force_feedback_source', 2),
    ('force_feedback_source', [0, 1] * 11), ('force_feedback_source', None),
])
def test_invalid_saved_settings_fail_closed(fake_sdk, key, value):
    fake_sdk.parameters[key] = value
    with pytest.raises(BackendError):
        read_mit_settings(fake_sdk)


def test_sensor_source_requires_sensor_gains(fake_sdk):
    fake_sdk.parameters['force_feedback_source'] = 1
    with pytest.raises(BackendError, match='mit_kp_fs'):
        read_mit_settings(fake_sdk)


def test_unsupported_current_source_is_not_hidden_by_legacy_key(fake_sdk):
    fake_sdk.parameters.update(force_feedback_source=2, torque_source=0)
    with pytest.raises(BackendError, match='expected uniform'):
        read_mit_settings(fake_sdk)


@pytest.mark.parametrize('payload', ['not-json', '[]'])
def test_invalid_parameter_json(fake_sdk, monkeypatch, payload):
    monkeypatch.setattr(fake_sdk, 'get_parameter', lambda names: (fake_sdk.status('parameters'), payload))
    with pytest.raises(BackendError):
        read_mit_settings(fake_sdk)


@pytest.mark.parametrize('overrides', [
    {'control_mode': 'bad'}, {'interpolation': True},
    {'command_timeout_sec': 0},
    {'mit_start_tolerance_rad': -1},
])
def test_configuration_rejected_before_sdk_load(overrides):
    factory = Mock()
    kwargs = dict(control_mode='mit', interpolation=False)
    kwargs.update(overrides)
    with pytest.raises(ValueError):
        module.SharpaSdkHand('LEFT-SERIAL', 0.3, 0.6, sdk_factory=factory, **kwargs)
    factory.assert_not_called()


def test_read_only_mit_does_not_read_gains_or_configure(fake_sdk):
    hand = module.SharpaSdkHand('LEFT-SERIAL', 0.3, 0.6, True,
                               control_mode='mit', read_only=True, sdk_factory=lambda: fake_sdk)
    hand.start()
    try:
        assert not any(c[0] in ('parameters', 'mode', 'source', 'mit_write') for c in fake_sdk.calls)
        with pytest.raises(BackendError, match='Read-only'):
            hand.set_joint_positions([0.0] * 22)
    finally:
        hand.stop()
