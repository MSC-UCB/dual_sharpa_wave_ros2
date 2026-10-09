import math
from unittest.mock import Mock

import pytest

from dual_sharpa_wave import sharpa_sdk_hand as module
from dual_sharpa_wave.hand_interface import BackendError


def make_hand(sdk, **kwargs):
    return module.SharpaSdkHand('LEFT-SERIAL', 0.3, 0.6, False, sdk_factory=lambda: sdk,
                               clock=sdk.clock, sleep=sdk.sleep, **kwargs)


def test_lifecycle_explicit_serial_and_independent_feedback(fake_sdk):
    hand = make_hand(fake_sdk)
    hand.start()
    hand.start()
    assert fake_sdk.calls[:8] == [
        ('discover',), ('connect', 'LEFT-SERIAL'), ('mode', 'POSITION'), ('speed', 0.3),
        ('current', 0.6), ('source', 'SDK'), ('start',), ('read',)]
    assert not any(c[0] == 'write' for c in fake_sdk.calls)
    command = [0.01 * i for i in range(22)]
    hand.set_joint_positions(command)
    assert fake_sdk.calls[-1] == ('write', command, False)
    assert hand.get_joint_positions() == pytest.approx([math.radians(i) for i in range(22)])
    hand.stop()
    before = list(fake_sdk.calls)
    hand.stop()
    assert fake_sdk.calls == before
    assert fake_sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]


def test_nonidentity_mapping_both_directions(fake_sdk, monkeypatch):
    mapping = tuple(range(1, 22)) + (0,)
    inverse = tuple(mapping.index(i) for i in range(22))
    monkeypatch.setattr(module, 'SDK_TO_URDF_INDEX', mapping)
    monkeypatch.setattr(module, 'URDF_TO_SDK_INDEX', inverse)
    hand = make_hand(fake_sdk)
    hand.start()
    hand.set_joint_positions([float(i) for i in range(22)])
    assert fake_sdk.calls[-1][1] == list(mapping)
    assert hand.get_joint_positions() == pytest.approx([math.radians(i) for i in inverse])
    hand.stop()


@pytest.mark.parametrize('failure', [
    'connect', 'mode', 'mode_status', 'speed_status', 'current_status', 'source_status',
    'start', 'start_false', 'read', 'read_status',
])
def test_partial_startup_failure_cleans_up_and_latches(fake_sdk, failure):
    fake_sdk.fail = failure
    hand = make_hand(fake_sdk)
    with pytest.raises(BackendError, match='startup failed'):
        hand.start()
    assert ('disconnect', 'LEFT-SERIAL') in fake_sdk.calls
    if failure != 'connect':
        assert ('stop',) in fake_sdk.calls
    fake_sdk.fail = None
    with pytest.raises(BackendError, match='fault latched'):
        hand.start()


def test_discovery_timeout_never_connects_other_serial(fake_sdk):
    fake_sdk.devices = ['RIGHT-SERIAL']
    hand = make_hand(fake_sdk, discovery_timeout_sec=0.25)
    with pytest.raises(BackendError, match='Discovery timed out'):
        hand.start()
    assert fake_sdk.now == pytest.approx(0.25)
    assert all(call == ('discover',) for call in fake_sdk.calls)


@pytest.mark.parametrize('failure', ['write', 'write_status', 'read', 'read_status'])
def test_runtime_failure_stops_and_rejects_future_commands(fake_sdk, failure):
    hand = make_hand(fake_sdk)
    hand.start()
    fake_sdk.fail = failure
    with pytest.raises(BackendError):
        if failure.startswith('write'):
            hand.set_joint_positions([0.1] * 22)
        else:
            hand.get_joint_positions()
    assert fake_sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]
    with pytest.raises(BackendError):
        hand.set_joint_positions([0.2] * 22)


@pytest.mark.parametrize('positions', [[0.0] * 21, [float('nan')] * 22, [float('inf')] * 22])
def test_invalid_feedback_is_never_substituted(fake_sdk, positions):
    hand = make_hand(fake_sdk)
    hand.start()
    fake_sdk.degrees = positions
    with pytest.raises(BackendError):
        hand.get_joint_positions()
    assert fake_sdk.calls[-1] == ('disconnect', 'LEFT-SERIAL')


def test_invalid_command_does_not_reach_sdk(fake_sdk):
    hand = make_hand(fake_sdk)
    hand.start()
    with pytest.raises(ValueError):
        hand.set_joint_positions([float('nan')] * 22)
    assert not any(c[0] == 'write' for c in fake_sdk.calls)
    hand.stop()


@pytest.mark.parametrize('failure', ['stop', 'stop_false', 'disconnect'])
def test_cleanup_error_reported_and_all_cleanup_attempted(fake_sdk, failure):
    hand = make_hand(fake_sdk)
    hand.start()
    fake_sdk.fail = failure
    with pytest.raises(BackendError):
        hand.stop()
    assert fake_sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]
    hand.stop()


def test_timeout_stops_session_and_never_auto_resumes(fake_sdk):
    hand = make_hand(fake_sdk)
    hand.start()
    with pytest.raises(BackendError, match='timeout'):
        hand.on_command_timeout()
    with pytest.raises(BackendError):
        hand.set_joint_positions([0.1] * 22)
    with pytest.raises(BackendError, match='restart'):
        hand.start()


@pytest.mark.parametrize('speed,current', [(0, 0.6), (1.1, 0.6), (0.3, -1), (float('nan'), 0.6)])
def test_invalid_coefficients_rejected_before_sdk_import(speed, current):
    factory = Mock()
    with pytest.raises(ValueError):
        module.SharpaSdkHand('LEFT-SERIAL', speed, current, True, sdk_factory=factory)
    factory.assert_not_called()


def test_import_failure_has_actionable_message(monkeypatch):
    def missing(name):
        assert name == 'sharpa'
        raise ImportError('no sharpa module')
    monkeypatch.setattr(module.importlib, 'import_module', missing)
    with pytest.raises(BackendError, match='PYTHONPATH'):
        module.SharpaSdkHand('LEFT-SERIAL', 0.3, 0.6, True).start()
