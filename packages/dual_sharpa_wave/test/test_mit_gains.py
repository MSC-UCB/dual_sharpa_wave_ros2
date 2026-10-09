"""Gain edits and transitions, using only a fake SDK."""

import json
from types import SimpleNamespace

import pytest

from dual_sharpa_wave.mit_gains import GainTransition, BASE_KP, BASE_KD, scaled_gains
from dual_sharpa_wave import sharpa_sdk_hand as sdk_module


def test_fixed_baseline_matches_saved_tuning():
    from pathlib import Path
    report = json.loads((Path(__file__).parents[1] / 'docs/mit_gain_io_2026-09-30.json').read_text())
    for hand in report:
        assert BASE_KP == pytest.approx(hand['before']['mit_kp'])
        assert BASE_KD == pytest.approx(hand['before']['mit_kd'])
    kp, kd = scaled_gains(.6, .8)
    assert kp[:5] == pytest.approx([12., 12., 4.8, 15., 2.4])
    assert kd[:5] == pytest.approx([.4, .4, .16, .2, .08])


def test_startup_ratio_writes_once_before_enabling_and_does_not_compound(fake_sdk, monkeypatch):
    fake_sdk.degrees = [0.] * 22
    mapping = tuple(reversed(range(22)))
    monkeypatch.setattr(sdk_module, 'SDK_TO_URDF_INDEX', mapping)
    monkeypatch.setattr(sdk_module, 'URDF_TO_SDK_INDEX', mapping)
    for ratio, kd_ratio, expected_writes in ((.6, .8, 1), (.6, .8, 1), (.6, 1., 2)):
        hand = sdk_module.SharpaSdkHand(
            'LEFT-SERIAL', .3, .6, False, control_mode='mit',
            mit_kp_ratio=ratio, mit_kd_ratio=kd_ratio, sdk_factory=lambda: fake_sdk)
        try:
            hand.start()
            hand.start()  # Repeated start on the same instance also does not write.
            assert sum(c[0] == 'gain_write' for c in fake_sdk.calls) == expected_writes
            kp, kd = scaled_gains(ratio, kd_ratio)
            assert fake_sdk.parameters['mit_kp'] == kp[::-1]
            assert hand.read_mit_gains() == (kp, kd)
            operations = [c[0] for c in fake_sdk.calls]
            assert operations.index('gain_write') < operations.index('mode') < operations.index('start')
            assert 'mit_write' not in operations and 'write' not in operations
        finally:
            hand.stop()


@pytest.mark.parametrize('readonly,mode', [(True, 'mit'), (True, 'position'), (False, 'position')])
def test_startup_ratio_does_not_write_other_modes(fake_sdk, readonly, mode):
    hand = sdk_module.SharpaSdkHand(
        'LEFT-SERIAL', .3, .6, False, control_mode=mode, read_only=readonly,
        mit_kp_ratio=.6, mit_kd_ratio=.8, sdk_factory=lambda: fake_sdk)
    try:
        hand.start()
        assert not any(c[0] == 'gain_write' for c in fake_sdk.calls)
    finally:
        hand.stop()


@pytest.mark.parametrize('ratio', [-.1, float('nan'), float('inf'), True])
@pytest.mark.parametrize('field', ['mit_kp_ratio', 'mit_kd_ratio'])
def test_invalid_startup_ratio_fails_before_connect(fake_sdk, ratio, field):
    with pytest.raises(ValueError):
        sdk_module.SharpaSdkHand('LEFT-SERIAL', .3, .6, False, control_mode='mit',
                                **{**dict(mit_kp_ratio=.6, mit_kd_ratio=.6), field: ratio},
                                sdk_factory=lambda: fake_sdk)
    assert not fake_sdk.calls


@pytest.mark.parametrize('failure', ['gain_write_status', 'mismatch', 'sensor_source'])
def test_startup_gain_failure_does_not_enable_control(fake_sdk, failure):
    if failure == 'sensor_source':
        fake_sdk.parameters.update(force_feedback_source=1, mit_kp_fs=[8.] * 10, mit_kd_fs=[.1] * 10)
    elif failure == 'mismatch':
        fake_sdk.set_parameter = lambda payload: SimpleNamespace(code=0)
    else:
        fake_sdk.fail = failure
    hand = sdk_module.SharpaSdkHand(
        'LEFT-SERIAL', .3, .6, False, control_mode='mit',
        mit_kp_ratio=.6, mit_kd_ratio=.8, sdk_factory=lambda: fake_sdk)
    with pytest.raises(Exception):
        hand.start()
    assert not any(c[0] in ('mode', 'source', 'start', 'mit_write') for c in fake_sdk.calls)
    assert ('disconnect', 'LEFT-SERIAL') in fake_sdk.calls


@pytest.fixture
def hand(fake_sdk):
    fake_sdk.degrees = [0.] * 22

    hand = sdk_module.SharpaSdkHand(
        'LEFT-SERIAL', .3, .6, False, sdk_factory=lambda: fake_sdk, control_mode='mit')
    hand.start()
    yield hand
    hand.stop()


def test_two_verified_steps_and_idle_never_writes(hand, fake_sdk):
    ramp = GainTransition(hand)
    ramp.read()
    ramp.step()
    assert not any(c[0] == 'gain_write' for c in fake_sdk.calls)
    ramp.apply([18.] * 22, [.2] * 22)
    for step in range(1, 3):
        ramp.step()
        assert ramp.kp == pytest.approx([8. + step * 5.] * 22)
        assert ramp.kd == pytest.approx([.1 + step * .05] * 22)
        assert ramp.remaining == 2 - step
    ramp.step()
    assert sum(c[0] == 'gain_write' for c in fake_sdk.calls) == 2
    assert not any(c[0] in ('mit_write', 'write') for c in fake_sdk.calls)


def test_replace_cancel_and_unchanged_target(hand, fake_sdk):
    ramp = GainTransition(hand)
    ramp.apply([18.] * 22, [.1] * 22)
    ramp.step()
    ramp.apply([19.] * 22, [.1] * 22)
    ramp.step()
    assert ramp.kp == [16.] * 22  # 8 -> 13; new target 19 -> midpoint 16.
    ramp.cancel()
    ramp.step()
    assert sum(c[0] == 'gain_write' for c in fake_sdk.calls) == 2
    ramp.apply(ramp.kp, ramp.kd)
    assert ramp.remaining == 0


@pytest.mark.parametrize('bad', [[1.] * 21, [-1.] * 22, [float('nan')] * 22,
                                  [float('inf')] * 22])
def test_bad_input_does_not_write(hand, fake_sdk, bad):
    with pytest.raises(ValueError):
        GainTransition(hand).apply(bad, [.1] * 22)
    assert not any(c[0] == 'gain_write' for c in fake_sdk.calls)


@pytest.mark.parametrize('failure', ['gain_write_status', 'parameters_status', 'mismatch'])
def test_write_failure_stops_ramp_without_stopping_driver(hand, fake_sdk, failure):
    ramp = GainTransition(hand)
    ramp.apply([18.] * 22, [.2] * 22)
    if failure == 'mismatch':
        fake_sdk.set_parameter = lambda payload: SimpleNamespace(code=0)
    else:
        fake_sdk.fail = failure
    ramp.step()
    assert ramp.failed and not ramp.remaining
    assert ramp.kp == ramp.kd == []
    assert hand._started and not hand._faulted
    assert not any(c[0] in ('stop', 'disconnect') for c in fake_sdk.calls)


@pytest.mark.parametrize('lock_before_apply', [True, False])
def test_locked_hand_can_tune_without_unlocking_motion(hand, fake_sdk, lock_before_apply):
    ramp = GainTransition(hand)
    if lock_before_apply:
        hand.mit_limits.lock('Default pose settling timed out')
    ramp.apply([18.] * 22, [.2] * 22)
    if not lock_before_apply:
        hand.mit_limits.lock('Measured limit exceeded during gain transition')
    reason = hand.mit_limits.reason
    for _ in range(2):
        ramp.step()
    assert not ramp.failed and not ramp.remaining
    assert hand.read_mit_gains() == ([18.] * 22, [.2] * 22)
    assert hand.mit_limits.state == 'limit_locked'
    assert hand.mit_limits.reason == reason
    with pytest.raises(ValueError, match='limit_locked'):
        hand.set_joint_positions([0.] * 22)
    assert sum(c[0] == 'gain_write' for c in fake_sdk.calls) == 2
    assert not any(c[0] in ('mit_write', 'write', 'stop', 'disconnect') for c in fake_sdk.calls)


@pytest.mark.parametrize('state', ['recovering', 'faulted'])
def test_existing_control_state_prevents_gain_writes(hand, fake_sdk, state):
    ramp = GainTransition(hand)
    ramp.apply([18.] * 22, [.2] * 22)
    hand.mit_limits.state = state
    ramp.step()
    assert ramp.failed and not ramp.remaining
    assert not any(c[0] == 'gain_write' for c in fake_sdk.calls)


def test_mapping_scalar_and_source(hand, fake_sdk, monkeypatch):
    mapping = tuple(reversed(range(22)))
    monkeypatch.setattr(sdk_module, 'SDK_TO_URDF_INDEX', mapping)
    monkeypatch.setattr(sdk_module, 'URDF_TO_SDK_INDEX', mapping)
    fake_sdk.parameters.update(mit_kp=8., mit_kd=.1)
    assert hand.read_mit_gains() == ([8.] * 22, [.1] * 22)
    kp, kd = list(map(float, range(22))), [.1] * 22
    assert hand.write_mit_gains(kp, kd) == (kp, kd)
    assert fake_sdk.parameters['mit_kp'] == kp[::-1]
    hand.mit_settings['torque_source'] = 1
    with pytest.raises(Exception, match='current-based'):
        hand.write_mit_gains(kp, kd)


def test_read_failure_stops_active_transition(hand, fake_sdk):
    ramp = GainTransition(hand)
    ramp.apply([18.] * 22, [.2] * 22)
    fake_sdk.fail = 'parameters_status'
    with pytest.raises(Exception):
        ramp.read()
    assert ramp.failed and not ramp.remaining and not ramp.kp
    fake_sdk.fail = None
    ramp.read()
    assert not ramp.failed and ramp.kp == [8.] * 22


@pytest.mark.parametrize('write_delay', [0., .31])
def test_gain_service_over_dds(hand, fake_sdk, monkeypatch, write_delay):
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from dual_sharpa_wave.srv import MitGains
    from dual_sharpa_wave import hand_node

    write = fake_sdk.set_parameter

    def delayed_write(payload):
        time.sleep(write_delay)
        return write(payload)

    fake_sdk.set_parameter = delayed_write

    ctx = Context()
    rclpy.init(context=ctx, domain_id=176)
    monkeypatch.setattr(hand_node, 'create_backend', lambda settings: hand)
    node = hand_node.HandNode(context=ctx, namespace='/mit_gain_test', parameter_overrides=[
        Parameter('backend', value='sharpa_sdk'), Parameter('control_mode', value='mit'),
        Parameter('interpolation', value=False)])
    client_node = Node('gain_test_client', context=ctx)
    client = client_node.create_client(MitGains, '/mit_gain_test/mit_gains')
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    executor.add_node(client_node)

    def request(operation, kp=None, kd=None):
        future = client.call_async(MitGains.Request(operation=operation, kp=kp or [], kd=kd or []))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.done()
        return future.result()

    try:
        assert client.wait_for_service(timeout_sec=3.)
        reads = sum(c[0] == 'parameters' for c in fake_sdk.calls)
        assert request(MitGains.Request.STATUS).success
        assert sum(c[0] == 'parameters' for c in fake_sdk.calls) == reads
        result = request(MitGains.Request.READ)
        assert result.success and list(result.kp) == [8.] * 22
        started = time.monotonic()
        assert request(MitGains.Request.APPLY, [18.] * 22, [.2] * 22).success
        deadline = started + 4.
        while node._gain_server.transition.remaining:
            executor.spin_once(timeout_sec=.01)
            assert time.monotonic() < deadline
        elapsed = time.monotonic() - started
        assert .39 <= elapsed < 1.0
        print(f'Two-step transition, write delay={write_delay:.2f}s: {elapsed:.3f}s')
        result = request(MitGains.Request.STATUS)
        assert result.success and list(result.kp) == [18.] * 22
        assert sum(c[0] == 'gain_write' for c in fake_sdk.calls) == 2
        assert not any(c[0] in ('mit_write', 'write') for c in fake_sdk.calls)
        assert request(MitGains.Request.APPLY, [8.] * 22, [.1] * 22).success
        assert request(MitGains.Request.CANCEL).remaining_steps == 0
        assert node._gain_server.timer.is_canceled()
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        ctx.try_shutdown()


def test_gui_only_writes_on_apply(monkeypatch):
    from concurrent.futures import Future
    import tkinter as tk
    from unittest.mock import Mock
    from dual_sharpa_wave.srv import MitGains
    from dual_sharpa_wave import mit_gains_gui as gui

    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('Tk display unavailable')
    root.withdraw()
    node = Mock()
    client = node.create_client.return_value
    client.service_is_ready.return_value = True
    monkeypatch.setattr(gui.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(gui.rclpy, 'spin_once', lambda *a, **kw: None)

    def respond(request):
        future = Future()
        future.set_result(MitGains.Response(success=True, message='read', kp=[8.] * 22, kd=[.1] * 22))
        return future

    client.call_async.side_effect = respond
    try:
        panel = gui.GainPanel(root, node, 'left')
        panel.tick()
        client.call_async.assert_not_called()
        panel.send(MitGains.Request.READ)
        panel.tick()
        assert panel.loaded and panel.kp[0].get() == '20.0'
        assert panel.actual[0].get() == '8 / 0.1'
        panel.kp[0].set('9.0')
        assert client.call_async.call_count == 1
        panel.send(MitGains.Request.APPLY)
        request = client.call_async.call_args.args[0]
        assert request.operation == MitGains.Request.APPLY
        assert list(request.kp) == [9.] + list(BASE_KP[1:])
        panel.tick()
        panel.kp[0].set('nan')
        panel.send(MitGains.Request.APPLY)
        assert client.call_async.call_count == 2
        # Separate ratios keep each array independent and never compound.
        panel.kp_ratio.set('0.5')
        assert [float(v.get()) for v in panel.kp] == [v * .5 for v in BASE_KP]
        assert [float(v.get()) for v in panel.kd] == list(BASE_KD)
        panel.kd_ratio.set('0.6')
        panel.kp_ratio.set('0.8')
        assert [float(v.get()) for v in panel.kp] == pytest.approx([v * .8 for v in BASE_KP])
        assert client.call_async.call_count == 2
        panel.send(MitGains.Request.APPLY)
        request = client.call_async.call_args.args[0]
        assert list(request.kp) == pytest.approx([v * .8 for v in BASE_KP])
        assert list(request.kd) == pytest.approx([v * .6 for v in BASE_KD])
        panel.tick()
        for invalid in ('nan', 'inf', '-1', '', '1e308'):
            panel.kp_ratio.set(invalid)
            panel.send(MitGains.Request.APPLY)
        assert client.call_async.call_count == 3
        panel.kp_ratio.set('1')
        assert [float(v.get()) for v in panel.kp] == list(BASE_KP)
        panel.kd_ratio.set('0')
        assert [float(v.get()) for v in panel.kd] == [0.] * 22
        assert [float(v.get()) for v in panel.kp] == list(BASE_KP)
        panel.send(MitGains.Request.READ)
        panel.tick()
        assert panel.kd_ratio.get() == '0'
        assert [float(v.get()) for v in panel.kd] == [0.] * 22
        panel.kd_ratio.set('1')
        assert [float(v.get()) for v in panel.kd] == list(BASE_KD)
    finally:
        root.destroy()
