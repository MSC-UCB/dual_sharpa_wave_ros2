"""Frame contract validation with synthetic SDK callbacks; no native SDK imports."""

from dataclasses import FrozenInstanceError
import json
import time
from unittest.mock import Mock

import numpy as np
import pytest
from builtin_interfaces.msg import Time
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message

from dual_sharpa_wave import hand_node
from dual_sharpa_wave.msg import TactileFrame
from dual_sharpa_wave.tactile import TactileCache, FINGERS
from dual_sharpa_wave.tactile_contract import (
    MISSING, VALID, EMPTY, INVALID, MAX_VALUES, raw_block, to_frame_message)
from dual_sharpa_wave.tactile_capture import FingerSummary, parse_args


def payload(channel=0, ident=1, ts=1., value=1.):
    return dict(channel=channel, frame_id=ident, ts=ts,
                content={'F6': [value, -value, 2., -2., 3., -3.],
                         'CONTACT_POINT': np.array([[[value, 2., 3., 4.]]], np.float32),
                         'DIST_FORCE': np.full((60, 60, 3), value, np.float32)},
                shape={'F6': [1, 1, 6], 'CONTACT_POINT': [1, 1, 4],
                       'DIST_FORCE': [60, 60, 3]})


def retained(p=None, side='right'):
    cache = TactileCache(side, lambda: Time(sec=7, nanosec=8), lambda: 1_000_000_000)
    cache.receive(payload() if p is None else p)
    frames, error = cache.drain()
    assert error is None and len(frames) == 1
    return frames[0]


@pytest.mark.parametrize('side,base', [('right', 0), ('left', 5)])
def test_all_channel_mappings(side, base):
    for i, finger in enumerate(FINGERS):
        f = retained(payload(channel=base+i), side)
        assert (f.side, f.finger, f.channel) == (side, finger, base+i)
    for channel in (-1, 10, base+5, True, '0'):
        c = TactileCache(side)
        c.receive(payload(channel=channel))
        assert c.drain()[1]


@pytest.mark.parametrize('shape', [(0, 3), (1, 3), (2, 4), (1, 2, 5), (3,)])
def test_contact_points_shape_and_all_columns(shape):
    values = np.arange(np.prod(shape), dtype=np.float32).reshape(shape)
    b = raw_block('CONTACT_POINT', {'CONTACT_POINT': values}, {'CONTACT_POINT': list(shape)})
    assert b.status == (EMPTY if values.size == 0 else VALID)
    assert b.shape == shape and json.loads(b.declared_shape_json) == list(shape)
    np.testing.assert_array_equal(b.values, values)


@pytest.mark.parametrize('shape', [[1, 5], [-1, 3], [1.0, 3], [True, 3], '1,3', [2**40, 3]])
def test_malformed_contact_shape_is_explicit(shape):
    p = payload()
    p['shape']['CONTACT_POINT'] = shape
    f = retained(p)
    assert f.raw_blocks['CONTACT_POINT'].status == INVALID
    assert 'CONTACT_POINT' in f.errors
    assert f.raw_blocks['F6'].status == f.raw_blocks['DIST_FORCE'].status == VALID
    assert json.loads(f.raw_blocks['CONTACT_POINT'].declared_shape_json) == shape


def test_immutable_frame_with_owned_arrays_and_stamp():
    p = payload()
    f = retained(p)
    p['content']['CONTACT_POINT'][:] = 99
    p['content']['DIST_FORCE'][:] = 99
    with pytest.raises(FrozenInstanceError):
        f.channel = 4
    with pytest.raises(TypeError):
        f.blocks['RAW'] = None
    for a in (f.f6, f.blocks['DIST_FORCE'], f.raw_blocks['CONTACT_POINT'].values):
        with pytest.raises(ValueError):
            a.setflags(write=True)
    assert f.raw_blocks['CONTACT_POINT'].values.flat[0] == 1.
    stamp = f.stamp
    stamp.sec = 99
    assert f.stamp.sec == 7


def test_missing_empty_invalid_and_no_cross_frame_mixing():
    c = TactileCache('right', lambda: Time(sec=7))
    c.receive(payload())
    p = payload(ident=2, ts=2., value=2.)
    del p['content']['F6']
    p['content']['CONTACT_POINT'] = np.empty((0, 4), np.float32)
    p['shape']['CONTACT_POINT'] = [0, 4]
    c.receive(p)
    f = c.drain()[0][0]
    assert f.overwritten == 1 and f.receipt_sequence == 2
    assert f.f6 is None and f.raw_blocks['F6'].status == MISSING
    assert f.raw_blocks['CONTACT_POINT'].status == EMPTY
    assert np.all(f.raw_blocks['DIST_FORCE'].values == 2.)
    p['ts'] = 3.
    p['content']['CONTACT_POINT'] = [[np.nan]]
    p['shape']['CONTACT_POINT'] = [1, 1]
    c.receive(p)
    f = c.drain()[0][0]
    assert f.raw_blocks['CONTACT_POINT'].status == INVALID
    assert f.raw_blocks['DIST_FORCE'].status == VALID


@pytest.mark.parametrize('name', ['F6', 'CONTACT_POINT', 'DIST_FORCE'])
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite_block_does_not_discard_other_blocks(name, bad):
    p = payload()
    a = np.asarray(p['content'][name]).copy()
    a.flat[0] = bad
    p['content'][name] = a
    f = retained(p)
    assert f.raw_blocks[name].status == INVALID
    assert all(b.status == VALID for key, b in f.raw_blocks.items() if key != name)


def test_payload_bound_and_dist_grid_policy_does_not_change_legacy_image():
    b = raw_block('CONTACT_POINT', {'CONTACT_POINT': np.ones(MAX_VALUES + 1)}, {})
    assert b.status == INVALID and 'exceeds' in b.error
    p = payload()
    p['content']['DIST_FORCE'] = np.ones((2, 3, 3), np.float32)
    p['shape']['DIST_FORCE'] = [2, 3, 3]
    f = retained(p)
    assert f.raw_blocks['DIST_FORCE'].status == INVALID
    assert f.blocks['DIST_FORCE'].shape == (2, 3, 3)  # Legacy behavior unchanged.


def test_identity_duplicates_restart_wrap_reorder_and_session():
    c = TactileCache('right', lambda: Time(sec=7))
    c.receive(payload(ident=2**31-1))
    first = c.drain()[0][0]
    c.receive(payload(ident=2**31-1))
    assert c.drain() == ([], None)
    c.receive(payload(ident=-(2**31), ts=2.))
    wrapped = c.drain()[0][0]
    assert wrapped.frame_id == -(2**31) and wrapped.duplicates == 1
    assert wrapped.receipt_sequence == 3 and wrapped.identity_warnings
    c.receive(payload(ident=10, ts=10.))
    c.drain()
    c.receive(payload(ident=0, ts=.1))
    restarted = c.drain()[0][0]
    assert 'sdk_timestamp_regressed' in restarted.identity_warnings
    c.receive(payload(ident=8, ts=8.))
    c.drain()
    c.receive(payload(ident=7, ts=7.))
    assert c.drain()[0][0].identity_warnings
    assert retained().session_id != first.session_id


@pytest.mark.parametrize('field,value', [('frame_id', 2**40), ('frame_id', None),
                                       ('frame_id', True), ('ts', np.nan), ('ts', None),
                                       ('ts', 10**400)])
def test_bad_identity_keeps_valid_blocks_with_explicit_error(field, value):
    p = payload()
    p[field] = value
    f = retained(p)
    assert field in f.errors
    msg = to_frame_message(f, Time(sec=8), monotonic_ns=2_000_000_000)
    assert msg.f6.status == msg.contact_points.status == msg.dist_force.status == VALID
    assert not (msg.sdk_frame_id_valid if field == 'frame_id' else msg.sdk_timestamp_valid)


def test_wire_roundtrip_preserves_raw_values_shapes_identity_and_timing():
    f = retained(payload(ident=-12, ts=123.456, value=.125))
    msg = to_frame_message(f, Time(sec=8), monotonic_ns=1_200_000_000)
    msg = deserialize_message(serialize_message(msg), TactileFrame)
    assert msg.schema_version == 1 and msg.header.stamp == Time(sec=7, nanosec=8)
    assert msg.header.frame_id == ''
    assert msg.sdk_frame_id == -12 and msg.sdk_timestamp_raw == 123.456
    assert msg.receive_to_publish_sec == .2
    assert msg.sdk_timestamp_unit == msg.sdk_timestamp_epoch == 'unknown'
    assert list(msg.f6.raw_values) == [.125, -.125, 2., -2., 3., -3.]
    assert list(msg.f6.shape) == [1, 1, 6]
    assert list(msg.contact_points.shape) == [1, 1, 4]
    assert list(msg.contact_points.raw_values) == [.125, 2., 3., 4.]
    assert len(msg.dist_force.raw_values) == 10800


def test_summary_rates_statuses_sessions_and_missing_messages():
    s = FingerSummary('right', 'pinky')
    assert s.summary(10.)['observed_hz'] is None
    for ident in (1, 3):
        msg = to_frame_message(retained(payload(ident=ident, value=float(ident))), Time(sec=8), 2_000_000_000)
        msg.session_id = 'same-session'
        msg.receipt_sequence = ident
        s.receive(msg, float(ident), 8.)
    result = s.summary(4.)
    assert result['observed_hz'] == .5
    assert result['receipt_sequence_gap_total'] == 1
    assert result['sdk_id_forward_gap_events'] == 1
    assert result['blocks']['f6']['statuses']['valid'] == 2
    assert result['receive_to_publish_sec']['mean'] == 1.
    assert result['f6_component_min_raw'] == [1., -3., 2., -2., 3., -3.]
    assert result['f6_component_max_raw'] == [3., -1., 2., -2., 3., -3.]
    assert result['blocks']['dist_force']['first_sample_max_raw'] == 1.
    assert result['blocks']['dist_force']['max_raw'] == 3.
    msg.side = 'left'
    s.receive(msg, 4., 9.)
    assert s.rejected == 1 and s.count == 2


@pytest.mark.parametrize('duration', ['0', '-1', 'nan', 'inf'])
def test_capture_rejects_invalid_duration(duration):
    with pytest.raises(SystemExit):
        parse_args(['--duration', duration, '--output', '/tmp/unused.json'])


@pytest.mark.parametrize('side,base', [('right', 0), ('left', 5)])
def test_all_fingers_publish_coherent_messages_over_dds_without_motion(monkeypatch, side, base):
    ctx = Context()
    rclpy.init(context=ctx, domain_id=176)
    cache = TactileCache(side, lambda: Time(sec=7))
    backend = Mock()
    backend.take_tactile_frames.side_effect = cache.drain
    backend.get_joint_positions.return_value = [0.] * 22
    monkeypatch.setattr(hand_node, 'create_backend', lambda params: backend)
    driver = observer = None
    executor = SingleThreadedExecutor(context=ctx)
    try:
        driver = hand_node.HandNode(context=ctx, namespace=f'sharpa/{side}_hand',
            parameter_overrides=[Parameter('side', value=side), Parameter('read_only', value=True),
                                 Parameter('tactile_enabled', value=True)])
        observer = Node('frame_test_observer', context=ctx, enable_rosout=False)
        executor.add_node(driver)
        executor.add_node(observer)
        received = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        subs = [observer.create_subscription(TactileFrame,
                    f'/sharpa/{side}_hand/tactile/{finger}/frame',
                    lambda msg, finger=finger: received.update({finger: msg}), qos)
                for finger in FINGERS]
        end = time.monotonic() + 5
        ident = 0
        while len(received) < 5 and time.monotonic() < end:
            ident += 1
            for i in range(5):
                cache.receive(payload(channel=base+i, ident=ident, ts=float(ident), value=float(ident)))
            driver._on_tactile_timer()
            executor.spin_once(timeout_sec=.02)
        assert set(received) == set(FINGERS)
        for finger, msg in received.items():
            assert msg.finger == finger and msg.side == side
            assert msg.f6.raw_values[0] == msg.contact_points.raw_values[0] == msg.dist_force.raw_values[0] == msg.sdk_frame_id
        assert driver._subscription is None
        assert observer.count_publishers(f'/sharpa/{side}_hand/joint_command') == 0
        backend.set_joint_positions.assert_not_called()
    finally:
        executor.shutdown()
        if observer is not None:
            observer.destroy_node()
        if driver is not None:
            driver.destroy_node()
        ctx.try_shutdown()


def test_disabled_tactile_creates_no_frame_publishers(monkeypatch):
    ctx = Context()
    rclpy.init(context=ctx, domain_id=176)
    driver = None
    try:
        driver = hand_node.HandNode(context=ctx,
            parameter_overrides=[Parameter('read_only', value=True)])
        assert not driver._frame_publishers and not driver._f6_publishers
    finally:
        if driver is not None:
            driver.destroy_node()
        ctx.try_shutdown()


def test_recorder_has_only_subscriptions_and_reports_no_data(monkeypatch, tmp_path):
    from dual_sharpa_wave import tactile_capture
    import builtins
    original_import = builtins.__import__
    def guarded_import(name, *args, **kwargs):
        if name == 'sharpa' or name.startswith('sharpa.'):
            raise AssertionError('Recorder must not import the SDK')
        return original_import(name, *args, **kwargs)
    def forbidden(*args, **kwargs):
        raise AssertionError('Recorder must not create publishers or service clients')
    original_publisher = Node.create_publisher
    def infrastructure_only(self, msg_type, topic, *args, **kwargs):
        # rclpy creates its standard parameter-events publisher during Node init.
        assert topic == '/parameter_events'
        return original_publisher(self, msg_type, topic, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded_import)
    monkeypatch.setattr(Node, 'create_publisher', infrastructure_only)
    monkeypatch.setattr(Node, 'create_client', forbidden)
    monkeypatch.setenv('ROS_DOMAIN_ID', '175')
    output = tmp_path / 'empty_capture.json'
    assert tactile_capture.main(['--side', 'both', '--duration', '.05', '--output', str(output)]) == 2
    result = json.loads(output.read_text())
    assert len(result['fingers']) == 10
    assert all(s['count'] == 0 for s in result['fingers'].values())
    with pytest.raises(FileExistsError):
        tactile_capture.main(['--output', str(output)])


def test_missing_message_support_fails_before_backend_creation(monkeypatch):
    from dual_sharpa_wave import tactile_contract
    ctx = Context()
    rclpy.init(context=ctx, domain_id=176)
    backend_factory = Mock()
    monkeypatch.setattr(hand_node, 'create_backend', backend_factory)
    def unavailable():
        raise RuntimeError('rebuild dual_sharpa_wave')
    monkeypatch.setattr(tactile_contract, 'message_type', unavailable)
    try:
        with pytest.raises(RuntimeError, match='rebuild'):
            hand_node.HandNode(context=ctx,
                parameter_overrides=[Parameter('tactile_enabled', value=True)])
        backend_factory.assert_not_called()
    finally:
        ctx.try_shutdown()


@pytest.mark.parametrize('kind', ['import', 'support'])
def test_type_support_errors_include_build_instruction(monkeypatch, kind):
    from rclpy import type_support
    from rclpy.exceptions import NoTypeSupportImportedException
    from dual_sharpa_wave.tactile_contract import message_type
    def unavailable(_):
        raise ImportError('missing library') if kind == 'import' else NoTypeSupportImportedException()
    monkeypatch.setattr(type_support, 'check_for_type_support', unavailable)
    with pytest.raises(RuntimeError, match='colcon build'):
        message_type()
