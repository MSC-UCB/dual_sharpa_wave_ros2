"""Offline tactile transport, display and read-only lifecycle tests; no native SDK."""

from array import array as byte_array
from types import SimpleNamespace as NS
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState

from dual_sharpa_wave import hand_node
from dual_sharpa_wave.hand_interface import BackendError
from dual_sharpa_wave.sharpa_sdk_hand import SharpaSdkHand
from dual_sharpa_wave.tactile import TactileCache, from_image, to_image
from dual_sharpa_wave.tactile_viewer import DisplayData, force_heatmap, parse_args, render


def frame(channel=5, ts=1., value=None):
    return dict(channel=channel, ts=ts, frame_id=int(ts), content={
        'DIST_FORCE': np.ones((6, 7, 3), np.float32) if value is None else value})


def test_cache_owns_data_bounded_latest_and_no_repeat():
    cache = TactileCache('left', lambda: Time(sec=7))
    payload = frame()
    cache.receive(payload)
    payload['content']['DIST_FORCE'][:] = 99
    frames, error = cache.drain()
    assert error is None and len(frames) == 1
    assert frames[0].stamp.sec == 7
    assert np.all(frames[0].blocks['DIST_FORCE'] == 1)
    assert cache.drain() == ([], None)
    cache.receive(payload)
    assert cache.drain() == ([], None)
    for ts in range(2, 100):
        cache.receive(frame(ts=float(ts)))
    assert len(cache.pending) == 1
    assert cache.drain()[0][0].sdk_ts == 99
    cache.close()
    cache.receive(frame(ts=100.))
    assert cache.drain() == ([], None)


def test_cache_wrong_side_and_malformed_blocks_are_not_zero_frames():
    cache = TactileCache('left')
    cache.receive(frame(channel=0))
    frames, error = cache.drain()
    assert not frames and 'channel' in error
    payload = frame(value=np.full((2, 2, 3), np.nan, np.float32))
    payload['content']['RAW'] = np.ones((2, 2), np.uint8)
    cache.receive(payload)
    frames, error = cache.drain()
    assert set(frames[0].blocks) == {'RAW'}
    assert 'nonfinite' in frames[0].errors['DIST_FORCE']
    cache.receive(None)
    assert cache.drain()[1]


@pytest.mark.parametrize('dtype,shape', [(np.uint8, (5, 6)), (np.uint8, (3, 4, 3)),
                                      (np.float32, (4, 5)), (np.float32, (6, 7, 3))])
def test_transport_roundtrip(dtype, shape):
    source = np.arange(np.prod(shape), dtype=dtype).reshape(shape)
    msg = to_image(source, Time(sec=123, nanosec=45))
    assert msg.header.stamp == Time(sec=123, nanosec=45)
    assert np.array_equal(from_image(msg), source)


def test_transport_big_endian_padded_rows_and_bad_stride():
    data = np.arange(12, dtype='>f4').reshape(2, 2, 3)
    wire = b''.join(row.tobytes() + b'PAD!' for row in data)
    msg = Image(height=2, width=2, encoding='32FC3', is_bigendian=1, step=28,
                data=byte_array('B', wire))
    assert np.array_equal(from_image(msg), data)
    msg.step = 20
    with pytest.raises(ValueError):
        from_image(msg)


def test_cache_reshape_from_binding_shape_and_reject_mismatch():
    cache = TactileCache('right')
    payload = frame(channel=4, value=np.arange(18, dtype=np.float32))
    payload['shape'] = {'DIST_FORCE': [2, 3, 3]}
    cache.receive(payload)
    assert cache.drain()[0][0].blocks['DIST_FORCE'].shape == (2, 3, 3)
    payload['ts'] = 2.
    payload['shape']['DIST_FORCE'] = [1, 1, 3]
    cache.receive(payload)
    result = cache.drain()[0][0]
    assert not result.blocks and result.errors


def test_actual_sdk_flat_buffers_and_float_lists():
    cache = TactileCache('left')
    payload = frame(value=[.25] * 10800)
    payload['content']['RAW'] = np.ones(76800, np.uint8)
    payload['shape'] = {'RAW': [1, 240, 320], 'DIST_FORCE': [60, 60, 3]}
    cache.receive(payload)
    result = cache.drain()[0][0]
    assert not result.errors
    assert result.blocks['RAW'].shape == (240, 320)
    assert result.blocks['DIST_FORCE'].shape == (60, 60, 3)
    assert result.blocks['DIST_FORCE'].dtype == np.float32
    payload['content']['DIST_FORCE'][0] = 1e300
    payload['ts'] = 2.
    cache.receive(payload)
    result = cache.drain()[0][0]
    assert 'float32' in result.errors['DIST_FORCE']


def tactile_sdk(fake_sdk, *, side='left', ready=True):
    fake_sdk.HandSide = NS(LEFT='left', RIGHT='right')
    fake_sdk.get_device_info = lambda: NS(hand_side=side, has_fingertip_tactile=lambda: True)
    fake_sdk.is_tactile_ready = lambda: ready
    fake_sdk.set_tactile_callback = lambda callback: setattr(fake_sdk, 'callback', callback)
    return fake_sdk


def test_read_only_sdk_never_configures_or_writes(fake_sdk):
    sdk = tactile_sdk(fake_sdk)
    hand = SharpaSdkHand('LEFT-SERIAL', .3, .6, True, sdk_factory=lambda: sdk,
                        read_only=True, tactile_enabled=True)
    hand.start()
    assert [call[0] for call in sdk.calls] == ['discover', 'connect', 'start', 'read']
    sdk.callback(frame())
    assert len(hand.take_tactile_frames()[0]) == 1
    with pytest.raises(BackendError, match='Read-only'):
        hand.set_joint_positions([0.] * 22)
    hand.stop()
    sdk.callback(frame(ts=2.))
    assert hand.take_tactile_frames() == ([], None)
    assert sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]
    assert not any(c[0] in ('write', 'mode', 'source', 'speed', 'current') for c in sdk.calls)


@pytest.mark.parametrize('side,channel', [('left', 5), ('right', 0)])
@pytest.mark.parametrize('read_only', [True, False])
def test_tactile_restart_uses_new_session_cache(fake_sdk, side, channel, read_only):
    sdk = tactile_sdk(fake_sdk, side=side)
    hand = SharpaSdkHand('LEFT-SERIAL', .3, .6, True, sdk_factory=lambda: sdk,
                        read_only=read_only, tactile_enabled=True, side=side,
                        stamp_clock=lambda: Time(sec=5))
    old_callbacks = []
    # Stopping before first start must not prevent future tactile delivery either.
    hand.stop()
    try:
        for _ in range(3):
            hand.start()
            callback = sdk.callback
            assert hand.take_tactile_frames() == ([], None)
            for old_callback in old_callbacks:
                old_callback(frame(channel=channel, ts=99.))
                old_callback(None)
            assert hand.take_tactile_frames() == ([], None)

            # The same frame identity is valid again in a new session.
            callback(frame(channel=channel))
            hand.start()  # Already started: preserve the pending frame and callback.
            assert sdk.callback is callback
            frames, error = hand.take_tactile_frames()
            assert error is None and len(frames) == 1
            assert frames[0].channel == channel
            assert frames[0].stamp == Time(sec=5)

            callback(frame(channel=channel, ts=2.))  # Leave a pending old frame.
            hand.stop()
            hand.stop()
            assert hand.take_tactile_frames() == ([], None)
            old_callbacks.append(callback)
    finally:
        hand.stop()


@pytest.mark.parametrize('side,ready', [('right', True), ('left', False)])
def test_tactile_start_failure_cleans_up(fake_sdk, side, ready):
    sdk = tactile_sdk(fake_sdk, side=side, ready=ready)
    hand = SharpaSdkHand('LEFT-SERIAL', .3, .6, True, sdk_factory=lambda: sdk,
                        read_only=True, tactile_enabled=True)
    with pytest.raises(BackendError):
        hand.start()
    assert sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]


def test_read_only_ros_node_publishes_new_images_without_motion_subscription(monkeypatch, fake_sdk):
    sdk = tactile_sdk(fake_sdk)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=177)
    adapter = SharpaSdkHand('LEFT-SERIAL', .3, .6, True, sdk_factory=lambda: sdk,
                           read_only=True, tactile_enabled=True, stamp_clock=lambda: Time(sec=5))
    monkeypatch.setattr(hand_node, 'create_backend', lambda parameters: adapter)
    subject = None
    try:
        subject = hand_node.HandNode(context=ctx, namespace='sharpa/left_hand',
            parameter_overrides=[Parameter('read_only', value=True),
                                 Parameter('tactile_enabled', value=True)])
        assert subject._subscription is None
        assert subject.count_subscribers('/sharpa/left_hand/joint_command') == 0
        subject._on_command(JointState(position=[0.] * 22))
        pub = Mock()
        subject._tactile_publishers['pinky', 'DIST_FORCE'] = pub
        sdk.callback(frame())
        subject._on_tactile_timer()
        subject._on_tactile_timer()
        pub.publish.assert_called_once()
        assert pub.publish.call_args.args[0].header.stamp.sec == 5
        assert not any(c[0] == 'write' for c in sdk.calls)
    finally:
        if subject is not None:
            subject.destroy_node()
        ctx.try_shutdown()


def test_viewer_fixed_scale_stale_and_missing(tmp_path):
    data = DisplayData()
    array = np.ones((60, 60, 3), np.float32)
    key = ('left', 'pinky', 'DIST_FORCE')
    data.receive(key, to_image(array, Time(sec=1)), 1.)
    data.receive(key, to_image(array, Time(sec=1)), 1.1)
    assert data.samples[key].received == 1.  # repeated stamp does not look fresh
    live = render(data, now=1.2, force_max=2.)
    stale = render(data, now=3., force_max=2.)
    assert live.shape == stale.shape == (1050, 1488, 3)
    assert not np.array_equal(live, stale)
    # A value has the same color regardless of other pixels in the frame.
    small = np.array([[[.5, 0., 0.], [1., 0., 0.]]], np.float32)
    large = small.copy()
    large[0, 1] = [100., 0., 0.]
    assert np.array_equal(force_heatmap(small, 1.)[0, 0], force_heatmap(large, 1.)[0, 0])
    assert cv2.imwrite(str(tmp_path / 'viewer.png'), live)
    bad = to_image(array, Time(sec=2))
    bad.data = byte_array('B', b'bad')
    data.receive(key, bad, 4.)
    assert key in data.errors
    render(data, now=4.)


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf'])
def test_invalid_heatmap_scale(value):
    with pytest.raises(SystemExit):
        parse_args(['--force-max', value])
