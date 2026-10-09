from unittest.mock import Mock

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from dual_sharpa_wave import hand_node
from dual_sharpa_wave.hand_interface import BackendError
from dual_sharpa_wave.sharpa_sdk_hand import SharpaSdkHand


@pytest.fixture
def context():
    ctx = Context()
    rclpy.init(context=ctx, domain_id=171)
    yield ctx
    ctx.try_shutdown()


@pytest.fixture
def node(context, monkeypatch):
    backend = Mock()
    backend.get_joint_positions.return_value = [0.05]*22
    monkeypatch.setattr(hand_node, 'create_backend', lambda parameters: backend)
    result = hand_node.HandNode(context=context, enable_rosout=False)
    # Exercise shared node logic without sending messages to a public topic here.
    publisher = Mock()
    result._publisher = publisher
    yield result, backend, publisher
    result.destroy_node()


def test_feedback_is_backend_position_and_ros_stamp(node):
    subject, backend, pub = node
    subject._on_command(JointState(position=[0.8]*22, velocity=[999.0], effort=[999.0]))
    subject._on_timer()
    msg = pub.publish.call_args.args[0]
    assert list(msg.position) == [0.05]*22
    assert msg.name == list(subject.names)
    assert msg.header.stamp.sec > 0
    assert list(msg.velocity) == list(msg.effort) == []


def test_timeout_and_invalid_command(node, monkeypatch):
    subject, backend, pub = node
    now = subject._started_at
    monkeypatch.setattr(hand_node.time, 'monotonic', lambda: now)
    subject._on_command(JointState(position=[0.2]*22))
    accepted_at = subject._last_command
    now += 0.6
    subject._on_command(JointState(position=[9.0]*21))
    assert subject._last_command == accepted_at
    assert backend.set_joint_positions.call_count == 1
    subject._on_timer()
    assert subject.timed_out
    backend.update.assert_called_once()
    pub.publish.assert_called_once()
    subject._on_command(JointState(position=[0.3]*22))
    assert not subject.timed_out


def test_write_error_does_not_refresh_timeout(node):
    subject, backend, pub = node
    backend.set_joint_positions.side_effect = BackendError('write failed')
    subject._on_command(JointState(position=[0.2]*22))
    assert subject._last_command is None


@pytest.mark.parametrize('feedback', [[], [0.0]*21, [float('nan')]*22, [float('inf')]*22])
def test_bad_feedback_is_not_published(node, feedback):
    subject, backend, pub = node
    backend.get_joint_positions.return_value = feedback
    subject._on_timer()
    pub.publish.assert_not_called()


def test_read_failure_and_cleanup(node):
    subject, backend, pub = node
    backend.get_joint_positions.side_effect = BackendError('read failed')
    subject._on_timer()
    pub.publish.assert_not_called()
    backend.stop.side_effect = BackendError('stop failed')
    subject.destroy_node()
    subject.destroy_node()
    backend.stop.assert_called_once()


def test_partial_startup_cleanup(context, monkeypatch):
    backend = Mock()
    backend.start.side_effect = BackendError('start failed')
    monkeypatch.setattr(hand_node, 'create_backend', lambda parameters: backend)
    with pytest.raises(BackendError, match='start failed'):
        hand_node.HandNode(context=context, enable_rosout=False)
    backend.stop.assert_called_once()


@pytest.mark.parametrize('rate', [0.0, -1.0, float('nan'), float('inf')])
def test_bad_publish_rate(context, rate):
    with pytest.raises(ValueError):
        hand_node.HandNode(
            context=context, parameter_overrides=[Parameter('publish_rate_hz', value=rate)],
        )


def test_hardware_requires_serial_before_sdk_import():
    with pytest.raises(ValueError, match='serial_number'):
        SharpaSdkHand('', 0.3, 0.6, True)


def test_timeout_hook_once_after_first_accepted_command(node, monkeypatch):
    subject, backend, pub = node
    now = subject._started_at + 1.0
    monkeypatch.setattr(hand_node.time, 'monotonic', lambda: now)
    subject._on_timer()
    backend.on_command_timeout.assert_not_called()
    subject._on_command(JointState(position=[0.1]*22))
    now += 0.6
    subject._on_timer()
    subject._on_timer()
    backend.on_command_timeout.assert_called_once()


def test_real_hardware_adapter_with_fake_sdk_uses_shared_ros_node(context, monkeypatch, fake_sdk):
    adapter = SharpaSdkHand('LEFT-SERIAL', 0.3, 0.6, True, sdk_factory=lambda: fake_sdk)
    monkeypatch.setattr(hand_node, 'create_backend', lambda parameters: adapter)
    subject = hand_node.HandNode(context=context, enable_rosout=False)
    subject._publisher = Mock()
    try:
        now = subject._started_at
        monkeypatch.setattr(hand_node.time, 'monotonic', lambda: now)
        subject._on_command(JointState(position=[0.25] * 22))
        subject._on_timer()
        published = subject._publisher.publish.call_args.args[0]
        assert list(published.position) == pytest.approx(adapter.get_joint_positions())
        assert list(published.position) != [0.25] * 22
        subject._publisher.reset_mock()
        now += 0.6
        subject._on_timer()
        subject._publisher.publish.assert_not_called()
        assert fake_sdk.calls[-2:] == [('stop',), ('disconnect', 'LEFT-SERIAL')]
        writes = sum(c[0] == 'write' for c in fake_sdk.calls)
        subject._on_command(JointState(position=[0.2] * 22))
        assert sum(c[0] == 'write' for c in fake_sdk.calls) == writes
    finally:
        subject.destroy_node()


def test_mit_node_with_fake_sdk_gains_commands_and_timeout(context, monkeypatch, fake_sdk):
    from dual_sharpa_wave import sharpa_sdk_hand
    fake_sdk.degrees = [0.0] * 22
    monkeypatch.setattr(sharpa_sdk_hand, 'load_sdk', lambda: fake_sdk)
    settings = dict(backend='sharpa_sdk', serial_number='LEFT-SERIAL',
                    control_mode='mit', interpolation=False, command_timeout_sec=0.5)
    subject = hand_node.HandNode(context=context, enable_rosout=False,
                                parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()])
    subject._publisher = Mock()
    now = subject._started_at
    monkeypatch.setattr(hand_node.time, 'monotonic', lambda: now)
    subject._backend._clock = lambda: now
    try:
        assert ('mode', 'MIT') in fake_sdk.calls
        assert subject._backend.mit_settings['torque_source'] == 0
        assert not any(c[0] == 'mit_write' for c in fake_sdk.calls)
        # First command acquires the measured pose. Then accept named/reordered targets.
        subject._on_command(JointState(position=[0.0] * 22))
        now += 0.01
        fake_sdk.now += 0.01
        target = [0.0] * 22
        target[0] = 0.05
        subject._on_command(JointState(name=list(reversed(subject.names)), position=list(reversed(target))))
        assert fake_sdk.calls[-1][0] == 'mit_write'
        assert fake_sdk.calls[-1][1] == target
        accepted_at = subject._last_command
        calls = len(fake_sdk.calls)
        subject._on_command(JointState(position=target, effort=[0.0] * 22))
        subject._on_command(JointState(position=target, velocity=[0.0] * 22))
        subject._on_command(JointState(position=[float('nan')] * 22))
        assert subject._last_command == accepted_at
        assert len(fake_sdk.calls) == calls
        # Feedback timer must not resend commands or refresh command age.
        subject._on_timer()
        assert list(subject._publisher.publish.call_args.args[0].position) == [0.0] * 22
        assert sum(c[0] == 'mit_write' for c in fake_sdk.calls) == 2
        now += 0.51
        subject._on_timer()
        assert not any(c[0] in ('stop', 'disconnect') for c in fake_sdk.calls)
        assert subject.timed_out
        assert subject._publisher.publish.call_count == 2
        assert sum(c[0] == 'mit_write' for c in fake_sdk.calls) == 2
        fake_sdk.degrees = [1.0] * 22
        now += 60
        subject._on_timer()
        assert list(subject._publisher.publish.call_args.args[0].position) == pytest.approx(
            [sharpa_sdk_hand.math.radians(1.0)] * 22)
        subject._on_command(JointState(position=[0.0] * 22))
        assert not subject.timed_out
        assert fake_sdk.calls[-1][1] == pytest.approx([sharpa_sdk_hand.math.radians(1.0)] * 22)
        subject._on_command(JointState(position=target))
        assert fake_sdk.calls[-1][1] == target
    finally:
        subject.destroy_node()


@pytest.mark.parametrize('settings', [
    {'control_mode': 'invalid'},
    {'control_mode': 'mit', 'interpolation': True},
    {'control_mode': 'mit', 'interpolation': False, 'command_timeout_sec': 0.0},
    {'control_mode': 'mit', 'interpolation': False, 'mit_start_tolerance_rad': float('nan')},
])
def test_invalid_mit_node_settings_precede_backend(context, monkeypatch, settings):
    backend = Mock()
    monkeypatch.setattr(hand_node, 'create_backend', backend)
    with pytest.raises(ValueError):
        hand_node.HandNode(context=context, enable_rosout=False,
                          parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()])
    backend.assert_not_called()
