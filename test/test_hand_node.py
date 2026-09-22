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


def test_hardware_placeholder_fails_without_sdk():
    with pytest.raises(ValueError, match='serial_number'):
        SharpaSdkHand('', 0.3, 0.6, True)
    backend = SharpaSdkHand('explicit-test-serial', 0.3, 0.6, True)
    with pytest.raises(NotImplementedError, match='No device connection'):
        backend.start()
    backend.stop()
