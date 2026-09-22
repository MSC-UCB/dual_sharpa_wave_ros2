from pathlib import Path
from unittest.mock import Mock

import pytest
from rclpy.context import Context
import rclpy
from sensor_msgs.msg import JointState

from dual_sharpa_wave import control_ros
from dual_sharpa_wave.control_model import load_limits
from dual_sharpa_wave.joint_names import joint_names


@pytest.fixture
def client():
    ctx = Context()
    rclpy.init(context=ctx, domain_id=172)
    node = control_ros.CommandClient('test_command_client', load_limits(Path(__file__).parents[1]),
                                     context=ctx, enable_rosout=False)
    node.command_publishers = {s: Mock() for s in ('left', 'right')}
    for side in node.command_publishers:
        node.command_publishers[side].get_subscription_count.return_value = 1
        node.on_state(side, JointState(name=list(joint_names(side)), position=[0.02] * 22))
    yield node
    node.destroy_node()
    ctx.try_shutdown()


def test_same_stamp_canonical_names_and_atomic_pair_validation(client):
    targets = {'left': [0.02] * 22, 'right': [0.03] * 22}
    client.send(targets)
    left = client.command_publishers['left'].publish.call_args.args[0]
    right = client.command_publishers['right'].publish.call_args.args[0]
    assert left.header.stamp == right.header.stamp
    assert left.name == list(joint_names('left'))
    assert right.name == list(joint_names('right'))
    assert list(left.position) == targets['left']
    targets['right'][0] = 999
    with pytest.raises(ValueError):
        client.send(targets)
    assert client.command_publishers['left'].publish.call_count == 1


def test_stale_invalid_feedback_and_missing_subscriber_stop_commands(client, monkeypatch):
    now = max(client.received_at.values()) + 1.01
    monkeypatch.setattr(control_ros.time, 'monotonic', lambda: now)
    with pytest.raises(RuntimeError, match='stale'):
        client.send({'left': [0.02] * 22})
    client.on_state('left', JointState(name=list(joint_names('left')), position=[0.02] * 22))
    assert client.ready('left')
    client.on_state('left', JointState(position=[0.02] * 22))
    assert not client.ready('left')
    assert 'include joint names' in client.feedback_errors['left']
    client.on_state('left', JointState(name=list(joint_names('left')), position=[0.02] * 22))
    client.command_publishers['left'].get_subscription_count.return_value = 0
    assert not client.ready('left')
    client.command_publishers['left'].publish.assert_not_called()


def test_feedback_reorders_by_name(client):
    client.on_state('left', JointState(name=list(reversed(joint_names('left'))),
                                      position=list(reversed([float(i) for i in range(22)]))))
    assert client.positions['left'] == list(range(22))
