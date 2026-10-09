"""Real DDS subscription with a Fake SDK; never connects to physical hardware."""

import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from dual_sharpa_wave.hand_node import HandNode
from dual_sharpa_wave.qos import hand_qos
from dual_sharpa_wave import sharpa_sdk_hand


@pytest.mark.integration
def test_mit_commands_over_dds_use_fake_sdk(monkeypatch, fake_sdk):
    fake_sdk.degrees = [0.0] * 22
    monkeypatch.setattr(sharpa_sdk_hand, 'load_sdk', lambda: fake_sdk)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=174)
    settings = dict(backend='sharpa_sdk', serial_number='LEFT-SERIAL', control_mode='mit',
                    interpolation=False, command_timeout_sec=0.3)
    subject = HandNode(context=ctx, namespace='/mit_offline_test',
                       parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()])
    observer = Node('mit_test_observer', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(subject)
    executor.add_node(observer)
    states = []
    observer.create_subscription(JointState, '/mit_offline_test/joint_states', states.append, hand_qos())
    pub = observer.create_publisher(JointState, '/mit_offline_test/joint_command', hand_qos())

    def writes():
        return [c for c in fake_sdk.calls if c[0] == 'mit_write']

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            executor.spin_once(timeout_sec=0.01)
            assert time.monotonic() < deadline, 'DDS condition timed out'

    try:
        wait_for(lambda: pub.get_subscription_count() == 1 and states)
        assert not writes()
        pub.publish(JointState(position=[0.0] * 22))
        wait_for(lambda: len(writes()) == 1)
        target = [0.0] * 22
        target[0] = 0.05
        pub.publish(JointState(name=list(reversed(subject.names)), position=list(reversed(target))))
        wait_for(lambda: len(writes()) == 2)
        assert writes()[-1][1] == target
        assert writes()[-1][2:] == ([0.0] * 22, [0.0] * 22)
        assert list(states[-1].position) == [0.0] * 22
        pub.publish(JointState(position=target, effort=[1.0] * 22))
        wait_for(lambda: subject.timed_out)
        assert len(writes()) == 2  # Neither rejected fields nor feedback timer send targets.
        assert not subject._backend._faulted
        assert not any(c[0] in ('stop', 'disconnect') for c in fake_sdk.calls)
        # Feedback stays live while idle, and a new sender can take over.
        fake_sdk.degrees = [1.0] * 22
        previous_states = len(states)
        wait_for(lambda: len(states) > previous_states + 2)
        assert list(states[-1].position) == pytest.approx([sharpa_sdk_hand.math.radians(1.0)] * 22)
        pub.publish(JointState(position=target))
        wait_for(lambda: len(writes()) == 3)
        assert writes()[-1][1] == pytest.approx([sharpa_sdk_hand.math.radians(1.0)] * 22)
        assert not subject.timed_out
        pub.publish(JointState(position=target))
        wait_for(lambda: len(writes()) == 4)
        assert writes()[-1][1] == target
        # A real SDK write error still closes and latches the session.
        fake_sdk.fail = 'mit_write_status'
        pub.publish(JointState(position=target))
        wait_for(lambda: subject._backend._faulted)
        assert ('stop',) in fake_sdk.calls
        assert ('disconnect', 'LEFT-SERIAL') in fake_sdk.calls
        assert not any(c[0] == 'write' for c in fake_sdk.calls)
    finally:
        executor.shutdown()
        subject.destroy_node()
        observer.destroy_node()
        ctx.try_shutdown()
