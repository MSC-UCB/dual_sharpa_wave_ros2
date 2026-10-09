"""Real ROS actions and paired client with Fake SDKs in an isolated DDS domain."""

import math
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dual_sharpa_wave.action import RecoverDefault

from conftest import FakeSdk
from dual_sharpa_wave import hand_node
from dual_sharpa_wave.control_ros import CommandClient
from dual_sharpa_wave.recovery_client import driver_modes, recover_both
from dual_sharpa_wave.sharpa_sdk_hand import SharpaSdkHand


@pytest.fixture
def system(monkeypatch, request):
    ctx = Context()
    rclpy.init(context=ctx, domain_id=183)
    executor = SingleThreadedExecutor(context=ctx)
    sdks, nodes = {}, {}
    tracking = {'left': True, 'right': True}

    def factory(parameters):
        side = parameters['side']
        sdk = FakeSdk()
        sdk.degrees = [0.] * 22
        sdk.degrees[17] = getattr(request, 'param', 21.)
        sdks[side] = sdk
        return SharpaSdkHand('LEFT-SERIAL', .3, .6, False, control_mode='mit',
                             side=side, sdk_factory=lambda: sdk)

    monkeypatch.setattr(hand_node, 'create_backend', factory)
    for side in ('left', 'right'):
        settings = dict(side=side, backend='sharpa_sdk', control_mode='mit', interpolation=False,
                        serial_number='LEFT-SERIAL')
        nodes[side] = hand_node.HandNode(
            context=ctx, namespace=f'/sharpa/{side}_hand', enable_rosout=False,
            parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()])
        executor.add_node(nodes[side])
    client = CommandClient('test_default_recovery', None, .5, context=ctx, enable_rosout=False)
    executor.add_node(client)

    def writes(side):
        return [c for c in sdks[side].calls if c[0] == 'mit_write']

    def spin(node=None, timeout_sec=.01):
        for side in nodes:
            if tracking[side] and writes(side):
                sdks[side].degrees = [math.degrees(q) for q in writes(side)[-1][1]]
        executor.spin_once(timeout_sec=timeout_sec)

    def wait(predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, 'Offline DDS/action wait timed out'
            spin()

    args = SimpleNamespace(wait_timeout=3., minimum_duration=.05, max_speed_deg_s=360.,
                           max_accel_deg_s2=10000., rate=100., tolerance_deg=2.,
                           hold_time=.05, settle_timeout=.3)
    subject = SimpleNamespace(ctx=ctx, executor=executor, nodes=nodes, sdks=sdks, client=client,
                              spin=spin, wait=wait, writes=writes, args=args, tracking=tracking)
    try:
        wait(lambda: all(client.ready(s) for s in nodes))
        yield subject
    finally:
        for node in nodes.values():
            node.destroy_node()
        client.destroy_node()
        executor.shutdown(timeout_sec=1)
        ctx.try_shutdown()


def request():
    return RecoverDefault.Goal(minimum_duration_sec=.1, max_speed_rad_s=6.,
                               max_acceleration_rad_s2=100., command_rate_hz=100.,
                               tolerance_rad=math.radians(2), hold_time_sec=.05,
                               settle_timeout_sec=.3)


def send_goal(system, action, value=None):
    system.wait(action.server_is_ready)
    future = action.send_goal_async(request() if value is None else value)
    system.wait(future.done)
    return future.result()


def test_paired_script_recovers_locked_hands_and_can_repeat(system):
    s = system
    assert driver_modes(s.client, 3., s.spin, lambda: True) == 'mit'
    assert all(n._backend.mit_limits.state == 'limit_locked' for n in s.nodes.values())
    assert recover_both(s.client, s.args, s.spin, lambda: True) == 0
    for side, node in s.nodes.items():
        assert s.writes(side)[0][1][17] == .2618
        assert node._recovery._start_positions[17] == math.radians(21.)
        assert node._backend.mit_limits.state == 'normal'
        assert not any(c[0] in ('stop', 'disconnect') for c in s.sdks[side].calls)
    assert recover_both(s.client, s.args, s.spin, lambda: True) == 0


@pytest.mark.parametrize('system', [0., 21.], indirect=True)
def test_no_heartbeat_never_moves_and_preserves_measured_limit_state(system):
    s = system
    action = ActionClient(s.client, RecoverDefault, '/sharpa/left_hand/recover_default')
    heartbeat = s.client.create_publisher(String, '/sharpa/left_hand/recovery_heartbeat', 1)
    try:
        goal = send_goal(s, action)
        assert goal.accepted
        heartbeat.publish(String(data='not-the-active-goal'))
        result = goal.get_result_async()
        s.wait(result.done)
        assert not result.result().result.success
        assert 'heartbeat' in result.result().result.message
        assert not s.writes('left')
        expected = 'limit_locked' if s.sdks['left'].degrees[17] > 20. else 'normal'
        assert s.nodes['left']._backend.mit_limits.state == expected
    finally:
        action.destroy()
        s.client.destroy_publisher(heartbeat)


def test_heartbeat_loss_after_start_releases_without_disconnect(system):
    s = system
    action = ActionClient(s.client, RecoverDefault, '/sharpa/left_hand/recover_default')
    heartbeat = s.client.create_publisher(String, '/sharpa/left_hand/recovery_heartbeat', 1)
    try:
        goal = send_goal(s, action, RecoverDefault.Goal(
            minimum_duration_sec=3., max_speed_rad_s=1., max_acceleration_rad_s2=1.,
            command_rate_hz=100., tolerance_rad=.02, hold_time_sec=.1, settle_timeout_sec=1.))
        s.wait(lambda: heartbeat.get_subscription_count() == 1)
        heartbeat.publish(String(data=bytes(goal.goal_id.uuid).hex()))
        s.wait(lambda: bool(s.writes('left')))
        result = goal.get_result_async()
        s.wait(result.done)
        count = len(s.writes('left'))
        for _ in range(10):
            s.spin()
        assert len(s.writes('left')) == count
        assert 'heartbeat' in result.result().result.message
        assert s.nodes['left']._backend.mit_limits.state == 'normal'
        assert not any(c[0] in ('stop', 'disconnect') for c in s.sdks['left'].calls)
    finally:
        action.destroy()
        s.client.destroy_publisher(heartbeat)


@pytest.mark.parametrize('system', [0., 21.], indirect=True)
def test_cancel_preparation_preserves_limits_and_blocks_commands_while_active(system):
    s = system
    action = ActionClient(s.client, RecoverDefault, '/sharpa/left_hand/recover_default')
    try:
        goal = send_goal(s, action)
        assert goal.accepted
        assert not send_goal(s, action).accepted
        s.client.command_publishers['left'].publish(JointState(position=[0.] * 22))
        cancel = goal.cancel_goal_async()
        s.wait(cancel.done)
        result = goal.get_result_async()
        s.wait(result.done)
        assert not result.result().result.success
        assert 'canceled' in result.result().result.message
        assert not s.writes('left')
        expected = 'limit_locked' if s.sdks['left'].degrees[17] > 20. else 'normal'
        assert s.nodes['left']._backend.mit_limits.state == expected
    finally:
        action.destroy()


def test_one_goal_rejected_other_side_never_moves(system):
    s = system
    s.nodes['right'].settings['publish_rate_hz'] = 50.  # Reject requested 100 Hz.
    with pytest.raises(RuntimeError, match='right.*rejected'):
        recover_both(s.client, s.args, s.spin, lambda: True)
    s.wait(lambda: not s.nodes['left']._recovery.active)
    assert not s.writes('left') and not s.writes('right')


@pytest.mark.parametrize('system', [3.], indirect=True)
def test_one_sdk_fault_cancels_peer(system):
    s = system
    s.sdks['right'].fail = 'mit_write_status'
    with pytest.raises(RuntimeError, match='right'):
        recover_both(s.client, s.args, s.spin, lambda: True)
    s.wait(lambda: not s.nodes['left']._recovery.active)
    assert s.nodes['right']._backend.mit_limits.state == 'faulted'
    assert s.nodes['left']._backend.mit_limits.state == 'normal'
    count = len(s.writes('left'))
    for _ in range(10):
        s.spin()
    assert len(s.writes('left')) == count


@pytest.mark.parametrize('system', [3., 21.], indirect=True)
def test_stuck_feedback_cannot_claim_recovery_success(system):
    s = system
    s.tracking['right'] = False
    with pytest.raises(RuntimeError, match='right.*settling timed out'):
        recover_both(s.client, s.args, s.spin, lambda: True)
    hand = s.nodes['right']._backend
    expected = 'limit_locked' if s.sdks['right'].degrees[17] > 20. else 'normal'
    assert hand.mit_limits.state == expected
    if expected == 'normal':
        # Timeout is still reported as failure, but a fresh ordinary command
        # can take over without first reaching the default pose.
        hand.set_joint_positions(hand.get_joint_positions())
        assert not hand._faulted


def test_interrupt_active_paired_recovery_cancels_both(system):
    s = system
    interrupted = False

    def spin(node=None, timeout_sec=.01):
        nonlocal interrupted
        s.spin(node, timeout_sec)
        if not interrupted and all(s.writes(side) for side in s.nodes):
            interrupted = True
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        recover_both(s.client, s.args, spin, lambda: True)
    s.wait(lambda: all(not n._recovery.active for n in s.nodes.values()))
    assert all(n._backend.mit_limits.state == 'normal' for n in s.nodes.values())
    counts = {side: len(s.writes(side)) for side in s.nodes}
    for _ in range(10):
        s.spin()
    assert all(len(s.writes(side)) == count for side, count in counts.items())


def test_read_error_during_recovery_faults_and_cancels_peer(system):
    s = system
    injected = False

    def spin(node=None, timeout_sec=.01):
        nonlocal injected
        s.spin(node, timeout_sec)
        if not injected and all(s.writes(side) for side in s.nodes):
            injected = True
            s.sdks['right'].fail = 'read_status'

    with pytest.raises(RuntimeError, match='right'):
        recover_both(s.client, s.args, spin, lambda: True)
    s.wait(lambda: not s.nodes['left']._recovery.active)
    assert s.nodes['left']._backend.mit_limits.state == 'normal'
    assert s.nodes['right']._backend.mit_limits.state == 'faulted'


def test_invalid_recovery_request_does_not_reserve_driver(system):
    s = system
    action = ActionClient(s.client, RecoverDefault, '/sharpa/left_hand/recover_default')
    try:
        value = request()
        value.max_speed_rad_s = float('nan')
        assert not send_goal(s, action, value).accepted
        assert not s.nodes['left']._recovery.active
        assert not s.writes('left')
    finally:
        action.destroy()
