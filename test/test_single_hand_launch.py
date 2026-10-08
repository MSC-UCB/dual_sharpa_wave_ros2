"""Installed single-hand launch checks; enable with SINGLE_SIDE_ROS_MOCK=1."""

from contextlib import contextmanager
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from dual_sharpa_wave.joint_names import joint_names

pytestmark = pytest.mark.skipif(os.environ.get('SINGLE_SIDE_ROS_MOCK') != '1',
                                reason='set SINGLE_SIDE_ROS_MOCK=1 for software mock launch checks')


@contextmanager
def stack(tmp_path, side, read_only, preview=False):
    log = tmp_path / 'launch.log'
    env = dict(os.environ, ROS_DOMAIN_ID='201', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    with log.open('w') as stream:
        process = subprocess.Popen([
            'ros2', 'launch', 'dual_sharpa_wave', 'single_sharpa.launch.py',
            f'prefix:={side}_', 'backend:=mock', f'read_only:={str(read_only).lower()}',
            f'use_rviz:={str(preview).lower()}'], env=env, stdout=stream,
            stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(domain_id=201)
    node = Node('single_hand_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    def wait(predicate, timeout=15.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
            assert process.poll() is None, log.read_text()
        pytest.fail(log.read_text())
    try:
        yield node, wait, log
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('read_only', [False, True])
def test_single_hand_feedback_commands_and_isolation(tmp_path, side, read_only):
    with stack(tmp_path, side, read_only) as (node, wait, log):
        received = []
        root = f'/sharpa/{side}_hand'
        node.create_subscription(JointState, root + '/joint_states', received.append, 10)
        wait(lambda: received and ('hand_node', root) in node.get_node_names_and_namespaces())
        assert received[-1].name == list(joint_names(side))
        other = 'right' if side == 'left' else 'left'
        assert not any(f'/sharpa/{other}_hand/' in name for name, _ in node.get_topic_names_and_types())
        assert not any(f'/sharpa/{other}_hand/' in name for name, _ in node.get_service_names_and_types())
        if read_only:
            assert node.count_subscribers(root + '/joint_command') == 0
        else:
            pub = node.create_publisher(JointState, root + '/joint_command', 1)
            wait(lambda: pub.get_subscription_count() == 1)
            target = [.01] + [0.] * 21
            timer = node.create_timer(.05, lambda: pub.publish(JointState(name=list(joint_names(side)), position=target)))
            wait(lambda: list(received[-1].position) == target)
            timer.cancel()
        assert 'Traceback' not in log.read_text() and 'process has died' not in log.read_text()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_single_hand_preview_tf_and_rviz(tmp_path, side):
    with stack(tmp_path, side, False, preview=True) as (node, wait, log):
        transforms = {}
        def receive(message):
            transforms.update({t.child_frame_id: t for t in message.transforms})
        node.create_subscription(TFMessage, '/tf', receive, 100)
        node.create_subscription(TFMessage, '/tf_static', receive,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wait(lambda: len(transforms) == 35 and 'sharpa_preview_rviz' in node.get_node_names())
        other = 'right' if side == 'left' else 'left'
        assert not any(other + '_' in frame for frame in transforms)
        assert not any(f'/sharpa/{other}_hand/' in name for name, _ in node.get_topic_names_and_types())
        for topic in ('/tf', '/tf_static'):
            assert {(e.node_name, e.node_namespace) for e in node.get_publishers_info_by_topic(topic)} == {
                ('preview_state_publisher', f'/sharpa/{side}_hand')}
        assert 'process has died' not in log.read_text()
