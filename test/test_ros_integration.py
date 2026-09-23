"""Installed launch and two real child processes; no device or SDK required."""

from collections import defaultdict
import os
import re
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import yaml

from dual_sharpa_wave.joint_names import joint_names
from dual_sharpa_wave.qos import hand_qos


@pytest.mark.integration
@pytest.mark.parametrize('use_rviz', [False, True])
def test_dual_mock_over_dds(tmp_path, use_rviz):
    # Dedicated domain and localhost discovery isolate this test from any robot graph.
    domain = 180 + os.getpid() % 30
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'logs'), PYTHONUNBUFFERED='1')
    guard = tmp_path / 'guard'
    guard.mkdir()
    marker = tmp_path / 'sdk_import_attempted'
    (guard / 'sitecustomize.py').write_text(
        'import sys\nfrom pathlib import Path\n'
        'class Guard:\n'
        '    def find_spec(self, fullname, path=None, target=None):\n'
        '        if fullname == "sharpa" or fullname.startswith("sharpa."):\n'
        f'            Path({str(marker)!r}).write_text(fullname)\n'
        '            raise RuntimeError("Official SDK import forbidden in mock test")\n'
        'sys.meta_path.insert(0, Guard())\n'
    )
    env['PYTHONPATH'] = str(guard) + os.pathsep + env.get('PYTHONPATH', '')
    config = {}
    for side in ('left', 'right'):
        config[f'/sharpa/{side}_hand/hand_node'] = {'ros__parameters': {
            # Deliberately contradictory: mock launch must pin side and backend.
            'backend': 'sharpa_sdk', 'side': 'right' if side == 'left' else 'left',
            'mock_mode': 'first_order', 'mock_max_velocity_rad_s': 0.2,
            'command_timeout_sec': 0.15, 'publish_rate_hz': 30.0,
        }}
    config_path = tmp_path / 'hands.yaml'
    config_path.write_text(yaml.safe_dump(config))
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    observer = Node('mock_test_observer', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(observer)
    messages = defaultdict(list)
    subscriptions = []
    transforms = {}
    if use_rviz:
        def on_tf(msg):
            for transform in msg.transforms:
                transforms[transform.child_frame_id] = transform

        subscriptions.append(observer.create_subscription(
            TFMessage, '/tf', on_tf, QoSProfile(depth=100),
        ))
        subscriptions.append(observer.create_subscription(
            TFMessage, '/tf_static', on_tf,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ))
    publishers = {}
    for side in ('left', 'right'):
        subscriptions.append(observer.create_subscription(
            JointState, f'/sharpa/{side}_hand/joint_states',
            lambda msg, side=side: messages[side].append(msg), hand_qos(),
        ))
        publishers[side] = observer.create_publisher(
            JointState, f'/sharpa/{side}_hand/joint_command', hand_qos(),
        )
    log_path = tmp_path / 'launch.log'
    stream = log_path.open('w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'dual_sharpa_wave', 'dual_sharpa.launch.py', 'backend:=mock',
         f'config_file:={config_path}', 'publish_rate_hz:=60.0',
         f'use_rviz:={str(use_rviz).lower()}'],
        env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
    )

    def wait_for(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
            if predicate():
                return
            assert proc.poll() is None, log_path.read_text()
        raise AssertionError(f'Timed out waiting for mock ROS state.\n{log_path.read_text()}')

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    try:
        wait_for(lambda: all(messages[s] and publishers[s].get_subscription_count() == 1
                             for s in ('left', 'right')))
        wait_for(lambda: all(
            ('hand_node', f'/sharpa/{s}_hand') in observer.get_node_names_and_namespaces()
            for s in ('left', 'right')
        ))
        nodes = observer.get_node_names_and_namespaces()
        pids = re.findall(r'\[hand_node-\d+\]: process started with pid \[(\d+)\]', log_path.read_text())
        assert len(set(pids)) == 2, log_path.read_text()
        if use_rviz:
            wait_for(lambda: ('sharpa_preview_rviz', '/') in observer.get_node_names_and_namespaces())
            wait_for(lambda: len(transforms) == 70)
            initial_rotations = {name: transform.transform.rotation for name, transform in transforms.items()}
            # The model test checks disjoint child sets; only these two publishers
            # may own the preview TF graph (Jazzy MessageInfo has no publisher GID).
            for topic in ('/tf', '/tf_static'):
                endpoints = observer.get_publishers_info_by_topic(topic)
                assert len(endpoints) == 2
                assert {(e.node_name, e.node_namespace) for e in endpoints} == {
                    ('preview_state_publisher', '/sharpa/left_hand'),
                    ('preview_state_publisher', '/sharpa/right_hand'),
                }
        for side in ('left', 'right'):
            assert ('hand_node', f'/sharpa/{side}_hand') in nodes
            msg = messages[side][-1]
            assert msg.name == list(joint_names(side))
            assert list(msg.position) == [0.0]*22
            assert msg.header.stamp.sec > 0
            endpoints = (
                observer.get_publishers_info_by_topic(f'/sharpa/{side}_hand/joint_states')
                + observer.get_subscriptions_info_by_topic(f'/sharpa/{side}_hand/joint_command')
            )
            assert len(endpoints) == 2
            for endpoint in endpoints:
                assert endpoint.topic_type == 'sensor_msgs/msg/JointState'
                qos = endpoint.qos_profile
                assert qos.reliability == ReliabilityPolicy.RELIABLE
                assert qos.durability == DurabilityPolicy.VOLATILE
                # Some DDS implementations report UNKNOWN history/depth in discovery.
                if qos.history != HistoryPolicy.UNKNOWN:
                    assert qos.history == HistoryPolicy.KEEP_LAST
                    assert qos.depth == 1
        assert not any('/sharpa/dual_hand/' in name for name, _ in observer.get_topic_names_and_types())
        publishers['left'].publish(JointState(position=[0.3]+[0.0]*21))
        wait_for(lambda: any(0.01 < msg.position[0] < 0.25 for msg in messages['left']))
        wait_for(lambda: abs(messages['left'][-1].position[0] - 0.3) < 1e-6)
        assert all(list(msg.position) == [0.0]*22 for msg in messages['right'])
        if use_rviz:
            spin_for(0.2)
            left_rotations = {name: t.transform.rotation for name, t in transforms.items()}
            assert any(left_rotations[n] != initial_rotations[n] for n in transforms if '/left_' in n)
            assert all(left_rotations[n] == initial_rotations[n] for n in transforms if '/right_' in n)
        # Movement exceeds the 0.15s timeout and still reaches the retained target.
        assert 'Command timeout' in log_path.read_text()
        right_target = [0.1, -0.12] + [0.0]*20
        publishers['right'].publish(JointState(
            name=list(reversed(joint_names('right'))), position=list(reversed(right_target)),
            velocity=[999.0], effort=[999.0],
        ))
        wait_for(lambda: list(messages['right'][-1].position) == right_target)
        assert messages['left'][-1].position[0] == pytest.approx(0.3)
        if use_rviz:
            spin_for(0.2)
            assert all(transforms[n].transform.rotation == left_rotations[n]
                       for n in transforms if '/left_' in n)
            assert any(transforms[n].transform.rotation != initial_rotations[n]
                       for n in transforms if '/right_' in n)
            assert all(name.startswith('sharpa_preview/') for name in transforms)
        bad_messages = [
            JointState(position=[9.0]*21), JointState(position=[9.0]*23),
            JointState(position=[float('nan')]*22), JointState(position=[float('inf')]*22),
            JointState(position=[-float('inf')]*22),
            JointState(name=['unknown']+list(joint_names('left'))[1:], position=[9.0]*22),
            JointState(name=[joint_names('left')[0]]*22, position=[9.0]*22),
            JointState(name=list(joint_names('left'))[:-1], position=[9.0]*22),
        ]
        before = len(messages['left'])
        for msg in bad_messages:
            publishers['left'].publish(msg)
            spin_for(0.08)
        spin_for(0.15)
        assert len(messages['left']) > before
        assert all(list(msg.position) == [0.3]+[0.0]*21 for msg in messages['left'][before:])
        publishers['left'].publish(JointState(position=[0.0]*22))
        wait_for(lambda: list(messages['left'][-1].position) == [0.0]*22)
        assert list(messages['right'][-1].position) == right_target
        # Stopping the left process must leave the right hand responsive.
        os.kill(int(pids[0]), signal.SIGINT)
        wait_for(lambda: publishers['left'].get_subscription_count() == 0)
        publishers['right'].publish(JointState(position=[0.0]*22))
        wait_for(lambda: list(messages['right'][-1].position) == [0.0]*22)
        assert not marker.exists(), 'Mock attempted to import the official SDK'
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        stream.close()
        executor.shutdown()
        observer.destroy_node()
        ctx.try_shutdown()
    assert proc.returncode == 0, log_path.read_text()
    assert 'process has died' not in log_path.read_text(), log_path.read_text()
