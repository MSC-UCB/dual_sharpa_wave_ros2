"""Run installed waveform scripts against two real mock hand processes over DDS."""

from collections import defaultdict
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from dual_sharpa_wave.joint_names import joint_names
from dual_sharpa_wave.qos import hand_qos


@pytest.mark.integration
@pytest.mark.parametrize('kind', ['sine', 'step'])
def test_wave_scripts_against_mock(tmp_path, kind):
    domain = 210 + os.getpid() % 20
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'logs'), PYTHONUNBUFFERED='1')
    guard = tmp_path / 'guard'
    guard.mkdir()
    (guard / 'sitecustomize.py').write_text(
        'import sys\n'
        'class Guard:\n'
        '    def find_spec(self, fullname, path=None, target=None):\n'
        '        if fullname == "sharpa" or fullname.startswith("sharpa."):\n'
        '            raise RuntimeError("SDK import forbidden in mock control test")\n'
        'sys.meta_path.insert(0, Guard())\n')
    env['PYTHONPATH'] = str(guard) + os.pathsep + env.get('PYTHONPATH', '')
    config = tmp_path / 'mock.yaml'
    config.write_text(yaml.safe_dump({f'/sharpa/{s}_hand/hand_node': {'ros__parameters': {
        'publish_rate_hz': 100.0}}
        for s in ('left', 'right')}))
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    observer = Node('wave_test_observer', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(observer)
    states, commands = {}, defaultdict(list)
    pubs, subs = {}, []
    for side in ('left', 'right'):
        subs.append(observer.create_subscription(
            JointState, f'/sharpa/{side}_hand/joint_states',
            lambda msg, s=side: states.__setitem__(s, list(msg.position)), hand_qos()))
        subs.append(observer.create_subscription(
            JointState, f'/sharpa/{side}_hand/joint_command',
            lambda msg, s=side: commands[s].append(msg), hand_qos()))
        pubs[side] = observer.create_publisher(JointState, f'/sharpa/{side}_hand/joint_command', hand_qos())
    processes, streams = [], []

    def start(command, name):
        stream = (tmp_path / name).open('w')
        streams.append(stream)
        proc = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    def spin_until(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while not predicate():
            executor.spin_once(timeout_sec=0.01)
            assert time.monotonic() < deadline, '\n'.join(
                p.read_text() for p in tmp_path.glob('*.log'))

    try:
        launch = start(['ros2', 'launch', 'dual_sharpa_wave', 'dual_sharpa.launch.py',
                        'backend:=mock',
                        f'config_file:={config}'], 'launch.log')
        # Observer itself is also a command subscriber.
        spin_until(lambda: len(states) == 2 and all(p.get_subscription_count() >= 2 for p in pubs.values()))
        baseline = {'left': [0.02] * 22, 'right': [0.03] * 22}
        for side in pubs:
            pubs[side].publish(JointState(position=baseline[side]))
        spin_until(lambda: states == baseline)
        # Release bootstrap command publishers before running the actual controller.
        for pub in pubs.values():
            observer.destroy_publisher(pub)
        commands.clear()
        options = (['--amplitude', '0.05', '--frequency', '1.0'] if kind == 'sine' else
                   ['--step-size', '0.05', '--base-seconds', '0.15', '--high-seconds', '0.2'])
        control = start(['ros2', 'run', 'dual_sharpa_wave', f'{kind}_control.py',
                         '--axes', 'thumb_CMC_FE,index_MCP_FE', '--cycles', '1', '--rate', '50',
                         # VOLATILE observers can discover a new publisher after its first cycle.
                         '--repeat', '3' if kind == 'sine' else '5',
                         '--gap', '0.1', '--settle-timeout', '3', *options], 'control.log')
        spin_until(lambda: control.poll() is not None)
        assert control.returncode == 0, (tmp_path / 'control.log').read_text()
        assert launch.poll() is None
        spin_until(lambda: all(states[s] == baseline[s] for s in baseline))
        keyed = {}
        for side in baseline:
            samples = commands[side]
            assert len(samples) > 10
            assert all(msg.name == list(joint_names(side)) for msg in samples)
            keyed[side] = {(m.header.stamp.sec, m.header.stamp.nanosec): m for m in samples}
            for axis in (0, 5):
                assert max(m.position[axis] for m in samples) > baseline[side][axis] + 0.04
                if kind == 'sine':
                    assert min(m.position[axis] for m in samples) < baseline[side][axis] - 0.04
            for msg in samples:
                assert all(msg.position[i] == baseline[side][i] for i in range(22) if i not in (0, 5))
                # Sequential axes: only one pair can differ from baseline per message.
                assert sum(abs(msg.position[i] - baseline[side][i]) > 1e-9 for i in (0, 5)) <= 1
        shared = set(keyed['left']) & set(keyed['right'])
        assert len(shared) > 10
        for stamp in shared:
            for axis in (0, 5):
                assert (keyed['left'][stamp].position[axis] - baseline['left'][axis]) == pytest.approx(
                    keyed['right'][stamp].position[axis] - baseline['right'][axis])
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
        for stream in streams:
            stream.close()
        executor.shutdown()
        observer.destroy_node()
        ctx.try_shutdown()
