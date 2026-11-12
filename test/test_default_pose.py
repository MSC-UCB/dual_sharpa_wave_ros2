"""Default pose regression tests with fake IO; no ROS/SDK/device connections."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT / 'dual_sharpa_wave').exists():
    ROOT = Path('/home/msc-crx/ws_fanuc/src/dual_sharpa_wave_ros2')
SCRIPT = ROOT / 'script/move_to_default_pose.py'
if not SCRIPT.exists():
    SCRIPT = Path(__file__).with_name('move_to_default_pose.py')
spec = importlib.util.spec_from_file_location('hand_default_pose', SCRIPT)
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)
from dual_sharpa_wave.control_model import load_limits, bounded_positions


def test_zero_within_real_limits_and_preview_no_ros(monkeypatch):
    limits = load_limits(ROOT)
    for s, q in motion.default_targets().items():
        assert bounded_positions(s, q, limits) == [0.] * 22
    monkeypatch.setattr(motion, 'execute', lambda args: pytest.fail('Unexpected execution'))
    assert motion.main([]) == 0


def test_quintic_endpoints_and_bounds():
    starts = {'left': [.6] * 22, 'right': [-.2] * 22}
    speed, accel = .15, .3
    plan = motion.ZeroMove(starts, 1., speed, accel)
    assert plan.sample(0) == starts
    assert plan.sample(plan.duration) == motion.default_targets()
    for i in range(1001):
        u = i / 1000
        velocity = .6 * 30 * u**2 * (1-u)**2 / plan.duration
        acceleration = .6 * abs(60*u - 180*u*u + 120*u**3) / plan.duration**2
        assert velocity <= speed + 1e-12
        assert acceleration <= accel + 1e-12
        target = plan.sample(u * plan.duration)
        assert 0 <= target['left'][0] <= .6
        assert -.2 <= target['right'][0] <= 0


@pytest.mark.parametrize('option,value', [('rate', '101'), ('state-timeout', 'nan'),
    ('minimum-duration', '0'), ('max-speed-deg-s', '-1'), ('hold-time', '15')])
def test_bad_cli(option, value):
    with pytest.raises(SystemExit):
        motion.parse_args(['--' + option, value])


@pytest.mark.parametrize('mode', ['success', 'overshoot', 'stale', 'competing', 'missing', 'stuck',
                                 'bad_feedback', 'lost_subscriber', 'nonfinite'])
@pytest.mark.parametrize('side', ['both', 'left', 'right'])
def test_lifecycle(monkeypatch, mode, side):
    clock, sent = [0.], []
    monkeypatch.setattr(motion.time, 'monotonic', lambda: clock[0])
    limits = load_limits(ROOT)
    targets = motion.default_targets(side)
    sides = tuple(targets)

    class FakeClient:
        def __init__(self):
            self.positions = {s: [.1] * 22 for s in sides}
            self.feedback_errors = {}
        def get_logger(self):
            return NS(info=lambda *args: None)
        def count_publishers(self, topic):
            assert any(f'/{s}_hand/' in topic for s in sides)
            return 2 if mode == 'competing' else 1
        def ready(self, side):
            assert side in sides
            return mode != 'missing' and not (mode in ('stale', 'lost_subscriber') and clock[0] > .1)
        def require_ready(self, selected):
            assert tuple(selected) == sides
            if not all(self.ready(s) for s in selected):
                raise RuntimeError('Not ready')
        def send(self, targets):
            assert tuple(targets) == sides
            sent.append(targets)
            if mode != 'stuck':
                self.positions = {s: list(q) for s, q in targets.items()}

    def spin(node, **kwargs):
        clock[0] += .02
        assert clock[0] < 20, 'Failed to terminate'
        if mode == 'bad_feedback':
            node.feedback_errors[sides[0]] = 'Malformed feedback'
        if mode == 'nonfinite':
            node.positions[sides[-1]] = [float('nan')] * 22
        if mode == 'overshoot' and clock[0] < .12:
            # Measured overshoot persists across the initial recovery ticks.
            node.positions[sides[-1]][17] = .26405816452245695
            node.positions[sides[0]][7] = -.002

    args = motion.parse_args(['--execute', '--side', side, '--minimum-duration', '.2', '--hold-time', '.1',
        '--wait-timeout', '.3', '--settle-timeout', '.3', '--tolerance-deg', '.01'])
    if mode in ('success', 'overshoot'):
        assert motion.run(FakeClient(), args, spin, lambda: True) == 0
        assert sent[-1] == targets
        if mode == 'overshoot':
            assert sent[0][sides[-1]][17] == .26405816452245695
            assert sent[0][sides[0]][7] == -.002
            assert all(a[sides[-1]][17] >= b[sides[-1]][17] for a, b in zip(sent, sent[1:]))
    else:
        expected = (TimeoutError if mode in ('missing', 'stuck') else
                    ValueError if mode == 'nonfinite' else RuntimeError)
        with pytest.raises(expected) as failure:
            motion.run(FakeClient(), args, spin, lambda: True)
        if mode == 'missing':
            assert 'not ready: ' + ', '.join(sides) in str(failure.value)
        if mode == 'stuck':
            assert f'{sides[0]}_thumb_CMC_FE error=5.730 deg' in str(failure.value)
            assert '22/22 outside 0.01 deg' in str(failure.value)
        if mode in ('competing', 'missing', 'bad_feedback', 'nonfinite'):
            assert not sent


def test_pose_error_summary_reports_each_side_and_degree_units():
    positions = motion.default_targets()
    positions['left'][8] = motion.math.radians(3.5)
    positions['right'][1] = motion.math.radians(-4.0)
    summary = motion.pose_error_summary(positions, motion.default_targets(), 2.0)
    assert 'left: 1/22 outside 2 deg, worst=left_index_DIP error=3.500 deg' in summary
    assert 'right: 1/22 outside 2 deg, worst=right_thumb_CMC_AA error=4.000 deg actual=-4.000 deg' in summary
