import math

import pytest

from dual_sharpa_wave.hand_interface import BackendError
from dual_sharpa_wave.mock_hand import MockHand


def hand(mode='first_order', speed=1.0):
    result = MockHand(mode, speed)
    result.start()
    return result


def test_initial_state_and_lifecycle():
    backend = MockHand()
    with pytest.raises(BackendError):
        backend.get_joint_positions()
    backend.start()
    assert backend.get_joint_positions() == [0.0] * 22
    backend.stop()
    backend.stop()
    with pytest.raises(BackendError):
        backend.set_joint_positions([1.0] * 22)


def test_instant_and_copy_isolation():
    left, right = hand('instant'), hand('instant')
    target = [0.25] * 22
    left.set_joint_positions(target)
    target[0] = 99.0
    feedback = left.get_joint_positions()
    feedback[0] = 88.0
    assert left.get_joint_positions() == [0.25] * 22
    assert right.get_joint_positions() == [0.0] * 22


def test_rate_bound_convergence_and_substep_equivalence():
    a, b = hand(speed=0.4), hand(speed=0.4)
    target = [0.15, -0.12] + [0.0] * 20
    for backend in (a, b):
        backend.set_joint_positions(target)
        assert backend.get_joint_positions() == [0.0] * 22
    a.update(0.25)
    for _ in range(5):
        b.update(0.05)
    assert a.get_joint_positions()[:2] == pytest.approx([0.1, -0.1])
    assert a.get_joint_positions() == pytest.approx(b.get_joint_positions())
    a.update(1.0)
    assert a.get_joint_positions() == target
    a.update(100.0)
    assert a.get_joint_positions() == target


@pytest.mark.parametrize('bad', [[0.0]*21, [0.0]*23, [math.nan]*22, [math.inf]*22, [-math.inf]*22])
def test_invalid_command_does_not_replace_target(bad):
    backend = hand()
    backend.set_joint_positions([0.4] * 22)
    with pytest.raises(ValueError):
        backend.set_joint_positions(bad)
    backend.update(1.0)
    assert backend.get_joint_positions() == [0.4] * 22


@pytest.mark.parametrize('dt', [-0.1, math.nan, math.inf])
def test_invalid_dt(dt):
    with pytest.raises(ValueError):
        hand().update(dt)


@pytest.mark.parametrize('speed', [0.0, -1.0, math.nan, math.inf])
def test_invalid_speed(speed):
    with pytest.raises(ValueError):
        MockHand(max_velocity_rad_s=speed)


def test_invalid_mode():
    with pytest.raises(ValueError):
        MockHand(mode='random')
