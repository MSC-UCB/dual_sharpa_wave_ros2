import math

import pytest

from dual_sharpa_wave.hand_interface import BackendError
from dual_sharpa_wave.mock_hand import MockHand


def hand():
    result = MockHand()
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
    left, right = hand(), hand()
    target = [0.25] * 22
    left.set_joint_positions(target)
    target[0] = 99.0
    feedback = left.get_joint_positions()
    feedback[0] = 88.0
    assert left.get_joint_positions() == [0.25] * 22
    assert right.get_joint_positions() == [0.0] * 22


def test_target_is_reached_without_rate_limit():
    backend = hand()
    target = [0.15, -0.12] + [0.0] * 20
    backend.set_joint_positions(target)
    assert backend.get_joint_positions() == target
    backend.set_joint_positions([-0.9, 0.8] + [0.0] * 20)
    assert backend.get_joint_positions()[:2] == [-0.9, 0.8]


@pytest.mark.parametrize('bad', [[0.0]*21, [0.0]*23, [math.nan]*22, [math.inf]*22, [-math.inf]*22])
def test_invalid_command_does_not_replace_target(bad):
    backend = hand()
    backend.set_joint_positions([0.4] * 22)
    with pytest.raises(ValueError):
        backend.set_joint_positions(bad)
    assert backend.get_joint_positions() == [0.4] * 22
