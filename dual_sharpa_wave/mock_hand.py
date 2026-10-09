"""Deterministic 22-joint mock, independent of ROS and the Sharpa SDK."""

from collections.abc import Sequence

from .hand_interface import BackendError, HandInterface
from .joint_validation import validate_positions


class MockHand(HandInterface):
    def __init__(self):
        self._actual = [0.0] * self.joint_count
        self._started = False

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def _require_started(self) -> None:
        if not self._started:
            raise BackendError('MockHand is not started')

    def set_joint_positions(self, positions: Sequence[float]) -> None:
        self._require_started()
        self._actual = validate_positions(positions)

    def get_joint_positions(self) -> list[float]:
        self._require_started()
        return self._actual.copy()
