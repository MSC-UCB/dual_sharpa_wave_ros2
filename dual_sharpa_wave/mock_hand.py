"""Deterministic 22-joint mock, independent of ROS and the Sharpa SDK."""

from collections.abc import Sequence
import math

from .hand_interface import BackendError, HandInterface
from .joint_validation import validate_positions


class MockHand(HandInterface):
    def __init__(self, mode: str = 'first_order', max_velocity_rad_s: float = 1.0):
        if mode not in ('instant', 'first_order'):
            raise ValueError("mock_mode must be 'instant' or 'first_order'")
        if not math.isfinite(max_velocity_rad_s) or max_velocity_rad_s <= 0:
            raise ValueError('mock_max_velocity_rad_s must be finite and positive')
        self.mode = mode
        self.max_velocity_rad_s = max_velocity_rad_s
        self._actual = [0.0] * self.joint_count
        self._target = [0.0] * self.joint_count
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
        target = validate_positions(positions)
        self._target = target
        if self.mode == 'instant':
            self._actual = target.copy()

    def get_joint_positions(self) -> list[float]:
        self._require_started()
        return self._actual.copy()

    def update(self, dt_sec: float) -> None:
        self._require_started()
        if not math.isfinite(dt_sec) or dt_sec < 0:
            raise ValueError('dt_sec must be finite and nonnegative')
        max_step = self.max_velocity_rad_s * dt_sec
        for i, target in enumerate(self._target):
            actual = self._actual[i]
            # Clip against target directly to avoid overflow in target - actual.
            if target > actual:
                self._actual[i] = min(target, actual + max_step)
            else:
                self._actual[i] = max(target, actual - max_step)
