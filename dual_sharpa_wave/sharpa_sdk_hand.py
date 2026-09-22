"""Hardware placeholder. Importing this module never loads the official SDK."""

from collections.abc import Sequence

from .hand_interface import HandInterface


class SharpaSdkHand(HandInterface):
    def __init__(
        self, serial_number: str, speed_coeff: float, current_coeff: float,
        interpolation: bool,
    ):
        if not serial_number.strip():
            raise ValueError('sharpa_sdk requires an explicit nonempty serial_number')
        self.serial_number = serial_number
        self.speed_coeff = speed_coeff
        self.current_coeff = current_coeff
        self.interpolation = interpolation

    def start(self) -> None:
        raise NotImplementedError(
            'Sharpa SDK hardware backend is not implemented or verified. '
            'No device connection was attempted. Use dual_sharpa_mock.launch.py.'
        )

    def stop(self) -> None:
        pass  # No SDK resources are acquired by this placeholder.

    def set_joint_positions(self, positions: Sequence[float]) -> None:
        raise NotImplementedError('Sharpa SDK hardware backend is not implemented')

    def get_joint_positions(self) -> list[float]:
        raise NotImplementedError('Sharpa SDK hardware backend is not implemented')
