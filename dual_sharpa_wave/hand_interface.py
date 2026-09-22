"""Backend boundary: canonical joint order and radians in both directions."""

from abc import ABC, abstractmethod
from collections.abc import Sequence

from .joint_names import JOINT_COUNT


class BackendError(RuntimeError):
    """A backend operation failed; no fabricated feedback may be substituted."""


class HandInterface(ABC):
    joint_count = JOINT_COUNT

    @abstractmethod
    def start(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def set_joint_positions(self, positions: Sequence[float]) -> None:
        """Accept a target, without asserting that it has been reached."""

    @abstractmethod
    def get_joint_positions(self) -> list[float]:
        """Read current position, never merely echo the last command."""

    def update(self, dt_sec: float) -> None:
        """Advance a simulation if needed; hardware may keep this no-op."""

    def on_command_timeout(self) -> None:
        """Called once after an accepted command stream expires; mock retains target."""
