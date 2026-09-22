"""ROS-independent validation; invalid commands are rejected atomically."""

from collections.abc import Sequence
import math
from numbers import Real

from .joint_names import JOINT_COUNT


def validate_positions(positions: Sequence[float]) -> list[float]:
    if len(positions) != JOINT_COUNT:
        raise ValueError(f'position must contain exactly {JOINT_COUNT} values')
    if any(isinstance(q, bool) or not isinstance(q, Real) for q in positions):
        raise ValueError('position values must be real numbers')
    result = [float(q) for q in positions]
    if not all(math.isfinite(q) for q in result):
        raise ValueError('position values must be finite (no NaN or infinity)')
    return result


def canonical_positions(
    names: Sequence[str], positions: Sequence[float], canonical: Sequence[str],
) -> list[float]:
    values = validate_positions(positions)
    if len(names) == 0:
        return values
    if len(names) != JOINT_COUNT or len(set(names)) != JOINT_COUNT:
        raise ValueError('name must contain 22 distinct canonical joint names')
    if set(names) != set(canonical):
        raise ValueError('name contains unknown or missing canonical joints')
    by_name = dict(zip(names, values))
    return [by_name[name] for name in canonical]
