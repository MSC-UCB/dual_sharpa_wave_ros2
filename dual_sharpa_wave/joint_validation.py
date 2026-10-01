"""ROS-independent validation; invalid commands are rejected atomically."""

from collections.abc import Sequence
import math
from numbers import Real

from .joint_names import JOINT_COUNT

# Absolute radians: tolerate representation/degree-conversion noise at a limit.
JOINT_LIMIT_TOLERANCE_RAD = 1e-7


def validate_positions(positions: Sequence[float]) -> list[float]:
    if len(positions) != JOINT_COUNT:
        raise ValueError(f'position must contain exactly {JOINT_COUNT} values')
    if any(isinstance(q, bool) or not isinstance(q, Real) for q in positions):
        raise ValueError('position values must be real numbers')
    result = [float(q) for q in positions]
    if not all(math.isfinite(q) for q in result):
        raise ValueError('position values must be finite (no NaN or infinity)')
    return result


def bounded_joint_positions(positions, limits, names) -> list[float]:
    """Reject real overruns; snap only numerical boundary noise to exact limits."""
    result = validate_positions(positions)
    if len(limits) != JOINT_COUNT or len(names) != JOINT_COUNT:
        raise ValueError('limits and names must contain exactly 22 values')
    for index, (name, value, (lo, hi)) in enumerate(zip(names, result, limits)):
        if value < lo - JOINT_LIMIT_TOLERANCE_RAD or value > hi + JOINT_LIMIT_TOLERANCE_RAD:
            raise ValueError(f'{name}: {value} rad outside [{lo}, {hi}]')
        result[index] = min(hi, max(lo, value))
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
