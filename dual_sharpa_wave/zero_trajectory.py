"""Shared quintic zero-pose trajectory, using unmodified measured start positions."""

import math


def zero_duration(positions, minimum, speed, acceleration):
    if any(not math.isfinite(v) or v <= 0 for v in (minimum, speed, acceleration)):
        raise ValueError('Duration, speed and acceleration must be finite and positive')
    distance = max(abs(v) for v in positions)
    duration = max(minimum, 1.875 * distance / speed,
                   math.sqrt(10 / math.sqrt(3) * distance / acceleration))
    if not math.isfinite(duration):
        raise ValueError('Computed zero-pose duration must be finite')
    return duration


def zero_sample(positions, elapsed, duration):
    if not math.isfinite(elapsed):
        raise ValueError('Elapsed time must be finite')
    u = min(1.0, max(0.0, elapsed / duration))
    if u == 1:
        return [0.0] * len(positions)
    blend = u**3 * (10 + u * (-15 + 6 * u))
    return [(1 - blend) * v for v in positions]
