"""ROS-independent limits and paired, sequential joint test trajectories."""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .joint_names import JOINT_SUFFIXES, joint_names
from .joint_validation import validate_positions

SIDES = ('left', 'right')


def positive(value, name, *, allow_zero=False):
    if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f'{name} must be finite and {"nonnegative" if allow_zero else "positive"}')
    return value


def axis_indices(text):
    axes = tuple(part.strip() for part in text.split(','))
    if not axes or len(set(axes)) != len(axes) or any(a not in JOINT_SUFFIXES for a in axes):
        raise ValueError('axes must be distinct joint suffixes: ' + ','.join(JOINT_SUFFIXES))
    return tuple(JOINT_SUFFIXES.index(a) for a in axes)


def load_limits(share: Path):
    limits = {}
    for side in SIDES:
        root = ET.parse(share / 'urdf' / f'{side}_sharpa_wave_with_flange.urdf').getroot()
        joints = {j.get('name'): j for j in root.findall('joint')}
        bounds = []
        for name in joint_names(side):
            limit = joints[name].find('limit')
            lo, hi = float(limit.get('lower')), float(limit.get('upper'))
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError(f'invalid model limits: {name}')
            bounds.append((lo, hi))
        limits[side] = tuple(bounds)
    return limits


def bounded_positions(side, positions, limits):
    result = validate_positions(positions)
    for name, value, (lo, hi) in zip(joint_names(side), result, limits[side]):
        if not lo <= value <= hi:
            raise ValueError(f'{name}: {value:.5f} rad outside [{lo}, {hi}]')
    return result


@dataclass(frozen=True)
class Waveform:
    kind: str
    amplitude: float = 0.1
    frequency: float = 0.25
    cycles: int = 2
    base_seconds: float = 1.0
    high_seconds: float = 1.0

    def __post_init__(self):
        if self.kind not in ('sine', 'step'):
            raise ValueError('waveform must be sine or step')
        if not math.isfinite(self.amplitude) or self.amplitude == 0:
            raise ValueError('amplitude/step-size must be finite and nonzero')
        if isinstance(self.cycles, bool) or not isinstance(self.cycles, int) or self.cycles < 1:
            raise ValueError('cycles must be a positive integer')
        positive(self.frequency, 'frequency')
        positive(self.base_seconds, 'base-seconds')
        positive(self.high_seconds, 'high-seconds')
        positive(self.duration, 'trajectory duration')

    @property
    def duration(self):
        if self.kind == 'sine':
            return self.cycles / self.frequency
        return self.cycles * (2 * self.base_seconds + self.high_seconds)

    @property
    def excursion(self):
        if self.kind == 'sine':
            return -abs(self.amplitude), abs(self.amplitude)
        return min(0, self.amplitude), max(0, self.amplitude)

    def offset(self, elapsed):
        positive(elapsed, 'elapsed', allow_zero=True)
        if elapsed >= self.duration:
            return 0.0
        if self.kind == 'sine':
            return self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed)
        phase = elapsed % (2 * self.base_seconds + self.high_seconds)
        return self.amplitude if self.base_seconds <= phase < self.base_seconds + self.high_seconds else 0.0


class WaveSequence:
    """One same-name axis pair at a time; settle at baseline before advancing."""

    def __init__(self, baseline, limits, axes, waveform, *, repeat=1, gap=0.25,
                 settle_timeout=10.0, tolerance=0.02):
        if not axes or len(set(axes)) != len(axes) or any(i not in range(22) for i in axes):
            raise ValueError('invalid or duplicate axes')
        if isinstance(repeat, bool) or not isinstance(repeat, int) or repeat < 1:
            raise ValueError('repeat must be a positive integer')
        self.baseline = {s: bounded_positions(s, baseline[s], limits) for s in SIDES}
        self.waveform = waveform
        self.axes = tuple(axes) * repeat
        self.gap = positive(gap, 'gap', allow_zero=True)
        self.settle_timeout = positive(settle_timeout, 'settle-timeout')
        self.tolerance = positive(tolerance, 'tolerance')
        if self.gap >= self.settle_timeout:
            raise ValueError('gap must be less than settle-timeout')
        low, high = waveform.excursion
        for side in SIDES:
            for axis in axes:
                lo, hi = limits[side][axis]
                q = self.baseline[side][axis]
                if not lo <= q + low <= q + high <= hi:
                    raise ValueError(f'{joint_names(side)[axis]} trajectory exceeds [{lo}, {hi}] rad')
        self.index = 0
        self.started_at = 0.0
        self.settling_at = None
        self.done = False

    @property
    def axis_name(self):
        return JOINT_SUFFIXES[self.axes[self.index]]

    def sample(self, elapsed, actual):
        positive(elapsed, 'elapsed', allow_zero=True)
        if self.done:
            return None
        target = {s: q.copy() for s, q in self.baseline.items()}
        if self.settling_at is None:
            local_time = elapsed - self.started_at
            if local_time < self.waveform.duration:
                offset = self.waveform.offset(local_time)
                for side in SIDES:
                    target[side][self.axes[self.index]] += offset
                return target
            # Always publish the exact baseline at least once before checking arrival.
            self.settling_at = elapsed
            return target
        settled = all(
            abs(q - ref) <= self.tolerance
            for side in SIDES
            for q, ref in zip(validate_positions(actual[side]), self.baseline[side])
        )
        waiting = elapsed - self.settling_at
        if waiting >= self.gap and settled:
            if self.index == len(self.axes) - 1:
                self.done = True
                return None
            self.index += 1
            self.started_at = elapsed
            self.settling_at = None
        elif waiting >= self.settle_timeout:
            raise TimeoutError(f'{self.axis_name}: feedback did not return to baseline')
        return target
