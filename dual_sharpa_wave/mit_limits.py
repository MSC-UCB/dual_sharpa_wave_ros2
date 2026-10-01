"""MIT command clipping and measured-limit latch; no SDK/ROS operations."""

import math

from .joint_validation import validate_positions


class RecoveryDiverged(RuntimeError):
    pass


class MitLimits:
    def __init__(self, limits, names, tolerance_deg=0.5,
                 worsening_deg=0.5, worsening_sec=0.2):
        if (len(limits) != 22 or len(names) != 22 or
                any(not math.isfinite(lo) or not math.isfinite(hi) or
                    not lo <= 0 <= hi or lo >= hi for lo, hi in limits)):
            raise ValueError('MIT needs 22 finite joint limits containing the zero pose')
        for v in (tolerance_deg, worsening_deg, worsening_sec):
            if isinstance(v, bool) or not math.isfinite(v) or v <= 0:
                raise ValueError('MIT limit/recovery tolerances must be finite and positive')
        self.limits, self.names = tuple(limits), tuple(names)
        self.tolerance = math.radians(tolerance_deg)
        self.worsening = math.radians(worsening_deg)
        self.worsening_sec = worsening_sec
        self.state, self.reason = 'normal', ''
        self._best = self._worse_since = self._last_target = None

    def clip(self, positions):
        return [min(hi, max(lo, q)) for q, (lo, hi) in
                zip(validate_positions(positions), self.limits)]

    def violations(self, positions):
        return [max(lo - q, 0.0, q - hi) for q, (lo, hi) in
                zip(validate_positions(positions), self.limits)]

    def within_tolerance(self, positions):
        return max(self.violations(positions)) <= self.tolerance + 1e-12

    def observe(self, positions, now):
        errors = self.violations(positions)
        if self.state == 'normal' and max(errors) > self.tolerance + 1e-12:
            worst = max(range(22), key=errors.__getitem__)
            self.lock(f'{self.names[worst]} measured {math.degrees(errors[worst]):.3f} deg '
                      'outside model; explicit default-pose recovery required')
        elif self.state == 'recovering':
            for i, error in enumerate(errors):
                self._best[i] = min(error, self._best[i])
                if error > self._best[i] + self.worsening:
                    if self._worse_since[i] is None:
                        self._worse_since[i] = now
                    if now - self._worse_since[i] >= self.worsening_sec:
                        raise RecoveryDiverged(
                            f'{self.names[i]} limit violation worsened by more than '
                            f'{math.degrees(self.worsening):g} deg for {self.worsening_sec:g} s')
                else:
                    self._worse_since[i] = None

    def lock(self, reason):
        self.state, self.reason = 'limit_locked', reason
        self._last_target = None

    def begin(self, current):
        if self.state not in ('normal', 'limit_locked'):
            raise RuntimeError(f'Cannot recover while {self.state}')
        self._best = self.violations(current)
        self._worse_since = [None] * 22
        self._last_target = self.clip(current)
        self.state, self.reason = 'recovering', ''

    def recovery_target(self, positions):
        if self.state != 'recovering':
            raise RuntimeError('No active recovery')
        target = self.clip(positions)
        # Each command stays between the previous command and zero, no reversals.
        if any(q < min(0.0, p) - 1e-12 or q > max(0.0, p) + 1e-12
               for q, p in zip(target, self._last_target)):
            raise ValueError('Recovery targets must move monotonically toward zero')
        self._last_target = target
        return target

    def finish(self, current):
        if self.state != 'recovering' or not self.within_tolerance(current):
            raise RuntimeError('Recovery cannot finish outside measured limit tolerance')
        self.state, self.reason = 'normal', ''
        self._last_target = None
