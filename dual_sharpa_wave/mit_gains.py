"""Two-step gain transition, allowing for the SDK's ~300 ms writes."""

import math


PERIOD = 0.2
# 200 ms initial wait + two ~300 ms writes fits approximately within one second.
STEPS = 2
# Fixed tuning baseline, shared by startup and GUI (both hands, canonical order).
BASE_KP = (20., 20., 8., 25., 4., 5., 20., 4., 1., 5., 20., 4., 1.,
           5., 20., 4., 1., 2., 5., 20., 4., 1.)
BASE_KD = (.5, .5, .2, .25, .1, .2, .25, .1, .01, .2, .25, .1, .01,
           .2, .25, .1, .01, .01, .2, .25, .1, .01)


def gains(values):
    values = list(values)
    if len(values) != 22 or any(
            isinstance(v, bool) or not math.isfinite(v) or v < 0 for v in values):
        raise ValueError('Expected 22 finite, nonnegative gains')
    return values


def scaled_gains(kp_ratio, kd_ratio):
    for name, ratio in (('Kp', kp_ratio), ('Kd', kd_ratio)):
        if isinstance(ratio, bool) or not math.isfinite(ratio) or ratio < 0:
            raise ValueError(f'{name} ratio must be finite and nonnegative')
    return gains(v * kp_ratio for v in BASE_KP), gains(v * kd_ratio for v in BASE_KD)


class GainTransition:
    def __init__(self, backend):
        self.backend = backend
        self.kp, self.kd = [], []
        self.remaining = 0
        self.message = 'Read gains to begin'
        self.failed = False

    def read(self):
        try:
            self.kp, self.kd = self.backend.read_mit_gains()
        except Exception:
            self.remaining = 0
            self.kp, self.kd = [], []
            self.failed = True
            self.message = 'Gain read failed; transition stopped'
            raise
        self.failed = False
        self.message = 'Device gains read'

    def apply(self, kp, kd):
        target = gains(kp), gains(kd)
        self.backend.require_gain_write()
        self.read()
        self.target = target
        self.remaining = STEPS if target != (self.kp, self.kd) else 0
        self.message = 'Transition accepted' if self.remaining else 'Already at target'

    def cancel(self):
        self.remaining = 0
        self.message = 'Transition stopped; gains retained'

    def step(self):
        if not self.remaining:
            return
        try:
            kp, kd = ([v + (t - v) / self.remaining for v, t in zip(current, target)]
                      for current, target in zip((self.kp, self.kd), self.target))
            if self.remaining == 1:
                kp, kd = self.target
            # The backend writes both arrays and returns verified device readback.
            self.kp, self.kd = self.backend.write_mit_gains(kp, kd)
            self.remaining -= 1
            self.message = f'{STEPS - self.remaining}/{STEPS} steps verified'
        except Exception as error:
            self.remaining = 0
            self.kp, self.kd = [], []  # Do not display stale values after an ambiguous write.
            self.failed = True
            self.message = f'Gain update stopped: {error}. Read gains before retrying.'
