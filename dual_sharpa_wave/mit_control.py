"""Read and validate saved MIT settings without ever writing device parameters."""

import json
import math
from numbers import Real

from .hand_interface import BackendError


FS_JOINT_INDICES = (0, 1, 5, 6, 9, 10, 13, 14, 18, 19)


def _read(hand, names):
    status, payload = hand.get_parameter(names)
    if getattr(status, 'code', None) != 0:
        raise BackendError(f'get_parameter({names}) failed: {getattr(status, "message", status)}')
    try:
        result = json.loads(payload)
    except (ValueError, TypeError) as error:
        raise BackendError(f'Invalid MIT parameter JSON: {error}') from error
    if not isinstance(result, dict):
        raise BackendError('MIT parameter response must be a JSON object')
    return result


def _gain(value, name, lengths):
    values = value if isinstance(value, list) else [value]
    if isinstance(value, list) and len(values) not in lengths:
        raise BackendError(f'{name} must be a scalar or an array of length {lengths}')
    if any(isinstance(v, bool) or not isinstance(v, Real) or
           not math.isfinite(v) or v < 0 for v in values):
        raise BackendError(f'{name} must contain finite nonnegative gains')
    return value


def read_mit_settings(hand):
    """Accept scalar/22-joint gains and Pilot's 10- or 22-element FS arrays.

    Probe the current source key first, then the legacy key. Both must fail
    closed; no guessed gains or torque source may be substituted.
    """
    errors = []
    for key in ('force_feedback_source', 'torque_source'):
        try:
            params = _read(hand, [key])
        except BackendError as error:
            errors.append(str(error))
            continue
        if key not in params:
            errors.append(f'Missing {key}')
            continue
        # A present but unsupported source must not be masked by a legacy key.
        value = params[key]
        values = value if isinstance(value, list) else [value]
        if ((isinstance(value, list) and len(value) != 22) or
                any(isinstance(v, bool) or not isinstance(v, Real) or
                    v not in (0, 1) for v in values) or len(set(values)) != 1):
            raise BackendError(f'{key}: expected uniform current (0) or sensor (1) source')
        source = int(values[0])
        break
    else:
        raise BackendError('Cannot read MIT torque source: ' + '; '.join(errors))
    names = ['mit_kp', 'mit_kd']
    if source == 1:
        names += ['mit_kp_fs', 'mit_kd_fs']
    params = _read(hand, names)
    gains = {name: _gain(params.get(name), name,
                         (len(FS_JOINT_INDICES), 22) if name.endswith('_fs') else (22,))
             for name in names}
    return {'source_key': key, 'torque_source': source, **gains}
