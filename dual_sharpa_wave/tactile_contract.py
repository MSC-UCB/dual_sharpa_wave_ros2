"""Version 1 raw tactile contract; no SDK import, physical conversion or control."""

from array import array
from dataclasses import dataclass
import json
import math
import numbers
import time

import numpy as np

MISSING, VALID, EMPTY, INVALID = range(4)
MAX_VALUES = 262144


def immutable_array(value):
    """An owned bytes backing prevents consumers from re-enabling writes."""
    return np.frombuffer(value.tobytes(order='C'), dtype=value.dtype).reshape(value.shape)


@dataclass(frozen=True)
class RawBlock:
    status: int = MISSING
    shape: tuple = ()
    shape_declared: bool = False
    declared_shape_json: str = ''
    dtype: str = ''
    values: object = None
    error: str = ''


def raw_block(name, content, shapes):
    """Preserve declared shape and all columns; isolate any malformed block."""
    declared = name in shapes
    shape_json, dtype, dims = '', '', ()
    try:
        shape = shapes.get(name)
        if declared:
            # JSON keeps malformed numeric declarations available for diagnosis.
            shape_json = json.dumps(shape, default=lambda v: v.tolist(), allow_nan=True)
            if len(shape_json) > 1024:
                shape_json = shape_json[:1024]
                raise ValueError('shape declaration exceeds 1024 characters (truncated)')
        if content.get(name) is None:
            return RawBlock(MISSING, (), declared, shape_json)
        value = np.asarray(content[name])
        dtype = str(value.dtype)
        if value.size > MAX_VALUES:
            raise ValueError(f'payload exceeds {MAX_VALUES} values')
        if declared:
            if not isinstance(shape, (list, tuple, np.ndarray)):
                raise ValueError('shape must be an integer sequence')
            if len(shape) > 8 or any(isinstance(n, (bool, np.bool_)) or
                    not isinstance(n, numbers.Integral) or not 0 <= n <= MAX_VALUES
                    for n in shape):
                raise ValueError('shape requires at most eight bounded nonnegative integers')
            dims = tuple(int(n) for n in shape)
        else:
            dims = value.shape
        if len(dims) > 8 or math.prod(dims) != value.size:
            raise ValueError('shape does not match data')
        if value.dtype.kind not in 'fiu' or not np.isfinite(value).all():
            raise ValueError('nonfinite or nonnumeric data')
        if value.dtype.kind in 'iu' and np.any(np.abs(value.astype(np.float64)) > 2**53):
            raise ValueError('integer cannot be represented exactly as float64')
        if value.dtype.kind == 'f' and value.dtype.itemsize > 8:
            raise ValueError('floating dtype wider than float64 is unsupported')
        if name == 'F6' and value.size != 6:
            raise ValueError('F6 requires exactly six values')
        if name == 'CONTACT_POINT' and not dims:
            raise ValueError('CONTACT_POINT must have a non-scalar shape')
        if name == 'DIST_FORCE':
            # Only the documented/observed grid is accepted by contract v1.
            # Legacy image validation deliberately remains independent.
            if dims != (60, 60, 3) or value.dtype.kind != 'f':
                raise ValueError('DIST_FORCE v1 requires floating (60, 60, 3)')
            if np.any(np.abs(value) > np.finfo(np.float32).max):
                raise ValueError('DIST_FORCE values exceed float32 range')
        values = immutable_array(np.asarray(value, dtype=np.float64).reshape(dims))
        return RawBlock(EMPTY if value.size == 0 else VALID, dims, declared,
                        shape_json, dtype, values)
    except Exception as exc:
        return RawBlock(INVALID, (), declared, shape_json, dtype, error=str(exc))


def message_type():
    try:
        from dual_sharpa_wave.msg import TactileFrame
        from rclpy.type_support import check_for_type_support
        check_for_type_support(TactileFrame)
        return TactileFrame
    except Exception as exc:
        raise RuntimeError('TactileFrame is unavailable: rebuild dual_sharpa_wave with '
                           'colcon build and source its install/setup.bash') from exc


def block_message(block):
    from dual_sharpa_wave.msg import TactileBlock
    msg = TactileBlock(status=block.status, shape=list(block.shape),
                       shape_declared=block.shape_declared,
                       declared_shape_json=block.declared_shape_json,
                       dtype=block.dtype, error=block.error)
    if block.values is not None:
        values = array('d')
        values.frombytes(block.values.tobytes(order='C'))
        msg.raw_values = values
    return msg


def to_frame_message(frame, publish_stamp, monotonic_ns=None):
    """One atomic message, solely from one cache record; no topic reconstruction."""
    msg = message_type()()
    msg.schema_version = 1
    msg.header.stamp = frame.stamp
    msg.source = 'sharpa_sdk'
    msg.side, msg.finger, msg.channel = frame.side, frame.finger, frame.channel
    msg.session_id, msg.receipt_sequence = frame.session_id, frame.receipt_sequence
    msg.sdk_frame_id_valid = frame.frame_id_valid
    msg.sdk_frame_id = frame.frame_id if frame.frame_id_valid else 0
    msg.sdk_frame_id_raw = frame.frame_id_raw
    msg.sdk_timestamp_valid = frame.sdk_ts_valid
    msg.sdk_timestamp_raw = frame.sdk_ts if frame.sdk_ts_valid else 0.0
    msg.sdk_timestamp_repr = frame.sdk_ts_raw
    msg.sdk_timestamp_unit = msg.sdk_timestamp_epoch = 'unknown'
    msg.receive_monotonic_ns = frame.receive_monotonic_ns
    msg.overwritten_frames, msg.duplicate_callbacks = frame.overwritten, frame.duplicates
    msg.identity_warnings = list(frame.identity_warnings)
    msg.f6 = block_message(frame.raw_blocks['F6'])
    msg.contact_points = block_message(frame.raw_blocks['CONTACT_POINT'])
    msg.dist_force = block_message(frame.raw_blocks['DIST_FORCE'])
    msg.validation_errors = [f'{name}: {error}' for name, error in frame.errors.items()]
    msg.publish_stamp = publish_stamp() if callable(publish_stamp) else publish_stamp
    now = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
    msg.receive_to_publish_sec = max(0, now - frame.receive_monotonic_ns) / 1e9
    return msg
