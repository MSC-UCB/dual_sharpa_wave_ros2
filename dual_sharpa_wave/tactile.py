"""Tactile frame validation/cache and typed ROS Image transport, without SDK imports."""

from dataclasses import dataclass
import math
import numbers
import threading
import time
from types import MappingProxyType
import uuid

import numpy as np

from .tactile_contract import INVALID, immutable_array, raw_block

FINGERS = ('pinky', 'ring', 'middle', 'index', 'thumb')
BLOCKS = ('RAW', 'DEFORM', 'DEFORM3', 'DEFORM3_SE', 'DIST_FORCE')
ENCODINGS = {'8UC1': (np.dtype('u1'), 1), '8UC3': (np.dtype('u1'), 3),
             '32FC1': (np.dtype('f4'), 1), '32FC3': (np.dtype('f4'), 3)}


def channels(side):
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    return range(5, 10) if side == 'left' else range(5)


def f6_array(value, shape=None):
    """Copy six finite SDK components without assigning physical units/axes."""
    array = np.asarray(value)
    if array.size != 6 or array.dtype.kind not in 'fi' or not np.isfinite(array).all():
        raise ValueError('F6 requires six finite numeric components')
    if shape is not None and len(shape):
        dims = tuple(int(n) for n in shape)
        if any(n <= 0 for n in dims) or math.prod(dims) != 6:
            raise ValueError('F6 shape does not match data')
    return np.array(array, dtype=np.float64, copy=True).reshape(6)


def image_array(value, shape=None):
    """Validate an SDK image block and take an owned, native-endian copy."""
    array = np.asarray(value)
    if array.size == 0 or array.size > 4_000_000:
        raise ValueError('empty or oversized block')
    if shape is not None and len(shape):
        dims = tuple(int(n) for n in shape)
        if any(n <= 0 for n in dims) or math.prod(dims) != array.size:
            raise ValueError('shape does not match data')
        array = array.reshape(dims)
    # Actual SDK RAW uses [1, 240, 320]; color images use HxWx3.
    if array.ndim == 3 and array.shape[0] == 1 and array.shape[-1] not in (1, 3):
        array = array[0]
    # Bindings may include a leading batch dimension of one.
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.ndim not in (2, 3) or (array.ndim == 3 and array.shape[2] != 3):
        raise ValueError(f'unsupported image shape {array.shape}')
    if array.dtype.kind == 'u' and array.dtype.itemsize == 1:
        dtype = np.uint8
    elif array.dtype.kind == 'f' and array.dtype.itemsize in (4, 8):
        dtype = np.float32
    else:
        raise ValueError(f'unsupported dtype {array.dtype}')
    if not np.isfinite(array).all():
        raise ValueError('nonfinite image data')
    with np.errstate(over='ignore', invalid='ignore'):
        result = np.array(array, dtype=dtype, order='C', copy=True)
    if not np.isfinite(result).all():
        raise ValueError('image values exceed float32 range')
    return result


@dataclass(frozen=True)
class Frame:
    channel: int
    stamp_ns: int
    sdk_ts: float
    frame_id: object
    blocks: dict
    errors: dict
    f6: object = None
    side: str = ''
    finger: str = ''
    receive_monotonic_ns: int = 0
    session_id: str = ''
    receipt_sequence: int = 0
    overwritten: int = 0
    duplicates: int = 0
    identity_warnings: tuple = ()
    raw_blocks: object = None
    frame_id_valid: bool = False
    frame_id_raw: str = ''
    sdk_ts_valid: bool = False
    sdk_ts_raw: str = ''

    @property
    def stamp(self):
        # Return a fresh ROS object, keeping the cache record immutable.
        from builtin_interfaces.msg import Time
        sec, nanosec = divmod(self.stamp_ns, 1_000_000_000)
        return Time(sec=sec, nanosec=nanosec)


class TactileCache:
    """Bounded latest-frame cache. Callbacks only stamp/copy; never publish or draw."""

    def __init__(self, side, stamp_clock=time.time_ns, monotonic_clock=time.monotonic_ns):
        self.allowed = channels(side)
        self.side = side
        self.stamp_clock = stamp_clock
        self.monotonic_clock = monotonic_clock
        self.lock = threading.Lock()
        self.receive_lock = threading.Lock()
        self.session_id = str(uuid.uuid4())
        self.pending = {}
        self.identities = {}
        self.sequences = {}
        self.overwritten = {}
        self.duplicates = {}
        self.error = None
        self.closed = False

    def receive(self, payload):
        # Serialize callbacks so a slow older copy cannot replace a newer record.
        with self.receive_lock:
            self._receive(payload)

    def _receive(self, payload):
        try:
            stamp = self.stamp_clock()
            stamp_ns = (stamp.sec * 1_000_000_000 + stamp.nanosec
                        if hasattr(stamp, 'sec') else int(stamp))
            received = self.monotonic_clock()
            if not isinstance(payload, dict):
                raise ValueError('frame must be a dict')
            channel = payload['channel']
            if isinstance(channel, bool) or not isinstance(channel, int) or channel not in self.allowed:
                raise ValueError(f'unexpected channel {channel!r}')
            blocks, errors = {}, {}
            raw_ts = payload.get('ts')
            try:
                ts = float(raw_ts)
                ts_valid = (not isinstance(raw_ts, (bool, np.bool_)) and
                            isinstance(raw_ts, numbers.Real) and math.isfinite(ts))
            except (ValueError, TypeError, OverflowError):
                ts, ts_valid = 0.0, False
            if not ts_valid:
                ts = 0.0
            if not ts_valid:
                errors['ts'] = 'missing or invalid SDK timestamp; see sdk_timestamp_repr'
            frame_id = payload.get('frame_id')
            id_valid = (not isinstance(frame_id, (bool, np.bool_)) and
                        isinstance(frame_id, numbers.Integral) and
                        -(2**31) <= frame_id < 2**31)
            if not id_valid:
                errors['frame_id'] = 'missing or invalid signed int32 SDK ID; see sdk_frame_id_raw'
            content = payload.get('content', {})
            shapes = payload.get('shape', {})
            if not isinstance(content, dict):
                errors['content'] = 'content must be a dict'
                content = {}
            if not isinstance(shapes, dict):
                errors['shape'] = 'shape must be a dict'
                # Mark each present raw block invalid; do not infer a trustworthy shape.
                shapes = {name: 'invalid shape container' for name in content}
            raw_blocks = {name: raw_block(name, content, shapes)
                          for name in ('F6', 'CONTACT_POINT', 'DIST_FORCE')}
            for name, block in raw_blocks.items():
                if block.status == INVALID:
                    errors[name] = block.error
            for name in BLOCKS:
                source = name
                if name == 'DEFORM' and content.get(name) is None:
                    source = 'DEFORM_JPG'
                value = content.get(source)
                try:
                    if value is None or np.asarray(value).size == 0:
                        continue
                    array = image_array(value, shapes.get(source))
                    if name == 'DIST_FORCE' and (array.ndim != 3 or array.dtype != np.float32):
                        raise ValueError('DIST_FORCE requires float32 HxWx3')
                    blocks[name] = immutable_array(array)
                except (ValueError, TypeError) as exc:
                    errors.setdefault(name, str(exc))
            f6 = None
            if content.get('F6') is not None:
                try:
                    f6 = immutable_array(f6_array(content['F6'], shapes.get('F6')))
                except (ValueError, TypeError) as exc:
                    errors['F6'] = str(exc)
            identity = (frame_id, ts)
            with self.lock:
                if self.closed:
                    return
                sequence = self.sequences.get(channel, 0) + 1
                self.sequences[channel] = sequence
                previous = self.identities.get(channel)
                if id_valid and ts_valid and identity == previous:
                    self.duplicates[channel] = self.duplicates.get(channel, 0) + 1
                    return
                warnings = []
                if previous is not None and id_valid and ts_valid:
                    if frame_id < previous[0]:
                        warnings.append('sdk_frame_id_regressed_restart_wrap_or_reorder_unknown')
                    elif frame_id == previous[0]:
                        warnings.append('sdk_frame_id_repeated_with_changed_timestamp')
                    elif frame_id > previous[0] + 1:
                        warnings.append('sdk_frame_id_forward_gap')
                    if ts < previous[1]:
                        warnings.append('sdk_timestamp_regressed')
                    elif ts == previous[1]:
                        warnings.append('sdk_timestamp_repeated')
                if id_valid and ts_valid:
                    self.identities[channel] = identity
                if channel in self.pending:
                    self.overwritten[channel] = self.overwritten.get(channel, 0) + 1
                self.pending[channel] = Frame(
                    channel, stamp_ns, ts, int(frame_id) if id_valid else None,
                    MappingProxyType(blocks), MappingProxyType(errors), f6,
                    self.side, FINGERS[channel % 5], received, self.session_id,
                    sequence, self.overwritten.get(channel, 0),
                    self.duplicates.get(channel, 0), tuple(warnings),
                    MappingProxyType(raw_blocks), id_valid, repr(frame_id),
                    ts_valid, repr(raw_ts))
        except Exception as exc:
            with self.lock:
                if not self.closed:
                    self.error = str(exc)

    def drain(self):
        with self.lock:
            frames, error = list(self.pending.values()), self.error
            self.pending, self.error = {}, None
        return frames, error

    def close(self):
        with self.lock:
            self.closed = True
            self.pending.clear()


def to_image(array, stamp):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.stamp = stamp
    msg.height, msg.width = array.shape[:2]
    count = 1 if array.ndim == 2 else array.shape[2]
    msg.encoding = ('8UC' if array.dtype == np.uint8 else '32FC') + str(count)
    msg.is_bigendian = False
    wire = np.ascontiguousarray(array, dtype=array.dtype.newbyteorder('<'))
    msg.step = msg.width * count * wire.dtype.itemsize
    msg.data = wire.tobytes()
    return msg


def from_image(msg):
    if msg.encoding not in ENCODINGS:
        raise ValueError(f'unsupported encoding {msg.encoding}')
    dtype, count = ENCODINGS[msg.encoding]
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    if msg.width <= 0 or msg.height <= 0 or msg.width * msg.height * count > 4_000_000:
        raise ValueError('invalid image dimensions')
    row_bytes = msg.width * count * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) != msg.height * msg.step:
        raise ValueError('invalid image stride or data length')
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    packed = rows[:, :row_bytes].copy().view(dtype)
    shape = (msg.height, msg.width) if count == 1 else (msg.height, msg.width, count)
    return image_array(packed.reshape(shape))
