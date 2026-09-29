"""Tactile frame validation/cache and typed ROS Image transport, without SDK imports."""

from dataclasses import dataclass
import math
import threading
import time

import numpy as np

FINGERS = ('pinky', 'ring', 'middle', 'index', 'thumb')
BLOCKS = ('RAW', 'DEFORM', 'DEFORM3', 'DEFORM3_SE', 'DIST_FORCE')
ENCODINGS = {'8UC1': (np.dtype('u1'), 1), '8UC3': (np.dtype('u1'), 3),
             '32FC1': (np.dtype('f4'), 1), '32FC3': (np.dtype('f4'), 3)}


def channels(side):
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    return range(5, 10) if side == 'left' else range(5)


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


@dataclass
class Frame:
    channel: int
    stamp: object
    sdk_ts: float
    frame_id: object
    blocks: dict
    errors: dict


class TactileCache:
    """Bounded latest-frame cache. Callbacks only stamp/copy; never publish or draw."""

    def __init__(self, side, stamp_clock=time.time_ns):
        self.allowed = channels(side)
        self.stamp_clock = stamp_clock
        self.lock = threading.Lock()
        self.pending = {}
        self.identities = {}
        self.error = None
        self.closed = False

    def receive(self, payload):
        try:
            stamp = self.stamp_clock()
            if not isinstance(payload, dict):
                raise ValueError('frame must be a dict')
            channel = payload['channel']
            if isinstance(channel, bool) or not isinstance(channel, int) or channel not in self.allowed:
                raise ValueError(f'unexpected channel {channel!r}')
            ts = float(payload['ts'])
            if not math.isfinite(ts):
                raise ValueError('nonfinite SDK timestamp')
            content = payload.get('content') or {}
            shapes = payload.get('shape') or {}
            blocks, errors = {}, {}
            for name in BLOCKS:
                source = name
                if name == 'DEFORM' and content.get(name) is None:
                    source = 'DEFORM_JPG'
                value = content.get(source)
                if value is None or np.asarray(value).size == 0:
                    continue
                try:
                    array = image_array(value, shapes.get(source))
                    if name == 'DIST_FORCE' and (array.ndim != 3 or array.dtype != np.float32):
                        raise ValueError('DIST_FORCE requires float32 HxWx3')
                    blocks[name] = array
                except (ValueError, TypeError) as exc:
                    errors[name] = str(exc)
            frame_id = payload.get('frame_id')
            identity = (frame_id, ts)
            with self.lock:
                if not self.closed and identity != self.identities.get(channel):
                    self.identities[channel] = identity
                    self.pending[channel] = Frame(channel, stamp, ts, frame_id, blocks, errors)
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
