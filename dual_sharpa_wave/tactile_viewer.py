"""OpenCV tactile viewer. Subscribes to ROS Images; never imports or connects to the SDK."""

import argparse
from collections import deque
from dataclasses import dataclass
import math
import os
import sys
import time

import cv2
import numpy as np

from .tactile import BLOCKS, FINGERS, from_image

WINDOW = 'Sharpa tactile | ROS subscriber'
TILE_W, IMAGE_H, ROW_H = 240, 130, 188
HAND_W, HEADER_H = TILE_W * 3 + 24, 72
BACKGROUND = (24, 26, 30)


def text(image, label, x, y, color=(220, 224, 230), scale=.45):
    cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def force_heatmap(array, maximum):
    if array.ndim != 3 or array.shape[2] != 3 or not np.isfinite(array).all():
        raise ValueError('expected finite HxWx3 force data')
    magnitude = np.linalg.norm(array.astype(np.float64), axis=2)
    levels = (np.clip(magnitude / maximum, 0, 1) * 255).astype(np.uint8)
    return cv2.applyColorMap(levels, cv2.COLORMAP_JET)


def preview(array):
    """Image preview only; float deform normalization does not imply physical units."""
    if array.dtype != np.uint8:
        lo, hi = float(array.min()), float(array.max())
        array = ((array - lo) / (hi - lo) * 255).astype(np.uint8) if hi > lo else np.zeros(array.shape, np.uint8)
    return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR) if array.ndim == 2 else array


@dataclass
class Sample:
    array: np.ndarray
    received: float
    stamp: tuple
    arrivals: deque


class DisplayData:
    def __init__(self):
        self.samples = {}
        self.errors = {}

    def receive(self, key, msg, now):
        try:
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            previous = self.samples.get(key)
            if previous is not None and stamp != (0, 0) and previous.stamp == stamp:
                return
            array = from_image(msg)
            if key[2] == 'DIST_FORCE' and (array.ndim != 3 or array.dtype != np.float32):
                raise ValueError('DIST_FORCE must be float32 HxWx3')
            arrivals = previous.arrivals if previous else deque(maxlen=120)
            arrivals.append(now)
            self.samples[key] = Sample(array, now, stamp, arrivals)
            self.errors.pop(key, None)
        except (ValueError, TypeError) as exc:
            self.errors[key] = str(exc)

    def choose(self, side, finger, blocks, now, stale_after):
        # Prefer a live format over stale data from a previous SDK output mode.
        candidates = [(block, self.samples.get((side, finger, block))) for block in blocks]
        for block, sample in candidates:
            if sample is not None and now - sample.received <= stale_after:
                return block, sample
        return next(((b, s) for b, s in candidates if s is not None), (blocks[0], None))


def render(data, *, now, force_max=1., stale_after=1.):
    canvas = np.full((HEADER_H + 5 * ROW_H + 38, HAND_W * 2, 3), BACKGROUND, np.uint8)
    groups = (('RAW',), ('DEFORM3_SE', 'DEFORM3', 'DEFORM'), ('DIST_FORCE',))
    for hand_index, side in enumerate(('left', 'right')):
        origin = hand_index * HAND_W + 12
        text(canvas, f'{side.upper()} HAND', origin, 26, scale=.65)
        for column, label in enumerate(('RAW', 'DEFORM preview', 'FORCE magnitude | SDK units')):
            text(canvas, label, origin + column * TILE_W, 53, scale=.42)
        for row, finger in enumerate(FINGERS):
            top = HEADER_H + row * ROW_H
            for column, blocks in enumerate(groups):
                x = origin + column * TILE_W
                block, sample = data.choose(side, finger, blocks, now, stale_after)
                label = f'{finger} / {block}'
                text(canvas, label, x, top + 13, scale=.40)
                tile = np.full((IMAGE_H, TILE_W - 8, 3), (37, 40, 45), np.uint8)
                error = data.errors.get((side, finger, block))
                if error:
                    status, color = 'INVALID DATA', (60, 80, 255)
                elif sample is None:
                    status, color = 'NO DATA', (150, 160, 170)
                else:
                    picture = force_heatmap(sample.array, force_max) if column == 2 else preview(sample.array)
                    height, width = picture.shape[:2]
                    scale = min(tile.shape[1] / width, IMAGE_H / height)
                    resized = cv2.resize(picture, (max(1, int(width * scale)), max(1, int(height * scale))),
                                         interpolation=cv2.INTER_NEAREST)
                    y0, x0 = (IMAGE_H - resized.shape[0]) // 2, (tile.shape[1] - resized.shape[1]) // 2
                    tile[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
                    age = max(0., now - sample.received)
                    recent = [t for t in sample.arrivals if now - t <= 2.]
                    hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.
                    stale = age > stale_after
                    status = f'{"STALE" if stale else "LIVE"} {hz:.1f} Hz  age {age:.1f}s'
                    color = (40, 130, 255) if stale else (130, 220, 150)
                    if stale:
                        tile = (tile * .25).astype(np.uint8)
                        text(tile, 'STALE', 75, 70, color, .7)
                if sample is None or error:
                    text(tile, status, 40, 70, color, .6)
                canvas[top + 20:top + 20 + IMAGE_H, x:x + TILE_W - 8] = tile
                text(canvas, status, x, top + 166, color, .38)
    bottom = canvas.shape[0] - 14
    text(canvas, 'q / Esc: close viewer | NO DATA may mean unsupported or not yet received', 12, bottom, scale=.42)
    ramp = np.tile(np.arange(256, dtype=np.uint8), (10, 1))
    canvas[-32:-22, -276:-20] = cv2.applyColorMap(ramp, cv2.COLORMAP_JET)
    text(canvas, f'Force: 0 ... {force_max:g} (fixed, SDK units)', canvas.shape[1] - 330, bottom, scale=.42)
    return canvas


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force-max', type=float, default=1., help='Fixed heatmap upper limit in SDK units')
    parser.add_argument('--stale-after', type=float, default=1., help='Seconds without a new image before STALE')
    args = parser.parse_args(argv)
    for name, value in vars(args).items():
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    return args


def main(argv=None):
    args = parse_args(argv)
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('OpenCV viewer needs a desktop display (DISPLAY or WAYLAND_DISPLAY).', file=sys.stderr)
        return 1
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    data = DisplayData()
    node = None
    rclpy.init(args=[])
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1488, 1050)
        node = Node('tactile_viewer')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in ('left', 'right'):
            for finger in FINGERS:
                for block in BLOCKS:
                    key = (side, finger, block)
                    node.create_subscription(
                        Image, f'/sharpa/{side}_hand/tactile/{finger}/{block.lower()}',
                        lambda msg, key=key: data.receive(key, msg, time.monotonic()), qos)
        node.get_logger().info('Tactile viewer: ROS subscriptions only. q/Esc closes this window.')
        while rclpy.ok():
            # Drain ROS callbacks for a bounded interval; all GUI calls stay on this thread.
            deadline = time.monotonic() + .015
            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.001)
            cv2.imshow(WINDOW, render(data, now=time.monotonic(), force_max=args.force_max,
                                     stale_after=args.stale_after))
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as exc:
        print(f'Tactile viewer failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    raise SystemExit(main())
