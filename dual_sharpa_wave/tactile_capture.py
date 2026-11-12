"""Subscribe to frame topics and write bounded summaries; never access the SDK."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np

from .tactile import FINGERS
from .tactile_contract import MISSING, VALID, EMPTY, INVALID, message_type

STATUS_NAMES = {MISSING: 'missing', VALID: 'valid', EMPTY: 'empty', INVALID: 'invalid'}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class RunningStats:
    def __init__(self):
        self.count, self.total = 0, 0.0
        self.minimum = self.maximum = None

    def add(self, value):
        if not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def summary(self):
        return dict(count=self.count, min=self.minimum, max=self.maximum,
                    mean=self.total / self.count if self.count else None)


class FingerSummary:
    """Constant-memory statistics, except for the small set of block statuses."""

    def __init__(self, side, finger):
        self.side, self.finger = side, finger
        self.count = self.rejected = self.validation_error_frames = 0
        self.first = self.last = None
        self.previous = None
        self.session_changes = self.id_regressions = self.id_gap_events = 0
        self.sequence_gap_total = self.repeated_sequences = 0
        self.receive_to_publish = RunningStats()
        self.receive_to_subscriber = RunningStats()
        self.blocks = {name: {'statuses': Counter(), 'last_shape': [], 'last_dtype': ''}
                       for name in ('f6', 'contact_points', 'dist_force')}
        self.last_identity = {}
        self.last_errors = []
        self.last_f6_raw = []
        self.first_f6_raw = []
        self.f6_component_min = []
        self.f6_component_max = []

    def receive(self, msg, now, ros_now):
        expected_channel = FINGERS.index(self.finger) + (5 if self.side == 'left' else 0)
        if (msg.schema_version != 1 or msg.source != 'sharpa_sdk' or
                (msg.side, msg.finger, msg.channel) != (self.side, self.finger, expected_channel)):
            self.rejected += 1
            return
        self.count += 1
        self.first = now if self.first is None else self.first
        self.last = now
        identity = (msg.session_id, msg.receipt_sequence,
                    msg.sdk_frame_id if msg.sdk_frame_id_valid else None)
        if self.previous is not None:
            if identity[0] != self.previous[0]:
                self.session_changes += 1
            else:
                gap = identity[1] - self.previous[1]
                self.sequence_gap_total += max(0, gap - 1)
                self.repeated_sequences += int(gap <= 0)
                if identity[2] is not None and self.previous[2] is not None:
                    delta = identity[2] - self.previous[2]
                    self.id_regressions += int(delta < 0)
                    self.id_gap_events += int(delta > 1)
        self.previous = identity
        self.last_identity = dict(session_id=msg.session_id,
            receipt_sequence=msg.receipt_sequence, sdk_frame_id=identity[2],
            sdk_timestamp_raw=msg.sdk_timestamp_raw if msg.sdk_timestamp_valid else None,
            overwritten_frames=msg.overwritten_frames, duplicate_callbacks=msg.duplicate_callbacks,
            identity_warnings=list(msg.identity_warnings))
        self.last_errors = list(msg.validation_errors)
        self.validation_error_frames += bool(msg.validation_errors)
        self.receive_to_publish.add(msg.receive_to_publish_sec)
        self.receive_to_subscriber.add(ros_now - stamp_seconds(msg.header.stamp))
        for name, info in self.blocks.items():
            block = getattr(msg, name)
            info['statuses'][STATUS_NAMES.get(block.status, 'unknown')] += 1
            info['last_shape'], info['last_dtype'] = list(block.shape), block.dtype
            info['last_shape_declared'] = block.shape_declared
            info['last_declared_shape_json'] = block.declared_shape_json
            if block.status == VALID and len(block.raw_values):
                values = np.asarray(block.raw_values)
                if np.isfinite(values).all():
                    low, high = float(values.min()), float(values.max())
                    info.setdefault('first_sample_min_raw', low)
                    info.setdefault('first_sample_max_raw', high)
                    info['min_raw'] = min(info.get('min_raw', low), low)
                    info['max_raw'] = max(info.get('max_raw', high), high)
        self.last_f6_raw = list(msg.f6.raw_values)
        if msg.f6.status == VALID and len(self.last_f6_raw) == 6:
            values = self.last_f6_raw
            if not self.first_f6_raw:
                self.first_f6_raw = values.copy()
                self.f6_component_min = values.copy()
                self.f6_component_max = values.copy()
            self.f6_component_min = [min(a, b) for a, b in zip(self.f6_component_min, values)]
            self.f6_component_max = [max(a, b) for a, b in zip(self.f6_component_max, values)]

    def summary(self, now):
        duration = self.last - self.first if self.count > 1 else 0
        return dict(count=self.count, rejected_metadata=self.rejected,
                    observed_hz=(self.count - 1) / duration if duration > 0 else None,
                    seconds_since_last_receive=now - self.last if self.last is not None else None,
                    validation_error_frames=self.validation_error_frames,
                    blocks=self.blocks, last_f6_raw=self.last_f6_raw,
                    first_f6_raw=self.first_f6_raw,
                    f6_component_min_raw=self.f6_component_min,
                    f6_component_max_raw=self.f6_component_max,
                    last_identity=self.last_identity, last_errors=self.last_errors,
                    session_changes=self.session_changes, sdk_id_regressions=self.id_regressions,
                    sdk_id_forward_gap_events=self.id_gap_events,
                    receipt_sequence_gap_total=self.sequence_gap_total,
                    repeated_or_regressed_sequences=self.repeated_sequences,
                    receive_to_publish_sec=self.receive_to_publish.summary(),
                    receive_to_subscriber_ros_sec=self.receive_to_subscriber.summary())


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='right')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args(args)
    if not math.isfinite(parsed.duration) or parsed.duration <= 0:
        parser.error('--duration must be finite and positive')
    return parsed


def main(args=None):
    args = parse_args(args)
    frame_type = message_type()  # Fail clearly before ROS startup if not built.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    sides = ('left', 'right') if args.side == 'both' else (args.side,)
    summaries = {(side, finger): FingerSummary(side, finger)
                 for side in sides for finger in FINGERS}
    # Exclusive creation avoids overwriting an earlier capture.
    with args.output.open('x') as output:
        rclpy.init(args=[])
        node = None
        try:
            node = Node('tactile_frame_capture', enable_rosout=False,
                        start_parameter_services=False)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            subscriptions = []
            for (side, finger), summary in summaries.items():
                def receive(msg, summary=summary):
                    summary.receive(msg, time.monotonic(), node.get_clock().now().nanoseconds / 1e9)
                subscriptions.append(node.create_subscription(
                    frame_type, f'/sharpa/{side}_hand/tactile/{finger}/frame', receive, qos))
            started = time.monotonic()
            interrupted = False
            try:
                while time.monotonic() - started < args.duration:
                    rclpy.spin_once(node, timeout_sec=min(.1, args.duration))
            except KeyboardInterrupt:
                interrupted = True
            now = time.monotonic()
            result = dict(schema_version=1, captured_at_utc=datetime.now(timezone.utc).isoformat(),
                duration_sec=now - started, interrupted=interrupted,
                mode='subscriptions only; no SDK, services, actions or motion publishers',
                timing_note='receive_to_publish uses driver monotonic time; receive_to_subscriber '
                            'uses ROS clocks and requires synchronized clocks, not SDK measurement time',
                continuity_note='sequence gaps include duplicate suppression, cache replacement and '
                                'transport loss; ID regressions do not distinguish wrap/restart/reordering',
                fingers={f'{side}/{finger}': summary.summary(now)
                         for (side, finger), summary in summaries.items()})
            json.dump(result, output, indent=2, allow_nan=False)
            output.write('\n')
            print(f'Saved {sum(s.count for s in summaries.values())} frames to {args.output}')
            return 0 if all(s.count and not s.rejected for s in summaries.values()) else 2
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
