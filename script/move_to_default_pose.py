#!/usr/bin/env python3
"""Move both Sharpa hands once to their 22-joint zero pose.

Source ROS Jazzy and ws_fanuc/install/setup.bash, then run with Python.
Without --execute, print the target only (no ROS node or SDK connection).
Stop other hand command publishers and clear the fingers' path before execution.
Exiting stops new commands; it does not disable the hardware or emergency-stop it.
"""

import argparse
import math
from pathlib import Path
import sys
import time


SIDES = ('left', 'right')
JOINT_COUNT = 22


def default_targets():
    return {side: [0.0] * JOINT_COUNT for side in SIDES}


class ZeroMove:
    """Synchronized quintic approach, bounded in target speed and acceleration."""

    def __init__(self, starts, minimum_duration, speed, acceleration):
        if any(not math.isfinite(v) or v <= 0 for v in (minimum_duration, speed, acceleration)):
            raise ValueError('Duration, speed and acceleration must be finite and positive')
        self.starts = {side: list(starts[side]) for side in SIDES}
        if any(len(q) != JOINT_COUNT or not all(math.isfinite(v) for v in q)
               for q in self.starts.values()):
            raise ValueError('Expected 22 finite positions for each hand')
        distance = max(abs(v) for q in self.starts.values() for v in q)
        self.duration = max(minimum_duration, 1.875 * distance / speed,
                            math.sqrt(10 / math.sqrt(3) * distance / acceleration))

    def sample(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError('Elapsed time must be finite')
        u = min(1., max(0., elapsed / self.duration))
        if u == 1.:
            return default_targets()
        blend = u**3 * (10 + u * (-15 + 6 * u))
        return {side: [(1 - blend) * v for v in q] for side, q in self.starts.items()}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Publish motion to both hands')
    for name, value in (('rate', 100.), ('minimum-duration', 3.),
                        ('max-speed-deg-s', 10.), ('max-accel-deg-s2', 20.),
                        ('state-timeout', .5), ('wait-timeout', 15.),
                        ('settle-timeout', 15.), ('tolerance-deg', 2.), ('hold-time', 1.)):
        parser.add_argument('--' + name, type=float, default=value)
    args = parser.parse_args(argv)
    for name, value in vars(args).items():
        if name != 'execute' and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.rate > 100:
        parser.error('--rate must be <= 100 Hz')
    if args.hold_time >= args.settle_timeout:
        parser.error('--hold-time must be less than --settle-timeout')
    return args


def run(node, args, limits, spin_once, ok):
    """Coordinate an existing CommandClient; also usable with offline fake IO."""
    from dual_sharpa_wave.control_model import bounded_positions

    targets = {side: bounded_positions(side, q, limits) for side, q in default_targets().items()}
    deadline = time.monotonic() + args.wait_timeout
    plan = None
    started = settled = last_sent = None
    next_tick = time.monotonic()
    node.get_logger().info('Waiting for both hands: fresh feedback and command subscribers')
    while ok():
        spin_once(node, timeout_sec=max(0., min(.01, next_tick - time.monotonic())))
        now = time.monotonic()
        for side in SIDES:
            if node.count_publishers(f'/sharpa/{side}_hand/joint_command') > 1:
                raise RuntimeError(f'{side}: another command publisher exists; stop it first')
        if node.feedback_errors:
            raise RuntimeError(f'Invalid feedback: {node.feedback_errors}')
        if plan is None:
            if not all(node.ready(s) for s in SIDES):
                if now >= deadline:
                    raise TimeoutError('Timed out waiting for both hands')
                next_tick = now + 1 / args.rate
                continue
            starts = {s: bounded_positions(s, node.positions[s], limits) for s in SIDES}
            plan = ZeroMove(starts, args.minimum_duration, math.radians(args.max_speed_deg_s),
                            math.radians(args.max_accel_deg_s2))
            started = next_tick = now
            node.get_logger().info(f'Moving both hands to zero over {plan.duration:.2f} s')
        node.require_ready()
        for side in SIDES:
            bounded_positions(side, node.positions[side], limits)
        if last_sent is not None and now - last_sent > args.state_timeout:
            raise RuntimeError('Command publication loop stalled')
        if now < next_tick:
            continue
        elapsed = now - started
        node.send(plan.sample(elapsed))
        last_sent = now
        next_tick = now + 1 / args.rate
        if elapsed >= plan.duration:
            reached = all(abs(q - goal) <= math.radians(args.tolerance_deg)
                          for s in SIDES for q, goal in zip(node.positions[s], targets[s]))
            settled = (now if settled is None else settled) if reached else None
            if settled is not None and now - settled >= args.hold_time:
                node.get_logger().info('Both hands reached default pose; finished sending.')
                return 0
            if elapsed > plan.duration + args.settle_timeout:
                raise TimeoutError('Both hands did not settle at the default pose in time')
    return 1


def execute(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from ament_index_python.packages import get_package_share_directory
    from dual_sharpa_wave.control_model import load_limits
    from dual_sharpa_wave.control_ros import CommandClient

    limits = load_limits(Path(get_package_share_directory('dual_sharpa_wave')))
    node = None
    rclpy.init(args=[])
    try:
        node = CommandClient('sharpa_move_to_default_pose', limits, args.state_timeout)
        return run(node, args, limits, rclpy.spin_once, rclpy.ok)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Interrupted: stopped new targets; this is not a hardware stop.', file=sys.stderr)
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    args = parse_args(argv)
    for side, q in default_targets().items():
        print(f'{side}: {JOINT_COUNT} joints, radians={q}', flush=True)
    if not args.execute:
        print('Preview only. Add --execute to move both hands to this pose.')
        return 0
    try:
        return execute(args)
    except Exception as exc:
        print(f'Failed: {exc}. No further targets will be sent; motors are not disabled.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
