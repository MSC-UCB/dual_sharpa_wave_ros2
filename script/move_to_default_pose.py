#!/usr/bin/env python3
"""Move selected Sharpa hands once to their 22-joint zero pose.

Source ROS Jazzy and ws_fanuc/install/setup.bash, then run with Python.
Without --execute, print the target only (no ROS node or SDK connection).
Stop other hand command publishers and clear the fingers' path before execution.
Exiting stops new commands; it does not disable the hardware or emergency-stop it.
"""

import argparse
import math
import sys
import time


SIDES = ('left', 'right')
JOINT_COUNT = 22


def default_targets(side='both'):
    return {s: [0.0] * JOINT_COUNT for s in SIDES if side == 'both' or s == side}


class ZeroMove:
    """Synchronized quintic approach, bounded in target speed and acceleration."""

    def __init__(self, starts, minimum_duration, speed, acceleration):
        if any(not math.isfinite(v) or v <= 0 for v in (minimum_duration, speed, acceleration)):
            raise ValueError('Duration, speed and acceleration must be finite and positive')
        if not starts or any(s not in SIDES for s in starts):
            raise ValueError('Expected left and/or right hand starts')
        self.starts = {side: list(q) for side, q in starts.items()}
        if any(len(q) != JOINT_COUNT or not all(math.isfinite(v) for v in q)
               for q in self.starts.values()):
            raise ValueError('Expected 22 finite positions for each hand')
        from dual_sharpa_wave.zero_trajectory import zero_duration
        self.duration = zero_duration([v for q in self.starts.values() for v in q],
                                      minimum_duration, speed, acceleration)

    def sample(self, elapsed):
        from dual_sharpa_wave.zero_trajectory import zero_sample
        return {side: zero_sample(q, elapsed, self.duration) for side, q in self.starts.items()}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='both',
                        help='Hand to reset (default: both)')
    parser.add_argument('--execute', action='store_true', help='Publish motion to selected hands')
    for name, value in (('rate', 100.), ('minimum-duration', 3.),
                        ('max-speed-deg-s', 10.), ('max-accel-deg-s2', 20.),
                        ('state-timeout', .5), ('wait-timeout', 15.),
                        ('settle-timeout', 15.), ('tolerance-deg', 2.), ('hold-time', 1.)):
        parser.add_argument('--' + name, type=float, default=value)
    args = parser.parse_args(argv)
    for name, value in vars(args).items():
        if name not in ('execute', 'side') and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.rate > 100:
        parser.error('--rate must be <= 100 Hz')
    if args.hold_time >= args.settle_timeout:
        parser.error('--hold-time must be less than --settle-timeout')
    return args


def pose_error_summary(positions, targets, tolerance_deg):
    """Report actual joint errors without changing the arrival criterion."""
    from dual_sharpa_wave.joint_names import joint_names

    summaries = []
    for side in targets:
        errors = [math.degrees(abs(q - goal))
                  for q, goal in zip(positions[side], targets[side])]
        worst = max(range(JOINT_COUNT), key=errors.__getitem__)
        outside = sum(error > tolerance_deg for error in errors)
        summaries.append(f'{side}: {outside}/22 outside {tolerance_deg:g} deg, '
                         f'worst={joint_names(side)[worst]} error={errors[worst]:.3f} deg '
                         f'actual={math.degrees(positions[side][worst]):.3f} deg '
                         f'target={math.degrees(targets[side][worst]):.3f} deg')
    return '; '.join(summaries)


def run(node, args, spin_once, ok):
    """Coordinate an existing CommandClient; also usable with offline fake IO."""
    from dual_sharpa_wave.joint_validation import validate_positions

    targets = default_targets(args.side)
    sides = tuple(targets)
    deadline = time.monotonic() + args.wait_timeout
    plan = None
    started = settled = last_sent = None
    last_settle_report = None
    next_tick = time.monotonic()
    node.get_logger().info('Waiting for selected hands: fresh feedback and command subscribers')
    while ok():
        spin_once(node, timeout_sec=max(0., min(.01, next_tick - time.monotonic())))
        now = time.monotonic()
        for side in sides:
            if node.count_publishers(f'/sharpa/{side}_hand/joint_command') > 1:
                raise RuntimeError(f'{side}: another command publisher exists; stop it first')
        errors = {s: node.feedback_errors[s] for s in sides if s in node.feedback_errors}
        if errors:
            raise RuntimeError(f'Invalid feedback: {errors}')
        if plan is None:
            if not all(node.ready(s) for s in sides):
                if now >= deadline:
                    waiting = ', '.join(side for side in sides if not node.ready(side))
                    raise TimeoutError(f'Timed out waiting for selected hands; not ready: {waiting}. '
                                       'Need fresh feedback and a joint_command subscriber; '
                                       'check driver logs for a stopped/faulted SDK session')
                next_tick = now + 1 / args.rate
                continue
            starts = {s: validate_positions(node.positions[s]) for s in sides}
            plan = ZeroMove(starts, args.minimum_duration, math.radians(args.max_speed_deg_s),
                            math.radians(args.max_accel_deg_s2))
            started = next_tick = now
            node.get_logger().info(f'Moving selected hands to zero over {plan.duration:.2f} s')
        node.require_ready(sides)
        for side in sides:
            validate_positions(node.positions[side])
        if last_sent is not None and now - last_sent > args.state_timeout:
            raise RuntimeError('Command publication loop stalled')
        if now < next_tick:
            continue
        elapsed = now - started
        node.send(plan.sample(elapsed))
        last_sent = now
        next_tick = now + 1 / args.rate
        if elapsed >= plan.duration:
            if last_settle_report is None or now - last_settle_report >= 1.0:
                node.get_logger().info('Settling: ' + pose_error_summary(
                    node.positions, targets, args.tolerance_deg))
                last_settle_report = now
            reached = all(abs(q - goal) <= math.radians(args.tolerance_deg)
                          for s in sides for q, goal in zip(node.positions[s], targets[s]))
            settled = (now if settled is None else settled) if reached else None
            if settled is not None and now - settled >= args.hold_time:
                node.get_logger().info('Selected hands reached default pose; finished sending.')
                return 0
            if elapsed > plan.duration + args.settle_timeout:
                raise TimeoutError('Selected hands did not settle at the default pose in time. '
                                   + pose_error_summary(node.positions, targets, args.tolerance_deg)
                                   + f'; required continuous hold={args.hold_time:g} s')
    return 1


def execute(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from dual_sharpa_wave.control_ros import CommandClient
    from dual_sharpa_wave.recovery_client import driver_modes, recover_selected

    sides = tuple(default_targets(args.side))
    node = None
    rclpy.init(args=[])
    try:
        node = CommandClient('sharpa_move_to_default_pose', None, args.state_timeout, sides=sides)
        mode = driver_modes(node, args.wait_timeout, rclpy.spin_once, rclpy.ok, sides=sides)
        if mode == 'mit':
            return recover_selected(node, args, rclpy.spin_once, rclpy.ok, sides=sides)
        return run(node, args, rclpy.spin_once, rclpy.ok)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Interrupted: stopped new targets; this is not a hardware stop.', file=sys.stderr)
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    args = parse_args(argv)
    for side, q in default_targets(args.side).items():
        print(f'{side}: {JOINT_COUNT} joints, radians={q}', flush=True)
    if not args.execute:
        print('Preview only. Add --execute to move selected hands to this pose.')
        return 0
    try:
        return execute(args)
    except Exception as exc:
        print(f'Failed: {exc}. No further targets will be sent; motors are not disabled.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
