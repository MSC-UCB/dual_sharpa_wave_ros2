#!/usr/bin/env python3
"""Run the Sharpa SDK sample-style sine motion through the ROS interface.

This script controls both hands through ``joint_command`` topics. It does not
import or call the Sharpa SDK directly. Start the unified hardware launch first
and run this script only when a real motion test is intended.
"""

import argparse
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args

from dual_sharpa_wave.control_model import SIDES, bounded_positions, load_limits, positive
from dual_sharpa_wave.control_ros import CommandClient


# Same joint ranges and skipped MCP joints as sharpa_test/sharpa_wave_example.py.
ANGLE_RANGES_DEG = (
    (0, 50), (0, 10), (0, 30), (0, 10), (0, 40),
    (0, 20), (-20, 20), (0, 20), (0, 20),
    (0, 20), (-20, 20), (0, 20), (0, 20),
    (0, 20), (-20, 20), (0, 20), (0, 20),
    (0, 10), (0, 20), (-20, 20), (0, 20), (0, 20),
)
SKIP_MOVE_JOINT_INDEX = (6, 10, 14, 17, 19)


def parser():
    result = argparse.ArgumentParser(
        description='Run the Sharpa sample-style sine motion on both ROS hands')
    result.add_argument('--rate', type=float, default=100.0,
                        help='command publication rate in Hz')
    result.add_argument('--frequency', type=float, default=1.0,
                        help='sine frequency in Hz')
    result.add_argument('--cycles', type=int, default=3,
                        help='number of sine cycles')
    result.add_argument('--range-scale', type=float, default=1.0,
                        help='scale each sample angle range around its midpoint')
    result.add_argument('--settle-seconds', type=float, default=1.0,
                        help='seconds to publish the starting pose at the end')
    result.add_argument('--state-timeout', type=float, default=1.0)
    result.add_argument('--wait-timeout', type=float, default=10.0)
    return result


def sample_target(elapsed, scale):
    phase = 2.0 * math.pi * elapsed
    targets = []
    for index, (low, high) in enumerate(ANGLE_RANGES_DEG):
        if index in SKIP_MOVE_JOINT_INDEX:
            targets.append(None)
            continue
        midpoint = (low + high) / 2.0
        half_range = (high - low) / 2.0 * scale
        targets.append(math.radians(midpoint + half_range * math.sin(phase)))
    return targets


def main(args=None):
    options = parser().parse_args(remove_ros_args(args or sys.argv)[1:])
    try:
        positive(options.rate, 'rate')
        positive(options.frequency, 'frequency')
        positive(options.settle_seconds, 'settle-seconds')
        positive(options.state_timeout, 'state-timeout')
        positive(options.wait_timeout, 'wait-timeout')
        if options.cycles < 1:
            raise ValueError('cycles must be positive')
        if not math.isfinite(options.range_scale) or options.range_scale <= 0:
            raise ValueError('range-scale must be finite and positive')
        limits = load_limits(Path(get_package_share_directory('dual_sharpa_wave')))
    except (ValueError, OSError) as error:
        parser().error(str(error))

    node = None
    rclpy.init(args=(args or sys.argv)[1:])
    try:
        node = CommandClient('sharpa_wave_example', limits, options.state_timeout)
        deadline = time.monotonic() + options.wait_timeout
        node.get_logger().info('Waiting for both hands: feedback and command subscribers')
        while not all(node.ready(side) for side in SIDES):
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() >= deadline:
                node.require_ready()

        baseline = {side: list(node.positions[side]) for side in SIDES}
        duration = options.cycles / options.frequency
        started = time.monotonic()
        next_tick = started
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.01, next_tick - time.monotonic())))
            now = time.monotonic()
            elapsed = now - started
            if now < next_tick:
                continue

            if elapsed < duration:
                offsets = sample_target(elapsed * options.frequency, options.range_scale)
                targets = {}
                for side in SIDES:
                    target = list(baseline[side])
                    for index, value in enumerate(offsets):
                        if value is not None:
                            target[index] = value
                    targets[side] = bounded_positions(side, target, limits)
                node.send(targets)
            elif elapsed < duration + options.settle_seconds:
                node.send(baseline)
            else:
                node.get_logger().info('Example motion complete; baseline is still being held')
                return 0
            next_tick = now + 1.0 / options.rate
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as error:
        print(f'sharpa_wave_example failed: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
