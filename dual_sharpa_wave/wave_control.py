"""CLI sine/step tools sharing one ROS loop and the same sequence model."""

import argparse
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args

from .control_model import SIDES, Waveform, WaveSequence, axis_indices, load_limits, positive
from .control_ros import CommandClient


def parser_for(kind):
    parser = argparse.ArgumentParser(description=f'Paired Sharpa {kind} commands, angles in radians')
    parser.add_argument('--axes', default='thumb_CMC_FE,index_MCP_FE,middle_MCP_FE',
                        help='comma-separated joint suffixes, without left_/right_')
    parser.add_argument('--rate', type=float, default=30.0, help='command publication Hz')
    parser.add_argument('--cycles', type=int, default=2, help='cycles per axis')
    parser.add_argument('--repeat', type=int, default=1, help='repeat the entire axis list')
    parser.add_argument('--gap', type=float, default=0.25, help='minimum hold between axes, seconds')
    parser.add_argument('--settle-timeout', type=float, default=10.0)
    parser.add_argument('--tolerance', type=float, default=0.02, help='baseline arrival tolerance, rad')
    parser.add_argument('--state-timeout', type=float, default=1.0)
    parser.add_argument('--wait-timeout', type=float, default=10.0, help='initial ROS readiness timeout')
    if kind == 'sine':
        parser.add_argument('--amplitude', type=float, default=0.1, help='offset amplitude, rad')
        parser.add_argument('--frequency', type=float, default=0.25, help='sine frequency, Hz')
    else:
        parser.add_argument('--step-size', type=float, default=0.1, help='signed step offset, rad')
        parser.add_argument('--base-seconds', type=float, default=1.0)
        parser.add_argument('--high-seconds', type=float, default=1.0)
    return parser


def main(kind, args=None):
    raw = sys.argv if args is None else [f'{kind}_control.py', *args]
    parser = parser_for(kind)
    options = parser.parse_args(remove_ros_args(raw)[1:])
    try:
        positive(options.rate, 'rate')
        positive(options.state_timeout, 'state-timeout')
        positive(options.wait_timeout, 'wait-timeout')
        axes = axis_indices(options.axes)
        wave = (Waveform(kind, options.amplitude, options.frequency, options.cycles)
                if kind == 'sine' else Waveform(
                    kind, options.step_size, cycles=options.cycles,
                    base_seconds=options.base_seconds, high_seconds=options.high_seconds))
        if kind == 'sine' and options.rate < 10 * options.frequency:
            raise ValueError('rate must be at least 10 times the sine frequency')
        if kind == 'step' and options.rate * min(wave.base_seconds, wave.high_seconds) < 2:
            raise ValueError('each step segment must span at least two publication periods')
        limits = load_limits(Path(get_package_share_directory('dual_sharpa_wave')))
    except (ValueError, OSError) as error:
        parser.error(str(error))
    node = None
    rclpy.init(args=raw[1:])
    try:
        node = CommandClient(f'sharpa_{kind}_control', limits, options.state_timeout)
        deadline = time.monotonic() + options.wait_timeout
        node.get_logger().info('Waiting for both hands: feedback and command subscribers')
        while not all(node.ready(s) for s in SIDES):
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() >= deadline:
                node.require_ready()
        sequence = WaveSequence(
            node.positions, limits, axes, wave, repeat=options.repeat, gap=options.gap,
            settle_timeout=options.settle_timeout, tolerance=options.tolerance)
        started = next_tick = time.monotonic()
        last_index = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.01, next_tick - time.monotonic())))
            now = time.monotonic()
            if now < next_tick:
                continue
            node.require_ready()
            targets = sequence.sample(now - started, node.positions)
            if targets is None:
                node.get_logger().info('Sequence complete; both hands returned to starting pose')
                return 0
            if sequence.index != last_index:
                node.get_logger().info(f'Testing {sequence.axis_name} ({sequence.index + 1}/{len(sequence.axes)})')
                last_index = sequence.index
            node.send(targets)
            next_tick = now + 1.0 / options.rate
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as error:
        print(f'{kind} control failed: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


def sine_main(args=None):
    return main('sine', args)


def step_main(args=None):
    return main('step', args)
