"""Shared ROS communication, validation, scheduling and feedback for either backend."""

import math
import signal
import sys
import time

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from .hand_interface import HandInterface
from .joint_names import joint_names
from .joint_validation import canonical_positions, validate_positions
from .mock_hand import MockHand
from .qos import hand_qos


def create_backend(parameters: dict) -> HandInterface:
    if parameters['backend'] == 'mock':
        return MockHand()
    if parameters['backend'] == 'sharpa_sdk':
        from .sharpa_sdk_hand import SharpaSdkHand
        return SharpaSdkHand(
            parameters['serial_number'], parameters['speed_coeff'],
            parameters['current_coeff'], parameters['interpolation'],
            parameters['sdk_discovery_timeout_sec'],
            read_only=parameters['read_only'], tactile_enabled=parameters['tactile_enabled'],
            side=parameters['side'], stamp_clock=parameters['_tactile_stamp_clock'],
        )
    raise ValueError("backend must be 'mock' or 'sharpa_sdk'")


class HandNode(Node):
    def __init__(self, **kwargs):
        super().__init__('hand_node', **kwargs)
        self._backend = None
        self._closed = False
        self._warnings = {}
        self._last_command = None
        self.timed_out = False
        try:
            defaults = {
                'side': 'left', 'backend': 'mock', 'publish_rate_hz': 100.0,
                'command_timeout_sec': 0.5, 'serial_number': '',
                'speed_coeff': 0.3, 'current_coeff': 0.6, 'interpolation': True,
                'sdk_discovery_timeout_sec': 10.0,
                'read_only': False, 'tactile_enabled': False, 'tactile_publish_rate_hz': 30.0,
            }
            self.settings = {
                name: self.declare_parameter(
                    name, value, ParameterDescriptor(read_only=True),
                ).value
                for name, value in defaults.items()
            }
            self.names = joint_names(self.settings['side'])
            for key in (
                'publish_rate_hz', 'command_timeout_sec',
                'speed_coeff', 'current_coeff', 'tactile_publish_rate_hz',
            ):
                if not math.isfinite(self.settings[key]):
                    raise ValueError(f'{key} must be finite')
            if self.settings['publish_rate_hz'] <= 0:
                raise ValueError('publish_rate_hz must be positive')
            if not 0 < self.settings['tactile_publish_rate_hz'] <= 100:
                raise ValueError('tactile_publish_rate_hz must be in (0, 100]')
            self._backend = create_backend(dict(
                self.settings, _tactile_stamp_clock=lambda: self.get_clock().now().to_msg()))
            self._backend.start()
            self._started_at = self._last_update = time.monotonic()
            self._publisher = self.create_publisher(JointState, 'joint_states', hand_qos())
            self._subscription = None if self.settings['read_only'] else self.create_subscription(
                JointState, 'joint_command', self._on_command, hand_qos(),
            )
            # Scheduling and timeout use elapsed steady time, not adjustable ROS time.
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(
                1.0 / self.settings['publish_rate_hz'], self._on_timer,
                clock=self._steady_clock,
            )
            self._tactile_publishers = {}
            self._tactile_formats = {}
            if self.settings['tactile_enabled']:
                from .tactile import BLOCKS, FINGERS
                from rclpy.qos import QoSProfile, ReliabilityPolicy
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                self._tactile_publishers = {
                    (finger, block): self.create_publisher(
                        Image, f'tactile/{finger}/{block.lower()}', qos)
                    for finger in FINGERS for block in BLOCKS
                }
                self._tactile_timer = self.create_timer(
                    1.0 / self.settings['tactile_publish_rate_hz'], self._on_tactile_timer,
                    clock=self._steady_clock)
            self.get_logger().info(
                f"{self.settings['side']} hand ready: backend={self.settings['backend']}, "
                f"rate={self.settings['publish_rate_hz']} Hz, 22 joints in radians, "
                f"read_only={self.settings['read_only']}, tactile={self.settings['tactile_enabled']}"
            )
        except BaseException:
            self.destroy_node()
            raise

    def _warn(self, category: str, message: str) -> None:
        now = time.monotonic()
        if now - self._warnings.get(category, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self._warnings[category] = now

    def _on_command(self, msg: JointState) -> None:
        if self.settings['read_only']:
            self._warn('read_only', 'Read-only node rejects motion commands')
            return
        try:
            positions = canonical_positions(msg.name, msg.position, self.names)
        except ValueError as error:
            self._warn('command', f'Rejected entire joint command: {error}')
            return
        try:
            self._backend.set_joint_positions(positions)
        except Exception as error:
            self._warn('write', f'Backend command failed: {error}')
            return
        self._last_command = time.monotonic()
        self.timed_out = False

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt_sec = max(0.0, now - self._last_update)
        self._last_update = now
        timeout = self.settings['command_timeout_sec']
        reference = self._started_at if self._last_command is None else self._last_command
        was_timed_out = self.timed_out
        self.timed_out = not self.settings['read_only'] and timeout > 0 and now - reference > timeout
        if self.timed_out:
            self._warn('timeout', 'Command timeout: no new target sent; applying backend timeout policy')
            if not was_timed_out and self._last_command is not None:
                try:
                    self._backend.on_command_timeout()
                except Exception as error:
                    self._warn('timeout_backend', f'Backend timeout handling: {error}')
        try:
            self._backend.update(dt_sec)
            positions = validate_positions(self._backend.get_joint_positions())
        except Exception as error:
            self._warn('read', f'Backend feedback unavailable; skipping state publication: {error}')
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = positions
        self._publisher.publish(msg)

    def _on_tactile_timer(self):
        from .tactile import FINGERS, to_image
        frames, error = self._backend.take_tactile_frames()
        if error:
            self._warn('tactile_receive', f'Tactile receive error: {error}')
        for frame in frames:
            finger = FINGERS[frame.channel % 5]
            signature = tuple((key, value.shape, str(value.dtype))
                              for key, value in frame.blocks.items())
            if self._tactile_formats.get(frame.channel) != signature:
                self._tactile_formats[frame.channel] = signature
                self.get_logger().info(
                    f'Tactile ch={frame.channel} {finger} sdk_ts={frame.sdk_ts} '
                    f'frame_id={frame.frame_id} blocks={signature}')
            for block, error in frame.errors.items():
                self._warn(f'tactile_{frame.channel}_{block}', f'{finger}/{block}: {error}')
            for block, array in frame.blocks.items():
                self._tactile_publishers[finger, block].publish(to_image(array, frame.stamp))

    def destroy_node(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self._backend is not None:
                self._backend.stop()
        except Exception as error:
            self.get_logger().error(f'Backend cleanup failed: {error}')
        finally:
            result = super().destroy_node()
        return result


def main(args=None):
    node = None
    rclpy.init(args=args)
    try:
        node = HandNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'hand_node failed: {error}', file=sys.stderr)
        return 1
    finally:
        # A terminal SIGINT and launch's forwarded SIGINT can arrive separately.
        # Do not allow a second interrupt to skip backend/resource cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
    return 0
