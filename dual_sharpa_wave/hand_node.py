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
from sensor_msgs.msg import JointState

from .hand_interface import HandInterface
from .joint_names import joint_names
from .joint_validation import canonical_positions, validate_positions
from .mock_hand import MockHand
from .qos import hand_qos


def create_backend(parameters: dict) -> HandInterface:
    if parameters['backend'] == 'mock':
        return MockHand(parameters['mock_mode'], parameters['mock_max_velocity_rad_s'])
    if parameters['backend'] == 'sharpa_sdk':
        from .sharpa_sdk_hand import SharpaSdkHand
        return SharpaSdkHand(
            parameters['serial_number'], parameters['speed_coeff'],
            parameters['current_coeff'], parameters['interpolation'],
            parameters['sdk_discovery_timeout_sec'],
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
                'command_timeout_sec': 0.5, 'mock_mode': 'first_order',
                'mock_max_velocity_rad_s': 1.0, 'serial_number': '',
                'speed_coeff': 0.3, 'current_coeff': 0.6, 'interpolation': True,
                'sdk_discovery_timeout_sec': 10.0,
            }
            self.settings = {
                name: self.declare_parameter(
                    name, value, ParameterDescriptor(read_only=True),
                ).value
                for name, value in defaults.items()
            }
            self.names = joint_names(self.settings['side'])
            for key in (
                'publish_rate_hz', 'command_timeout_sec', 'mock_max_velocity_rad_s',
                'speed_coeff', 'current_coeff',
            ):
                if not math.isfinite(self.settings[key]):
                    raise ValueError(f'{key} must be finite')
            if self.settings['publish_rate_hz'] <= 0:
                raise ValueError('publish_rate_hz must be positive')
            self._backend = create_backend(self.settings)
            self._backend.start()
            self._started_at = self._last_update = time.monotonic()
            self._publisher = self.create_publisher(JointState, 'joint_states', hand_qos())
            self._subscription = self.create_subscription(
                JointState, 'joint_command', self._on_command, hand_qos(),
            )
            # Scheduling and timeout use elapsed steady time, not adjustable ROS time.
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(
                1.0 / self.settings['publish_rate_hz'], self._on_timer,
                clock=self._steady_clock,
            )
            self.get_logger().info(
                f"{self.settings['side']} hand ready: backend={self.settings['backend']}, "
                f"rate={self.settings['publish_rate_hz']} Hz, 22 joints in radians"
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
        self.timed_out = timeout > 0 and now - reference > timeout
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
