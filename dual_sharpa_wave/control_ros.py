"""Shared ROS command transport for GUI and waveform tools; no backend imports."""

import time

from rclpy.node import Node
from sensor_msgs.msg import JointState

from .control_model import SIDES, bounded_positions, positive
from .joint_names import joint_names
from .joint_validation import canonical_positions
from .qos import hand_qos


class CommandClient(Node):
    def __init__(self, name, limits, state_timeout=1.0, **kwargs):
        super().__init__(name, **kwargs)
        self.limits = limits
        self.state_timeout = positive(state_timeout, 'state-timeout')
        self.positions = {}
        self.received_at = {}
        self.feedback_errors = {}
        self.command_publishers = {}
        self.subscriptions_ = []
        for side in SIDES:
            self.command_publishers[side] = self.create_publisher(
                JointState, f'/sharpa/{side}_hand/joint_command', hand_qos())
            self.subscriptions_.append(self.create_subscription(
                JointState, f'/sharpa/{side}_hand/joint_states',
                lambda msg, side=side: self.on_state(side, msg), hand_qos()))

    def on_state(self, side, msg):
        try:
            if not msg.name:
                raise ValueError('feedback must include joint names')
            self.positions[side] = canonical_positions(msg.name, msg.position, joint_names(side))
            self.received_at[side] = time.monotonic()
            self.feedback_errors.pop(side, None)
        except ValueError as error:
            self.feedback_errors[side] = str(error)
            self.received_at.pop(side, None)

    def ready(self, side):
        return (
            side in self.received_at
            and time.monotonic() - self.received_at[side] <= self.state_timeout
            and self.command_publishers[side].get_subscription_count() > 0
        )

    def require_ready(self, sides=SIDES):
        for side in sides:
            if not self.ready(side):
                detail = self.feedback_errors.get(side, 'feedback missing/stale or command subscriber missing')
                raise RuntimeError(f'{side}: {detail}')

    def send(self, targets):
        self.require_ready(targets)
        # Validate both sides before publishing either one.
        checked = {s: bounded_positions(s, q, self.limits) for s, q in targets.items()}
        stamp = self.get_clock().now().to_msg()
        for side, positions in checked.items():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(joint_names(side))
            msg.position = positions
            self.command_publishers[side].publish(msg)
