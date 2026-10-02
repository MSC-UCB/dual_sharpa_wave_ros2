"""Driver-owned zero recovery with action ownership, cancellation and client lease."""

import math
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from std_msgs.msg import String
from sharpa_control_interfaces.action import RecoverDefault

from .zero_trajectory import zero_duration, zero_sample


class RecoveryServer:
    def __init__(self, node, backend, client_timeout=0.5):
        self.node, self.backend = node, backend
        self.client_timeout = client_timeout
        self._reserved = False
        self._goal = self._done = None
        self._heartbeat_at = None
        self._started = self._settled = self._last_step = None
        self.server = ActionServer(
            node, RecoverDefault, 'recover_default', self._execute,
            goal_callback=self._accept, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.heartbeat = node.create_subscription(
            String, 'recovery_heartbeat', self._heartbeat, 1)

    @property
    def active(self):
        return self._reserved

    def _accept(self, request):
        fields = ('minimum_duration_sec', 'max_speed_rad_s', 'max_acceleration_rad_s2',
                  'command_rate_hz', 'tolerance_rad', 'hold_time_sec', 'settle_timeout_sec')
        if (self.active or not self.backend._started or self.backend._faulted or
                any(not math.isfinite(getattr(request, key)) or getattr(request, key) <= 0
                    for key in fields) or
                request.command_rate_hz > min(100.0, self.node.settings['publish_rate_hz']) or
                request.hold_time_sec >= request.settle_timeout_sec):
            return GoalResponse.REJECT
        self._reserved = True  # Reserve before execute so simultaneous goals cannot pass.
        return GoalResponse.ACCEPT

    async def _execute(self, goal):
        self._goal = goal
        self._done = Future()
        self._accepted_at = time.monotonic()
        self._heartbeat_at = self._started = self._settled = self._last_step = None
        self._next_send = 0.0
        self._last_feedback = -math.inf
        return await self._done

    def _heartbeat(self, message):
        if self._goal is not None and message.data == bytes(self._goal.goal_id.uuid).hex():
            self._heartbeat_at = time.monotonic()

    def fail(self, message):
        if self._goal is not None:
            self._finish(False, message)

    def _finish(self, success, message, canceled=False):
        goal, done = self._goal, self._done
        if goal is None:
            return
        if not success:
            try:
                self.backend.cancel_recovery(message)
            except Exception as error:
                # Fresh feedback failure retains the SDK fault, but still ends
                # the action and releases ownership of ordinary commands.
                message = f'{message}; {error}'
        self.node._last_command = None
        self.node.timed_out = False
        self.node._started_at = time.monotonic()
        result = RecoverDefault.Result(success=success, message=message)
        if canceled:
            goal.canceled()
        elif success:
            goal.succeed()
        else:
            goal.abort()
        self._goal = self._done = None
        self._reserved = False
        done.set_result(result)

    def step(self, current, now):
        if self._goal is None:
            return
        goal = self._goal
        if goal.is_cancel_requested:
            self._finish(False, 'Recovery canceled by client', canceled=True)
            return
        last_seen = self._heartbeat_at if self._heartbeat_at is not None else self._accepted_at
        if now - last_seen > self.client_timeout:
            self._finish(False, 'Recovery client heartbeat expired; recovery ended')
            return
        if self._heartbeat_at is None:
            return  # No motion until the client has accepted both hand goals.
        request = goal.request
        try:
            if self._started is None:
                self._start_positions = self.backend.begin_recovery()  # Re-read at actual takeover.
                self._duration = zero_duration(self._start_positions, request.minimum_duration_sec,
                                               request.max_speed_rad_s, request.max_acceleration_rad_s2)
                now = time.monotonic()
                if now - self._heartbeat_at > self.client_timeout:
                    self._finish(False, 'Recovery client heartbeat expired during takeover')
                    return
                self._started = now
                self.node.get_logger().info(f'Explicit default-pose recovery over {self._duration:.2f} s')
            if self._last_step is not None and now - self._last_step > self.client_timeout:
                self.backend.fail_recovery('Recovery control loop stalled')
            self._last_step = now
            elapsed = now - self._started
            if now >= self._next_send:
                self.backend.set_recovery_positions(
                    zero_sample(self._start_positions, elapsed, self._duration))
                self._next_send = now + 1.0 / request.command_rate_hz
            if now - self._last_feedback >= 0.1:
                goal.publish_feedback(RecoverDefault.Feedback(
                    state=self.backend.mit_limits.state, actual_positions=current,
                    max_error_rad=max(abs(q) for q in current)))
                self._last_feedback = now
            if elapsed >= self._duration:
                reached = (max(abs(q) for q in current) <= request.tolerance_rad and
                           self.backend.mit_limits.within_tolerance(current))
                self._settled = (now if self._settled is None else self._settled) if reached else None
                if self._settled is not None and now - self._settled >= request.hold_time_sec:
                    self.backend.finish_recovery(request.tolerance_rad)
                    self._finish(True, 'Default pose reached; measured limits valid; control unlocked')
                elif elapsed > self._duration + request.settle_timeout_sec:
                    worst = max(range(22), key=lambda i: abs(current[i]))
                    violation = math.degrees(max(self.backend.mit_limits.violations(current)))
                    self._finish(False, 'Default pose/limit settling timed out; '
                                 f'{self.backend.mit_limits.names[worst]} '
                                 f'zero error={math.degrees(abs(current[worst])):.3f} deg; '
                                 f'max limit violation={violation:.3f} deg; recovery ended')
        except Exception as error:
            self._finish(False, str(error))

    def close(self):
        self.fail('Hand node is closing')
        self.server.destroy()
