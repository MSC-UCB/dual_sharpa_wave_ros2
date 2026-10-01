"""Paired default-pose action client; no SDK calls, no arbitrary recovery targets."""

import math
import time

from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from std_msgs.msg import String
from sharpa_control_interfaces.action import RecoverDefault

from .control_model import SIDES
from .joint_names import joint_names
from .zero_trajectory import zero_duration


def driver_modes(node, timeout, spin_once, ok):
    clients = {s: node.create_client(GetParameters, f'/sharpa/{s}_hand/hand_node/get_parameters')
               for s in SIDES}
    futures = {}
    deadline = time.monotonic() + timeout
    try:
        while ok():
            for side, client in clients.items():
                if side not in futures and client.service_is_ready():
                    futures[side] = client.call_async(GetParameters.Request(names=['control_mode', 'read_only']))
            if len(futures) == 2 and all(f.done() for f in futures.values()):
                modes = {}
                for side, future in futures.items():
                    values = future.result().values
                    if len(values) != 2 or values[1].bool_value:
                        raise RuntimeError(f'{side}: driver is read-only or parameters unavailable')
                    modes[side] = values[0].string_value
                if any(mode not in ('position', 'mit') for mode in modes.values()):
                    raise RuntimeError(f'Unknown driver control modes: {modes}')
                if len(set(modes.values())) != 1:
                    raise RuntimeError('Paired default pose requires both hands in the same control mode')
                return modes['left']
            if time.monotonic() > deadline:
                raise TimeoutError('Cannot read both hand driver modes; check launch and ROS domain')
            spin_once(node, timeout_sec=.01)
        raise RuntimeError('ROS stopped while querying hand drivers')
    finally:
        for client in clients.values():
            node.destroy_client(client)


def recover_both(node, args, spin_once, ok):
    clients = {s: ActionClient(node, RecoverDefault, f'/sharpa/{s}_hand/recover_default') for s in SIDES}
    heartbeats = {s: node.create_publisher(String, f'/sharpa/{s}_hand/recovery_heartbeat', 1) for s in SIDES}
    requests, handles, results = {}, {}, {}
    complete = False
    deadline = time.monotonic() + args.wait_timeout

    def check_publishers():
        if node.feedback_errors:
            raise RuntimeError(f'Invalid feedback: {node.feedback_errors}')
        for side in SIDES:
            if node.count_publishers(f'/sharpa/{side}_hand/joint_command') > 1:
                raise RuntimeError(f'{side}: another command publisher exists; stop it first')

    try:
        node.get_logger().info('Waiting for both MIT recovery actions and fresh hand feedback')
        while ok():
            spin_once(node, timeout_sec=.01)
            check_publishers()
            if all(node.ready(s) and clients[s].server_is_ready() for s in SIDES):
                break
            if time.monotonic() > deadline:
                raise TimeoutError('MIT recovery action/feedback unavailable; rebuild and restart hand drivers')
        else:
            raise RuntimeError('ROS stopped before recovery')
        duration = max(zero_duration(node.positions[s], args.minimum_duration,
                                    math.radians(args.max_speed_deg_s), math.radians(args.max_accel_deg_s2))
                       for s in SIDES)
        for side in SIDES:
            request = RecoverDefault.Goal(
                minimum_duration_sec=duration, max_speed_rad_s=math.radians(args.max_speed_deg_s),
                max_acceleration_rad_s2=math.radians(args.max_accel_deg_s2), command_rate_hz=args.rate,
                tolerance_rad=math.radians(args.tolerance_deg), hold_time_sec=args.hold_time,
                settle_timeout_sec=args.settle_timeout)
            requests[side] = clients[side].send_goal_async(request)
        deadline = time.monotonic() + args.wait_timeout
        while ok() and len(handles) < 2:
            spin_once(node, timeout_sec=.01)
            for side, future in requests.items():
                if side not in handles and future.done():
                    handle = future.result()
                    if not handle.accepted:
                        raise RuntimeError(f'{side}: default recovery request rejected; check driver state/rate')
                    handles[side] = handle
                    results[side] = handle.get_result_async()
            if time.monotonic() > deadline:
                raise TimeoutError('Waiting for recovery goal acceptance timed out')
        # No heartbeat (and thus no driver motion) until both goals are accepted.
        next_report = 0.0
        while ok():
            check_publishers()
            node.require_ready()
            for side, future in results.items():
                if future.done() and not future.result().result.success:
                    raise RuntimeError(f'{side}: {future.result().result.message}')
            if len(results) == 2 and all(f.done() for f in results.values()):
                complete = True
                node.get_logger().info('Both hands reached default pose; MIT control unlocked.')
                return 0
            for side, handle in handles.items():
                if not results[side].done():
                    heartbeats[side].publish(String(data=bytes(handle.goal_id.uuid).hex()))
            now = time.monotonic()
            if now >= next_report:
                details = []
                for side in SIDES:
                    errors = [math.degrees(abs(q)) for q in node.positions[side]]
                    worst = max(range(22), key=errors.__getitem__)
                    details.append(f'{side}: {sum(e > args.tolerance_deg for e in errors)}/22 '
                                   f'outside {args.tolerance_deg:g} deg, '
                                   f'worst={joint_names(side)[worst]} error={errors[worst]:.3f} deg')
                node.get_logger().info('Recovering: ' + '; '.join(details))
                next_report = now + 1
            spin_once(node, timeout_sec=.01)
        raise RuntimeError('ROS stopped during recovery')
    finally:
        if not complete and ok():
            # Cancel accepted peers on rejection, stale feedback, failure or Ctrl+C.
            # If cancellation cannot be delivered, absence of heartbeat expires the lease.
            cancels = []
            for side, future in requests.items():
                try:
                    if future.done() and not future.cancelled():
                        handle = future.result()
                        if handle is not None and handle.accepted:
                            cancels.append(handle.cancel_goal_async())
                except Exception:
                    # Context/transport may already be gone; the lease is the fallback.
                    pass
            until = time.monotonic() + .3
            while ok() and any(not f.done() for f in cancels) and time.monotonic() < until:
                try:
                    spin_once(node, timeout_sec=.01)
                except Exception:
                    break
        for client in clients.values():
            client.destroy()
        for publisher in heartbeats.values():
            node.destroy_publisher(publisher)
