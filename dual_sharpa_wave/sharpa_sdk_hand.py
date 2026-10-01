"""SDK 5.0.10 adapter. The SDK is imported only by start(), never by mock use.

The injectable sdk_factory returns the same module-shaped API as ``sharpa``.
See docs/hardware.md for source evidence and unverified device behavior.
"""

from collections.abc import Sequence
import importlib
import math
import time

from .hand_interface import BackendError, HandInterface
from .joint_names import SDK_TO_URDF_INDEX, URDF_TO_SDK_INDEX
from .joint_validation import validate_positions


def load_sdk():
    try:
        return importlib.import_module('sharpa')
    except (ImportError, OSError) as error:
        raise BackendError(
            'Cannot load Sharpa SDK. Configure PYTHONPATH=/opt/sharpa-wave-sdk/python '
            'and LD_LIBRARY_PATH=/opt/sharpa-wave-sdk/lib for the matching Python ABI.'
        ) from error


class SharpaSdkHand(HandInterface):
    def __init__(self, serial_number: str, speed_coeff: float, current_coeff: float,
                 interpolation: bool, discovery_timeout_sec=10.0, *, sdk_factory=None,
                 clock=time.monotonic, sleep=time.sleep, read_only=False,
                 tactile_enabled=False, side='left', stamp_clock=time.time_ns,
                 control_mode='position', mit_start_tolerance_rad=0.1,
                 command_timeout_sec=0.5, joint_limits=None,
                 joint_limit_tolerance_deg=0.5, recovery_worsening_deg=0.5,
                 recovery_worsening_sec=0.2):
        if not isinstance(serial_number, str) or not serial_number.strip():
            raise ValueError('sharpa_sdk requires an explicit nonempty serial_number')
        for name, value in (('speed_coeff', speed_coeff), ('current_coeff', current_coeff)):
            if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')
        if not isinstance(interpolation, bool):
            raise ValueError('interpolation must be bool')
        if not math.isfinite(discovery_timeout_sec) or discovery_timeout_sec <= 0:
            raise ValueError('sdk_discovery_timeout_sec must be finite and positive')
        if control_mode not in ('position', 'mit'):
            raise ValueError('control_mode must be position or mit')
        self.control_mode = control_mode
        self.mit_settings = None
        self._mit_last_command_at = None
        self._mit_start_tolerance = mit_start_tolerance_rad
        self._command_timeout = command_timeout_sec
        self.mit_limits = None
        if control_mode == 'mit' and not read_only:
            if interpolation:
                raise ValueError('MIT requires interpolation=false; interpolate targets upstream')
            for name, value in (('mit_start_tolerance_rad', self._mit_start_tolerance),
                                ('command_timeout_sec', self._command_timeout)):
                if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'MIT requires finite positive {name}')
            from .mit_limits import MitLimits
            from .joint_names import joint_names
            if joint_limits is None:
                from pathlib import Path
                from ament_index_python.packages import get_package_share_directory
                from .control_model import load_limits
                joint_limits = load_limits(Path(get_package_share_directory('dual_sharpa_wave')))[side]
            self.mit_limits = MitLimits(joint_limits, joint_names(side), joint_limit_tolerance_deg,
                                        recovery_worsening_deg, recovery_worsening_sec)
        self.serial_number = serial_number.strip()
        self.speed_coeff, self.current_coeff = speed_coeff, current_coeff
        self.interpolation = interpolation
        self.discovery_timeout_sec = discovery_timeout_sec
        self._sdk_factory = sdk_factory or load_sdk
        self._clock, self._sleep = clock, sleep
        self._manager = self._hand = None
        self._connection_attempted = False
        self._started = False
        self._faulted = False
        self.read_only = read_only
        self.side = side
        self._tactile_enabled = tactile_enabled
        self._stamp_clock = stamp_clock
        self._tactile = None
        if tactile_enabled:
            from .tactile import channels
            channels(side)  # Validate before any SDK connection is attempted.

    @staticmethod
    def _status(operation, status):
        if getattr(status, 'code', None) != 0:
            raise BackendError(f'{operation} failed: code={getattr(status, "code", None)} '
                               f'{getattr(status, "message", "invalid SDK status")}')

    @staticmethod
    def _bool(operation, result):
        if result is not True:
            raise BackendError(f'{operation} failed: expected True, got {result!r}')

    def start(self):
        if self._faulted:
            raise BackendError('Hardware fault latched; restart the hand node before resuming')
        if self._started:
            return
        try:
            sdk = self._sdk_factory()
            self._manager = sdk.SharpaWaveManager.get_instance()
            deadline = self._clock() + self.discovery_timeout_sec
            while self.serial_number not in self._manager.get_all_device_sn():
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise BackendError(f'Discovery timed out for serial {self.serial_number}')
                self._sleep(min(0.1, remaining))
            self._connection_attempted = True
            self._hand = self._manager.connect(self.serial_number)
            if self._hand is None:
                raise BackendError(f'connect returned no hand for {self.serial_number}')
            if self._tactile_enabled:
                from .tactile import TactileCache
                info = self._hand.get_device_info()
                expected = sdk.HandSide.LEFT if self.side == 'left' else sdk.HandSide.RIGHT
                if info.hand_side != expected:
                    raise BackendError(f'{self.serial_number}: hand side does not match {self.side}')
                if not info.has_fingertip_tactile():
                    raise BackendError(f'{self.serial_number}: fingertip tactile is unsupported')
                # A stopped cache stays closed for late callbacks from the old session.
                # Each new session gets fresh pending frames and duplicate tracking.
                self._tactile = TactileCache(self.side, self._stamp_clock)
                self._hand.set_tactile_callback(self._tactile.receive)
            if self.control_mode == 'mit' and not self.read_only:
                from .mit_control import read_mit_settings
                self.mit_settings = read_mit_settings(self._hand)
            settings = () if self.read_only else (
                ('set_control_mode', sdk.ControlMode.MIT if self.control_mode == 'mit'
                 else sdk.ControlMode.POSITION),
                ('set_speed_coeff', self.speed_coeff),
                ('set_current_coeff', self.current_coeff),
                ('set_control_source', sdk.ControlSource.SDK),
            )
            for method, value in settings:
                self._status(method, getattr(self._hand, method)(value))
            if self.control_mode == 'mit' and not self.read_only:
                status, actual_mode = self._hand.get_control_mode()
                self._status('get_control_mode', status)
                if actual_mode != sdk.ControlMode.MIT:
                    raise BackendError(f'MIT mode readback mismatch: {actual_mode}')
            self._bool('start', self._hand.start())
            if self._tactile is not None and not self._hand.is_tactile_ready():
                raise BackendError('Tactile receiver is not ready after start')
            self._started = True
            # Check that real feedback is readable, without issuing a motion command.
            self.get_joint_positions()
        except Exception as error:
            self._faulted = True
            try:
                self.stop()
            except BackendError as cleanup:
                raise BackendError(f'Hardware startup failed: {error}; cleanup failed: {cleanup}') from error
            raise BackendError(f'Hardware startup failed: {error}') from error

    def _require_started(self, *, command=False):
        if not self._started or self._hand is None:
            raise BackendError('SharpaSdkHand is not started')
        if command and self._faulted:
            raise BackendError('Hardware fault latched; restart the hand node before resuming')

    def set_joint_positions(self, positions: Sequence[float]):
        if self.read_only:
            raise BackendError('Read-only session rejects all motion commands')
        self._require_started(command=True)
        canonical = validate_positions(positions)
        if self.control_mode == 'mit':
            self._set_mit_positions(canonical)
            return
        sdk_positions = [canonical[index] for index in SDK_TO_URDF_INDEX]
        try:
            self._status('set_joint_position', self._hand.set_joint_position(sdk_positions, self.interpolation))
        except Exception as error:
            self._trip(error)

    def _set_mit_positions(self, positions):
        current = self.get_joint_positions()  # Observe actual limits even before the next timer tick.
        if self.mit_limits.state != 'normal':
            raise ValueError(f'MIT {self.mit_limits.state}: {self.mit_limits.reason}; '
                             'use the default-pose recovery action')
        target = self.mit_limits.clip(positions)
        now = self._clock()
        if (self._mit_last_command_at is not None and
                now - self._mit_last_command_at > self._command_timeout):
            # A new publisher may arrive before the node's idle timer runs.
            self.on_command_timeout()
        if self._mit_last_command_at is None:
            # Initial/resumed commands take over at freshly measured pose.
            clipped_current = self.mit_limits.clip(current)
            if max(abs(q - p) for q, p in zip(target, clipped_current)) > self._mit_start_tolerance:
                raise ValueError('MIT takeover target too far from feedback; load current pose first')
            command = clipped_current
            now = self._clock()
        else:
            command = target
        self._write_mit(command)
        self._mit_last_command_at = now

    def _write_mit(self, command):
        try:
            self._status('set_mit_control', self._hand.set_mit_control(
                [command[index] for index in SDK_TO_URDF_INDEX], [0.0] * 22, [0.0] * 22))
        except Exception as error:
            self._trip(error)

    def begin_recovery(self):
        self._require_started(command=True)
        if self.mit_limits is None:
            raise BackendError('Recovery requires writable MIT mode')
        current = self.get_joint_positions()
        self.mit_limits.begin(current)
        self._mit_last_command_at = None
        return current

    def set_recovery_positions(self, positions):
        self._require_started(command=True)
        self._write_mit(self.mit_limits.recovery_target(positions))

    def cancel_recovery(self, reason):
        if self.mit_limits is not None and self.mit_limits.state == 'recovering':
            self.mit_limits.lock(reason)
        self._mit_last_command_at = None

    def finish_recovery(self, tolerance_rad=math.radians(2.0)):
        self._require_started(command=True)
        current = self.get_joint_positions()
        if max(abs(q) for q in current) > tolerance_rad:
            raise BackendError('Measured hand moved away from default pose before completion')
        self.mit_limits.finish(current)
        self._mit_last_command_at = None

    def fail_recovery(self, reason):
        self._trip(BackendError(reason))

    def get_joint_positions(self):
        self._require_started()
        try:
            status, degrees = self._hand.get_joint_position_degree()
            self._status('get_joint_position_degree', status)
            values = validate_positions(degrees)
            positions = [math.radians(values[index]) for index in URDF_TO_SDK_INDEX]
            if self.mit_limits is not None:
                self.mit_limits.observe(positions, self._clock())
            return positions
        except Exception as error:
            self._trip(error)

    def _trip(self, error):
        self._faulted = True
        if self.mit_limits is not None:
            self.mit_limits.state, self.mit_limits.reason = 'faulted', str(error)
        try:
            self.stop()
        except BackendError as cleanup:
            raise BackendError(f'{error}; cleanup failed: {cleanup}') from error
        raise BackendError(str(error)) from error

    def on_command_timeout(self):
        if self.control_mode == 'mit':
            self._require_started()
            # Normal publisher idle: keep feedback/session alive, issue no target.
            # Do not clear hardware faults or change gains/mode/motor enable.
            self.cancel_recovery('Command stream idle; recovery canceled')
            return
        # A stopped SDK session never auto-resumes on the next queued command.
        self._trip(BackendError('Hardware command timeout; SDK stopped and session closed'))

    def take_tactile_frames(self):
        return self._tactile.drain() if self._tactile is not None else ([], None)

    def stop(self):
        if self._tactile is not None:
            self._tactile.close()
        hand, manager = self._hand, self._manager
        attempted = self._connection_attempted
        self._hand = self._manager = None
        self._connection_attempted = self._started = False
        self._mit_last_command_at = None
        errors = []
        if hand is not None:
            try:
                self._bool('stop', hand.stop())
            except Exception as error:
                errors.append(str(error))
        if manager is not None and attempted:
            try:
                # Disconnect only our explicitly configured serial, even on partial startup.
                manager.disconnect(self.serial_number)
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise BackendError('; '.join(errors))
