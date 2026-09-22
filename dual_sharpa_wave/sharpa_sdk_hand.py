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
                 clock=time.monotonic, sleep=time.sleep):
        if not isinstance(serial_number, str) or not serial_number.strip():
            raise ValueError('sharpa_sdk requires an explicit nonempty serial_number')
        for name, value in (('speed_coeff', speed_coeff), ('current_coeff', current_coeff)):
            if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')
        if not isinstance(interpolation, bool):
            raise ValueError('interpolation must be bool')
        if not math.isfinite(discovery_timeout_sec) or discovery_timeout_sec <= 0:
            raise ValueError('sdk_discovery_timeout_sec must be finite and positive')
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
            for method, value in (
                ('set_control_mode', sdk.ControlMode.POSITION),
                ('set_speed_coeff', self.speed_coeff),
                ('set_current_coeff', self.current_coeff),
                ('set_control_source', sdk.ControlSource.SDK),
            ):
                self._status(method, getattr(self._hand, method)(value))
            self._bool('start', self._hand.start())
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
        self._require_started(command=True)
        canonical = validate_positions(positions)
        sdk_positions = [canonical[index] for index in SDK_TO_URDF_INDEX]
        try:
            self._status('set_joint_position', self._hand.set_joint_position(sdk_positions, self.interpolation))
        except Exception as error:
            self._trip(error)

    def get_joint_positions(self):
        self._require_started()
        try:
            status, degrees = self._hand.get_joint_position_degree()
            self._status('get_joint_position_degree', status)
            values = validate_positions(degrees)
            return [math.radians(values[index]) for index in URDF_TO_SDK_INDEX]
        except Exception as error:
            self._trip(error)

    def _trip(self, error):
        self._faulted = True
        try:
            self.stop()
        except BackendError as cleanup:
            raise BackendError(f'{error}; cleanup failed: {cleanup}') from error
        raise BackendError(str(error)) from error

    def on_command_timeout(self):
        # A stopped SDK session never auto-resumes on the next queued command.
        self._trip(BackendError('Hardware command timeout; SDK stopped and session closed'))

    def stop(self):
        hand, manager = self._hand, self._manager
        attempted = self._connection_attempted
        self._hand = self._manager = None
        self._connection_attempted = self._started = False
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
