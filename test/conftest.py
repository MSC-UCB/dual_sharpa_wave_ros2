"""Offline SDK test doubles. Never import the official SDK or discover devices."""

from types import SimpleNamespace

import pytest


class FakeSdk:
    def __init__(self):
        self.calls = []
        self.fail = None
        self.code = 7
        self.degrees = [float(i) for i in range(22)]
        self.devices = ['unrelated-device', 'RIGHT-SERIAL', 'LEFT-SERIAL']
        self.now = 0.0
        self.ControlMode = SimpleNamespace(POSITION='POSITION')
        self.ControlSource = SimpleNamespace(SDK='SDK')
        self.SharpaWaveManager = SimpleNamespace(get_instance=lambda: self)

    def clock(self):
        return self.now

    def sleep(self, duration):
        self.now += duration

    def record(self, operation, *args):
        self.calls.append((operation, *args))
        if self.fail == operation:
            raise RuntimeError(f'{operation} exception')

    def get_all_device_sn(self):
        self.record('discover')
        return self.devices

    def connect(self, serial):
        self.record('connect', serial)
        return self

    def disconnect(self, serial):
        self.record('disconnect', serial)

    def status(self, operation, *args):
        self.record(operation, *args)
        return SimpleNamespace(code=self.code if self.fail == operation + '_status' else 0,
                               message='injected status')

    def set_control_mode(self, value):
        return self.status('mode', value)

    def set_speed_coeff(self, value):
        return self.status('speed', value)

    def set_current_coeff(self, value):
        return self.status('current', value)

    def set_control_source(self, value):
        return self.status('source', value)

    def start(self):
        self.record('start')
        return self.fail != 'start_false'

    def stop(self):
        self.record('stop')
        return self.fail != 'stop_false'

    def set_joint_position(self, positions, interpolation):
        return self.status('write', list(positions), interpolation)

    def get_joint_position_degree(self):
        return self.status('read'), self.degrees.copy()


@pytest.fixture
def fake_sdk():
    return FakeSdk()
