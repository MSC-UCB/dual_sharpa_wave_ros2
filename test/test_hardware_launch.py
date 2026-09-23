import builtins
import importlib.util
from pathlib import Path
from unittest.mock import Mock

from launch import LaunchContext
import pytest
import yaml


LAUNCH_PATH = Path(__file__).parents[1] / 'launch' / 'dual_sharpa.launch.py'
_SPEC = importlib.util.spec_from_file_location('dual_sharpa_launch', LAUNCH_PATH)
launch_file = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(launch_file)


def config_context(tmp_path, serials):
    path = tmp_path / 'hardware.yaml'
    path.write_text(yaml.safe_dump({f'/sharpa/{s}_hand/hand_node': {
        'ros__parameters': {'serial_number': sn}}
        for s, sn in zip(('left', 'right'), serials)}))
    context = LaunchContext()
    context.launch_configurations['config_file'] = str(path)
    return context


@pytest.mark.parametrize('serials', [('', 'right'), ('left', ''), ('same', ' same '), (123, 'right')])
def test_hardware_config_fails_before_any_node_action(tmp_path, monkeypatch, serials):
    construct = Mock()
    monkeypatch.setattr(launch_file, '_hand_actions', construct)
    with pytest.raises(ValueError):
        launch_file._hardware_config(Path(config_context(tmp_path, serials).launch_configurations['config_file']))
    construct.assert_not_called()


def test_valid_config_pins_hardware_without_importing_sdk(tmp_path, monkeypatch):
    construct = Mock(return_value=['hardware-nodes'])
    monkeypatch.setattr(launch_file, '_hand_actions', construct)
    original = builtins.__import__
    def guard(name, *args, **kwargs):
        if name == 'sharpa' or name.startswith('sharpa.'):
            raise AssertionError('launch construction imported SDK')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guard)
    config = Path(config_context(tmp_path, ('left', 'right')).launch_configurations['config_file'])
    launch_file._hardware_config(config)
    construct.assert_not_called()
