import builtins
from unittest.mock import Mock

from launch import LaunchContext
import pytest
import yaml

from dual_sharpa_wave import launch_helpers


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
    monkeypatch.setattr(launch_helpers, '_hand_actions', construct)
    with pytest.raises(ValueError):
        launch_helpers._hardware_actions(config_context(tmp_path, serials))
    construct.assert_not_called()


def test_valid_config_pins_hardware_without_importing_sdk(tmp_path, monkeypatch):
    construct = Mock(return_value=['hardware-nodes'])
    monkeypatch.setattr(launch_helpers, '_hand_actions', construct)
    original = builtins.__import__
    def guard(name, *args, **kwargs):
        if name == 'sharpa' or name.startswith('sharpa.'):
            raise AssertionError('launch construction imported SDK')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guard)
    context = config_context(tmp_path, ('left', 'right'))
    assert launch_helpers._hardware_actions(context) == ['hardware-nodes']
    construct.assert_called_once_with(context, 'sharpa_sdk')
