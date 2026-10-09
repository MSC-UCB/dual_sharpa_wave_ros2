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


def test_mit_launch_selects_config_and_one_hand(monkeypatch):
    share = LAUNCH_PATH.parents[1]
    monkeypatch.setattr(launch_file, 'get_package_share_directory', lambda name: str(share))
    construct = Mock(side_effect=lambda **kwargs: kwargs)
    monkeypatch.setattr(launch_file, 'Node', construct)
    ctx = LaunchContext()
    ctx.launch_configurations.update(backend='sharpa_sdk', control_mode='mit', hands='left',
                                     config_file='', publish_rate_hz='', read_only='false', use_rviz='false')
    actions = launch_file._actions(ctx)
    assert len(actions) == 1
    assert actions[0]['namespace'] == 'sharpa/left_hand'
    path, overrides = actions[0]['parameters']
    assert Path(path).name == 'dual_sharpa_mit.yaml'
    assert overrides['control_mode'] == 'mit'
    config = yaml.safe_load(Path(path).read_text())
    for side in ('left', 'right'):
        params = config[f'/sharpa/{side}_hand/hand_node']['ros__parameters']
        assert params['control_mode'] == 'mit'
        assert params['interpolation'] is False
        assert params['mit_kp_ratio'] == params['mit_kd_ratio'] == .6
        assert params['command_timeout_sec'] > 0


def test_mode_from_custom_yaml_is_not_overridden(tmp_path, monkeypatch):
    ctx = config_context(tmp_path, ('left', 'right'))
    ctx.launch_configurations.update(publish_rate_hz='', read_only='true', hands='right')
    monkeypatch.setattr(launch_file, 'Node', lambda **kwargs: kwargs)
    actions = launch_file._hand_actions(ctx, 'sharpa_sdk', Path(ctx.launch_configurations['config_file']))
    assert len(actions) == 1
    overrides = actions[0]['parameters'][1]
    assert overrides['side'] == 'right'
    assert overrides['read_only'] is True
    assert 'control_mode' not in overrides
    assert 'mit_kp_ratio' not in overrides and 'mit_kd_ratio' not in overrides


def test_gain_ratio_override_reaches_both_nodes(monkeypatch):
    ctx = LaunchContext()
    ctx.launch_configurations.update(publish_rate_hz='', mit_kp_ratio='0.6', mit_kd_ratio='0.8')
    monkeypatch.setattr(launch_file, 'Node', lambda **kwargs: kwargs)
    actions = launch_file._hand_actions(ctx, 'sharpa_sdk', LAUNCH_PATH.parents[1] / 'config/dual_sharpa_mit.yaml')
    assert len(actions) == 2
    assert all(action['parameters'][1]['mit_kp_ratio'] == .6 for action in actions)
    assert all(action['parameters'][1]['mit_kd_ratio'] == .8 for action in actions)


@pytest.mark.parametrize('backend,mode,filename', [
    ('sharpa_sdk', '', 'dual_sharpa_mit.yaml'),
    ('sharpa_sdk', 'position', 'dual_sharpa_hardware.yaml'),
    ('mock', '', 'dual_sharpa_wave.yaml'),
])
def test_default_and_explicit_mode_choose_matching_config(monkeypatch, backend, mode, filename):
    share = LAUNCH_PATH.parents[1]
    monkeypatch.setattr(launch_file, 'get_package_share_directory', lambda name: str(share))
    monkeypatch.setattr(launch_file, 'Node', lambda **kwargs: kwargs)
    ctx = LaunchContext()
    ctx.launch_configurations.update(backend=backend, control_mode=mode, config_file='',
                                     publish_rate_hz='', use_rviz='false', read_only='false')
    actions = launch_file._actions(ctx)
    assert len(actions) == 2
    assert all(Path(action['parameters'][0]).name == filename for action in actions)
    if backend == 'sharpa_sdk':
        data = yaml.safe_load((share / 'config' / filename).read_text())
        for side in ('left', 'right'):
            params = data[f'/sharpa/{side}_hand/hand_node']['ros__parameters']
            expected = mode or 'mit'
            assert params['control_mode'] == expected
            assert params['interpolation'] is (expected == 'position')


def test_explicit_legacy_yaml_preserves_position_mode(monkeypatch):
    share = LAUNCH_PATH.parents[1]
    monkeypatch.setattr(launch_file, 'get_package_share_directory', lambda name: str(share))
    monkeypatch.setattr(launch_file, 'Node', lambda **kwargs: kwargs)
    ctx = LaunchContext()
    config = share / 'config/dual_sharpa_hardware.yaml'
    ctx.launch_configurations.update(backend='sharpa_sdk', control_mode='', config_file=str(config),
                                     publish_rate_hz='', use_rviz='false', read_only='false')
    actions = launch_file._actions(ctx)
    assert all(action['parameters'][0] == str(config) for action in actions)
    assert all('control_mode' not in action['parameters'][1] for action in actions)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_selected_serial_config_and_preview(tmp_path, monkeypatch, side):
    share = LAUNCH_PATH.parents[1]
    path = tmp_path / 'single.yaml'
    path.write_text(yaml.safe_dump({f'/sharpa/{side}_hand/hand_node': {
        'ros__parameters': {'serial_number': 'SELECTED', 'control_mode': 'position'}}}))
    ctx = LaunchContext()
    ctx.launch_configurations.update(backend='sharpa_sdk', hands=side, config_file=str(path),
                                     publish_rate_hz='', read_only='true', use_rviz='true')
    monkeypatch.setattr(launch_file, 'get_package_share_directory', lambda name: str(share))
    monkeypatch.setattr(launch_file, 'Node', lambda **kwargs: kwargs)
    actions = launch_file._actions(ctx)
    assert [a['executable'] for a in actions] == ['hand_node', 'robot_state_publisher', 'rviz2']
    assert actions[0]['namespace'] == actions[1]['namespace'] == f'sharpa/{side}_hand'
    assert actions[0]['parameters'][1]['read_only'] is True
    assert 'control_mode' not in actions[0]['parameters'][1]
    assert actions[2]['arguments'][-1].endswith('/single_sharpa.rviz')
    assert actions[2]['remappings'] == [('/single_hand_description', f'/sharpa/{side}_hand/robot_description')]
    other = 'right' if side == 'left' else 'left'
    with pytest.raises(ValueError, match='serial_number'):
        launch_file._hardware_config(path, (other,))
    with pytest.raises(ValueError, match='serial_number'):
        launch_file._hardware_config(path)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('serial', ['', '  ', None, 123])
def test_selected_serial_must_be_explicit(tmp_path, side, serial):
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump({f'/sharpa/{side}_hand/hand_node': {
        'ros__parameters': {'serial_number': serial}}}))
    with pytest.raises(ValueError, match='serial_number'):
        launch_file._hardware_config(path, (side,))


def test_single_preview_has_one_description_topic():
    config = yaml.safe_load((LAUNCH_PATH.parents[1] / 'rviz/single_sharpa.rviz').read_text())
    models = [d for d in config['Visualization Manager']['Displays']
              if d['Class'] == 'rviz_default_plugins/RobotModel']
    assert len(models) == 1
    assert models[0]['Description Topic']['Value'] == '/single_hand_description'


@pytest.mark.parametrize('prefix', ['left_', 'right_', 'other', 'left', ''])
def test_single_wrapper_pins_selection(monkeypatch, prefix):
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    spec = importlib.util.spec_from_file_location('single_sharpa_launch', LAUNCH_PATH.with_name('single_sharpa.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ctx = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = perform_substitutions(ctx, entity.default_value)
    ctx.launch_configurations.update(prefix=prefix, hands='both', read_only='true', mit_kp_ratio='0.7')
    monkeypatch.setattr(module, 'GroupAction', lambda actions: actions)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', lambda source, launch_arguments: dict(launch_arguments))
    if prefix not in ('left_', 'right_'):
        with pytest.raises(ValueError, match='prefix'):
            module._single(ctx)
        return
    arguments = module._single(ctx)[0][0]
    assert arguments['hands'] == prefix[:-1]
    assert arguments['backend'] == 'mock'
    assert arguments['read_only'] == 'true'
    assert arguments['mit_kp_ratio'] == '0.7'
    assert arguments['control_mode'] == ''
