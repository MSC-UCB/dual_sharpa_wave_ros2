"""Launch both Sharpa hands with either the mock or SDK backend."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _preview_description(share: Path, side: str) -> str:
    root = ET.parse(share / 'urdf' / f'{side}_sharpa_wave_with_flange.urdf').getroot()
    ET.SubElement(root, 'link', name='preview_world')
    joint = ET.SubElement(root, 'joint', name=f'preview_{side}_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='preview_world')
    ET.SubElement(joint, 'child', link=f'{side}_hand_flange')
    ET.SubElement(
        joint, 'origin',
        xyz=f'0 {0.15 if side == "left" else -0.15} 0',
        rpy='0 0 0',
    )
    return ET.tostring(root, encoding='unicode')


def _hand_actions(context, backend: str, config: Path):
    if not config.is_file():
        raise FileNotFoundError(f'config_file does not exist: {config}')
    rate = LaunchConfiguration('publish_rate_hz').perform(context)
    actions = []
    for side in ('left', 'right'):
        overrides = {'side': side, 'backend': backend}
        if rate:
            overrides['publish_rate_hz'] = float(rate)
        actions.append(Node(
            package='dual_sharpa_wave',
            executable='hand_node',
            namespace=f'sharpa/{side}_hand',
            name='hand_node',
            parameters=[str(config), overrides],
            output='screen',
        ))
    return actions


def _hardware_config(config: Path) -> None:
    data = yaml.safe_load(config.read_text())
    serials = []
    for side in ('left', 'right'):
        name = f'/sharpa/{side}_hand/hand_node'
        try:
            serial = data[name]['ros__parameters']['serial_number']
        except (KeyError, TypeError):
            raise ValueError(
                f'{name} requires serial_number in {config}'
            ) from None
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f'{name} requires an explicit serial_number')
        serials.append(serial.strip())
    if len(set(serials)) != 2:
        raise ValueError('left and right serial_number must be different')


def _preview_actions(context, share: Path):
    if LaunchConfiguration('use_rviz').perform(context).lower() != 'true':
        return []
    prefix = 'sharpa_preview/'
    actions = [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=f'sharpa/{side}_hand',
        name='preview_state_publisher',
        parameters=[{
            'robot_description': _preview_description(share, side),
            'frame_prefix': prefix,
        }],
        output='screen',
    ) for side in ('left', 'right')]
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='sharpa_preview_rviz',
        arguments=['-d', str(share / 'rviz/dual_sharpa.rviz')],
        output='screen',
    ))
    return actions


def _actions(context):
    share = Path(get_package_share_directory('dual_sharpa_wave'))
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in ('mock', 'sharpa_sdk'):
        raise ValueError("backend must be 'mock' or 'sharpa_sdk'")

    supplied_config = LaunchConfiguration('config_file').perform(context)
    if supplied_config:
        config = Path(supplied_config)
    else:
        filename = (
            'dual_sharpa_hardware.yaml'
            if backend == 'sharpa_sdk' else 'dual_sharpa_wave.yaml'
        )
        config = share / 'config' / filename

    if backend == 'sharpa_sdk':
        _hardware_config(config)
    return _hand_actions(context, backend, config) + _preview_actions(context, share)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='mock',
            choices=['mock', 'sharpa_sdk'],
            description='Use mock or real Sharpa SDK backend',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Optional YAML config; defaults according to backend',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='',
            description='Optional publish-rate override',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start RViz and both preview state publishers',
        ),
        OpaqueFunction(function=_actions),
    ])
