"""Shared launch construction; the selected launch pins the backend and side."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .visualization import preview_description


def _hand_actions(context, backend):
    config = LaunchConfiguration('config_file').perform(context)
    if not Path(config).is_file():
        raise FileNotFoundError(f'config_file does not exist: {config}')
    rate = LaunchConfiguration('publish_rate_hz').perform(context)
    actions = []
    for side in ('left', 'right'):
        overrides = {'side': side, 'backend': backend}
        if rate:
            overrides['publish_rate_hz'] = float(rate)
        actions.append(Node(
            package='dual_sharpa_wave', executable='hand_node',
            namespace=f'sharpa/{side}_hand', name='hand_node',
            parameters=[config, overrides], output='screen',
        ))
    return actions


def _preview_actions(context):
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    if not use_rviz:
        return []
    share = Path(get_package_share_directory('dual_sharpa_wave'))
    prefix = 'sharpa_preview/'
    actions = [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=f'sharpa/{side}_hand', name='preview_state_publisher',
        parameters=[{
            'robot_description': preview_description(share, side),
            'frame_prefix': prefix,
        }], output='screen',
    ) for side in ('left', 'right')]
    actions.append(Node(
        package='rviz2', executable='rviz2', name='sharpa_preview_rviz',
        arguments=['-d', str(share / 'rviz/dual_sharpa.rviz')], output='screen',
    ))
    return actions


def mock_launch_description():
    share = Path(get_package_share_directory('dual_sharpa_wave'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(share / 'config/dual_sharpa_wave.yaml')),
        DeclareLaunchArgument('publish_rate_hz', default_value='', description='Optional rate override'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz and both preview robot state publishers',
        ),
        OpaqueFunction(function=lambda context: _hand_actions(context, 'mock')),
        OpaqueFunction(function=_preview_actions),
    ])
