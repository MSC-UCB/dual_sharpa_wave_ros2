"""Start one Sharpa hand using the existing driver and configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _single(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    if prefix not in ('left_', 'right_'):
        raise ValueError('prefix must be left_ or right_')
    arguments = {name: LaunchConfiguration(name).perform(context) for name in (
        'backend', 'control_mode', 'config_file', 'publish_rate_hz',
        'mit_kp_ratio', 'mit_kd_ratio', 'read_only', 'use_rviz')}
    arguments['hands'] = prefix[:-1]
    return [GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dual_sharpa_wave'), 'launch', 'dual_sharpa.launch.py'])),
        launch_arguments=arguments.items(),
    )])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='left_', choices=['left_', 'right_']),
        DeclareLaunchArgument('backend', default_value='mock', choices=['mock', 'sharpa_sdk']),
        DeclareLaunchArgument('control_mode', default_value='', choices=['', 'position', 'mit'],
                              description='Blank preserves upstream backend/YAML mode'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('publish_rate_hz', default_value=''),
        DeclareLaunchArgument('mit_kp_ratio', default_value=''),
        DeclareLaunchArgument('mit_kd_ratio', default_value=''),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_single),
    ])
