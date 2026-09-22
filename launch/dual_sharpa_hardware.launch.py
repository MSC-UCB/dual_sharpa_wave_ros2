"""Deliberately unavailable until an explicitly reviewed hardware backend exists."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def _unavailable(context):
    raise RuntimeError(
        'Sharpa SDK hardware backend is not implemented or verified. '
        'Hardware launch is disabled; no SDK import, discovery or connection attempted. '
        'Use dual_sharpa_mock.launch.py.'
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=''),
        OpaqueFunction(function=_unavailable),
    ])
