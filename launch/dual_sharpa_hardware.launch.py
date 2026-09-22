"""Explicit-serial hardware launch; no discovery-order binding or mock fallback."""

from dual_sharpa_wave.launch_helpers import hardware_launch_description


def generate_launch_description():
    return hardware_launch_description()
