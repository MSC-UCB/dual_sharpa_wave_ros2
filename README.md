# Dual Sharpa Wave ROS 2

One ROS 2 Jazzy package, `dual_sharpa_wave`, containing the Python hand driver,
tools, models, and custom control interfaces. It uses `ament_cmake` to generate
the ROS interfaces and install the Python programs.

```text
packages/dual_sharpa_wave/
├── CMakeLists.txt
├── package.xml
├── action/RecoverDefault.action
├── srv/MitGains.srv
├── dual_sharpa_wave/          # Python driver and tools
├── script/                   # ros2 run entry points and source tools
└── launch/                   # Single-hand and dual-hand bringup
```

Clone this repository into your workspace's `src/` directory. The separate
`sharpa_control_interfaces` package is no longer needed. Existing launch and
`ros2 run` commands, topics, and SDK control behavior are preserved.

## Build

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dual_sharpa_wave
source install/setup.bash
```

Existing workspaces need a one-time cleanup of the old Python installation and
interface package. Follow the [migration guide](packages/dual_sharpa_wave/docs/migration.md)
before rebuilding; `--cmake-clean-cache` alone does not remove the old installation.

## Interface names

| Previous type | Current type |
|---|---|
| `sharpa_control_interfaces/action/RecoverDefault` | `dual_sharpa_wave/action/RecoverDefault` |
| `sharpa_control_interfaces/srv/MitGains` | `dual_sharpa_wave/srv/MitGains` |

The included clients and servers use the new types together. External clients
using the old types must update and restart; the types are not wire-compatible.
Ordinary `sensor_msgs/msg/JointState` hand commands are unchanged.

```bash
ros2 interface show dual_sharpa_wave/action/RecoverDefault
ros2 interface show dual_sharpa_wave/srv/MitGains
```

## Usage and tests

See the [driver guide](packages/dual_sharpa_wave/README.md) for hardware setup,
mock and real-hand launch commands, and tests. Relative source paths in that
guide refer to `packages/dual_sharpa_wave/` unless stated otherwise.

The driver guide also documents the [action and service](packages/dual_sharpa_wave/README.md#custom-control-interfaces).
