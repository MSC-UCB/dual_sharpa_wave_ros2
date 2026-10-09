# Migrate to the single Sharpa package

`dual_sharpa_wave` now includes the Python driver, `RecoverDefault.action`, and
`MitGains.srv`. Its build type changed from `ament_python` to `ament_cmake`.
The source package remains at `src/dual_sharpa_wave_ros2/packages/dual_sharpa_wave`.

## Interface compatibility

| Previous type | Current type |
|---|---|
| `sharpa_control_interfaces/action/RecoverDefault` | `dual_sharpa_wave/action/RecoverDefault` |
| `sharpa_control_interfaces/srv/MitGains` | `dual_sharpa_wave/srv/MitGains` |

Update external Python imports to:

```python
from dual_sharpa_wave.action import RecoverDefault
from dual_sharpa_wave.srv import MitGains
```

Update any CLI commands, C++ clients, or other workspaces that name the old types.
The old and new ROS types are distinct even though their fields are unchanged;
an import alias alone does not preserve wire compatibility. Recordings containing
the old custom types also need separate compatibility handling if replayed.

The included driver and tools already use the new types. Restart them together.
Launch arguments, all eight `ros2 run` executable names, topic names, ordinary
`sensor_msgs/msg/JointState` commands, and SDK control behavior are preserved.
The bimanual package continues finding the driver and models by `dual_sharpa_wave`.

## Existing workspace

Stop the old driver and tools before replacing their installation. Preserve local
source edits, then update this repository. Move any separate
`src/sharpa_control_interfaces` checkout outside the source tree; the updated repo
must contain only one ROS package named `dual_sharpa_wave`.

Use a fresh terminal that has not sourced the old workspace (disable automatic
sourcing for this terminal if configured). Back up the two packages' old build
and install directories. No other package artifacts need to be removed:

```bash
cd ~/ws_fanuc
mkdir -p .repo_backups
touch .repo_backups/COLCON_IGNORE
migration_backup=$(mktemp -d "$PWD/.repo_backups/sharpa-install.XXXXXX")
for relative in build/dual_sharpa_wave install/dual_sharpa_wave \
                build/sharpa_control_interfaces install/sharpa_control_interfaces; do
  if [ -e "$relative" ]; then
    mkdir -p "$migration_backup/$(dirname "$relative")"
    mv "$relative" "$migration_backup/$relative"
  fi
done

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dual_sharpa_wave
source install/setup.bash
```

For an arm-and-hand workspace, use `--packages-up-to bimanual_manipulation`
instead. `--cmake-clean-cache` alone cannot remove the old Python installation
or retired interface package. Rebuild before opening other terminals with the
workspace setup; close shells that still retain the old environment.

## Verify

From the workspace root after building and sourcing:

```bash
colcon list --base-paths src/dual_sharpa_wave_ros2
ros2 pkg prefix dual_sharpa_wave
ros2 pkg executables dual_sharpa_wave
ros2 interface show dual_sharpa_wave/action/RecoverDefault
ros2 interface show dual_sharpa_wave/srv/MitGains
python3 -c 'from dual_sharpa_wave.action import RecoverDefault; from dual_sharpa_wave.srv import MitGains; from dual_sharpa_wave import hand_node; print(hand_node.__file__)'
python3 -m pytest -q src/dual_sharpa_wave_ros2/packages/dual_sharpa_wave/test -m 'not integration'
```

Package discovery should list only `dual_sharpa_wave` with type `ros.ament_cmake`.
The executables remain `hand_node`, `tactile_viewer.py`, `gui_control.py`,
`adjust_mit_gains_gui.py`, `sine_control.py`, `step_control.py`,
`move_to_default_pose.py`, and `read_mit_settings.py`.

Use the [driver guide](../README.md#mock-tests) for the full suite and mock launch
checks. Run Python tests from the workspace root after sourcing the installation;
putting the source Python package first on `PYTHONPATH` can hide its generated
`action` and `srv` subpackages.
