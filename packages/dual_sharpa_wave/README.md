# Dual Sharpa Wave ROS 2

This ROS 2 Jazzy package controls two Sharpa Wave hands. Each hand runs an independent `hand_node`, has 22 joints, and uses radians at the ROS interface.

Current status:

- The mock backend passes the offline tests.
- Sharpa SDK 5.0.10/build 5.0.10.6 is integrated.
- The left hand `CD52943BCD53` (`192.168.10.10`) and right hand `C956943BC957` (`192.168.10.20`) have been connected through the SDK.
- Both `joint_states` feedback streams work.
- The real hands have been controlled through the GUI.
- The ROS sample-style motion in `script/sharpa_wave_example.py` has completed successfully on both hands.

This package controls the Sharpa hands. It does not control the FANUC/CRX robot arm.

## Communication structure

```text
GUI / sine / step / manual ROS command
              ↓
      ROS 2 joint_command
              ↓
          hand_node
       ↙              ↘
   MockHand       SharpaSdkHand
              ↓
      ROS 2 joint_states
```

The two nodes are:

```text
/sharpa/left_hand/hand_node
/sharpa/right_hand/hand_node
```

Topics:

| Topic | Type | Description |
|---|---|---|
| `/sharpa/left_hand/joint_command` | `sensor_msgs/msg/JointState` | 22-joint left-hand target, radians |
| `/sharpa/left_hand/joint_states` | `sensor_msgs/msg/JointState` | Actual left-hand feedback, radians |
| `/sharpa/right_hand/joint_command` | `sensor_msgs/msg/JointState` | 22-joint right-hand target, radians |
| `/sharpa/right_hand/joint_states` | `sensor_msgs/msg/JointState` | Actual right-hand feedback, radians |

Both sides use RELIABLE, VOLATILE, KEEP_LAST, depth=1 QoS. Continuous control must keep publishing commands; a single command does not mean that the target has been reached.

## Build

Clone the `dual_sharpa_wave_ros2` repository under the workspace `src/`. Its
single package, `packages/dual_sharpa_wave`, includes the driver and custom
interfaces. CMake generates the interfaces and installs the Python programs.
For an existing installation, first follow the [migration guide](docs/migration.md).
Build from the workspace root:

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dual_sharpa_wave
source install/setup.bash
```

Check that the current installation is being used:

```bash
ros2 pkg prefix dual_sharpa_wave
ros2 pkg executables dual_sharpa_wave
ros2 interface show dual_sharpa_wave/action/RecoverDefault
ros2 interface show dual_sharpa_wave/srv/MitGains
```

With `--symlink-install`, edits to existing Python files normally take effect
after restarting the relevant node or tool. Adding files or changing installation
rules or `.action`/`.srv` definitions requires rebuilding.

## Custom control interfaces

These interfaces are part of `dual_sharpa_wave`; building them does not start a
driver or connect to hardware. The writable Sharpa SDK MIT backend exposes:

| Endpoint (`<side>` is `left` or `right`) | Type | Purpose |
|---|---|---|
| `/sharpa/<side>_hand/recover_default` | `dual_sharpa_wave/action/RecoverDefault` | Driver-owned trajectory to zero, with position/error feedback, completion and cancellation. |
| `/sharpa/<side>_hand/mit_gains` | `dual_sharpa_wave/srv/MitGains` | Read/apply the 22 joints' Kp/Kd, cancel a transition, or query status. |

`RecoverDefault` accepts duration, speed, acceleration, command rate, tolerance,
hold time and settling timeout; it does not accept arbitrary joint targets.
Use the [default-pose tool](docs/default_pose.md), which also handles the recovery
heartbeat. `MitGains` arrays use the driver's canonical joint order. Its `STATUS`
operation returns cached data without reading hardware. See [control semantics](#control-semantics)
for runtime gain tuning and recovery behavior.

Python clients import `RecoverDefault` from `dual_sharpa_wave.action` and
`MitGains` from `dual_sharpa_wave.srv`. The old `sharpa_control_interfaces` types
must be updated in external clients; see [migration](docs/migration.md).
Ordinary joint commands continue to use `sensor_msgs/msg/JointState`.

## Mock tests

Build and source the workspace first, then run tests from the workspace root.
Testing the installed package ensures generated interfaces are available:

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/dual_sharpa_wave_ros2/packages/dual_sharpa_wave/test -m 'not integration'
```

The full suite is registered with CMake, including software-only DDS integration
tests. It requires localhost ROS networking; the RViz checks require a usable
display/rendering environment. Tk GUI tests skip when no display is available:

```bash
colcon test --packages-select dual_sharpa_wave --event-handlers console_direct+
colcon test-result --test-result-base build/dual_sharpa_wave --verbose
```

Start the mock hands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=172

ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=mock
```

## Single-hand bringup

Use `single_sharpa.launch.py` to start only the selected hand. The source directory
is `dual_sharpa_wave_ros2/packages/dual_sharpa_wave`, while the ROS package name remains `dual_sharpa_wave`.
Source ROS and the workspace in every terminal.

```bash
# Left hand only, software mock
ros2 launch dual_sharpa_wave single_sharpa.launch.py prefix:=left_ backend:=mock

# Right hand only, software mock with a single-hand RViz preview
ros2 launch dual_sharpa_wave single_sharpa.launch.py \
  prefix:=right_ backend:=mock use_rviz:=true

# Feedback only, without a joint-command subscription
ros2 launch dual_sharpa_wave single_sharpa.launch.py \
  prefix:=left_ backend:=mock read_only:=true
```

`prefix` accepts only `left_` and `right_` (default `left_`). The launch defaults to
`backend:=mock`, `read_only:=false`, and `use_rviz:=false`. It forwards
`control_mode`, `config_file`, `publish_rate_hz`, `mit_kp_ratio`, and `mit_kd_ratio`
to the existing driver launch; empty overrides retain the backend/YAML defaults.
It starts no motion sender. Existing dual-hand launch defaults remain unchanged.

Only `/sharpa/<side>_hand/hand_node` is started. Its feedback contains the selected
22 joints. Hardware YAML may contain both hands or just the selected node's entry;
only the selected serial number is required. Launching both hands still requires
two distinct serial numbers. With `use_rviz:=true`, only the selected preview
state publisher and one hand model display are started.

After configuring the SDK environment below, select physical hardware explicitly:

```bash
ros2 launch dual_sharpa_wave single_sharpa.launch.py \
  prefix:=left_ backend:=sharpa_sdk read_only:=true

# Writable hardware; defaults to MIT unless overridden by control_mode or YAML
ros2 launch dual_sharpa_wave single_sharpa.launch.py \
  prefix:=left_ backend:=sharpa_sdk read_only:=false
```

Writable hardware startup configures the selected hand's mode and enables motors;
it does not wait for a motion sender to perform that configuration. The other
hand is not connected or configured by this launch.

The existing GUI can be started separately:

```bash
ros2 run dual_sharpa_wave gui_control.py
```

The connected side's panel becomes available independently. The absent side stays
waiting/disabled. The GUI still creates command publishers and feedback
subscriptions for both sides, so inactive-side topics may appear when the GUI is
running; they do not indicate an inactive-side driver. Sine/step waveform tools
retain their existing dual-hand behavior and are not started by this launch.

After building, run the installed-launch mock and preview checks explicitly:

```bash
SINGLE_SIDE_ROS_MOCK=1 python3 -m pytest -q \
  src/dual_sharpa_wave_ros2/packages/dual_sharpa_wave/test/test_single_hand_launch.py
```

## Sharpa SDK environment

The terminal running the real hardware nodes must have the SDK paths configured:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH="/opt/sharpa-wave-sdk/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/sharpa-wave-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

Verify that the current Python can load the SDK:

```bash
python3 -c 'import sharpa; print(sharpa.__file__)'
```

ROS and the SDK both use Python 3.12. There is no need to copy the SDK into the ROS installation or install ROS into the SDK virtual environment. Each new terminal needs the two `export` commands; the import check is only needed after changing the environment.

## Real hardware configuration

Real hardware defaults to MIT using [config/dual_sharpa_mit.yaml](config/dual_sharpa_mit.yaml).
The legacy POSITION configuration remains available as
[config/dual_sharpa_hardware.yaml](config/dual_sharpa_hardware.yaml).
Both contain the serial numbers confirmed by SDK discovery:

```yaml
# left:  CD52943BCD53, 192.168.10.10
# right: C956943BC957, 192.168.10.20
```

If the hands or computer are changed, update the corresponding `serial_number`. The two serial numbers must be different and must match the physical hands.

## Start the real hands

After setting up the SDK environment:

```bash
export ROS_LOG_DIR=/tmp/dual_sharpa_wave_logs
ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=sharpa_sdk use_rviz:=false
```

This starts MIT control using the fixed tuning baseline multiplied by
`mit_kp_ratio` and `mit_kd_ratio` (each defaults to `0.6`, shared by both hands). Each hand writes both gain arrays once before
control starts, unless readback already matches. Override with
`mit_kp_ratio:=0.6 mit_kd_ratio:=0.8`; `1.0` restores the original tuning baseline. To explicitly select
POSITION, add `control_mode:=position`. An explicit `config_file` keeps its own
settings unless you also supply a control-mode override. The mock backend remains
POSITION by default. Starting writable hardware control configures the mode and
enables motors; use `read_only:=true` for feedback-only operation.

Normal logs include:

```text
left hand ready: backend=sharpa_sdk, ... 22 joints in radians
right hand ready: backend=sharpa_sdk, ... 22 joints in radians
```

Use another terminal to inspect feedback:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /sharpa/left_hand/joint_states sensor_msgs/msg/JointState
ros2 topic echo /sharpa/right_hand/joint_states sensor_msgs/msg/JointState
```

Before the first command, `Waiting for first joint command; no target sent` is expected. The node waits for a command and does not close the hardware connection just because it is waiting.

## Tactile viewer (OpenCV)

After rebuilding the package and sourcing ROS/workspace in both terminals,
configure the SDK environment in terminal 1 and run:

```bash
# Terminal 1: read joint feedback and tactile data without accepting motion commands.
ros2 launch dual_sharpa_wave dual_sharpa.launch.py \
  backend:=sharpa_sdk read_only:=true use_rviz:=false

# Terminal 2: open the ROS subscriber viewer.
ros2 run dual_sharpa_wave tactile_viewer.py
```

`read_only` is a launch argument (default: `false`). Setting it to `true`
disables the joint-command subscription, rejects motion targets, and skips SDK
joint control configuration. Sensing startup still initializes ports and time
synchronization; it does not automatically calibrate the sensors.

The viewer shows RAW, deformation and force-magnitude heatmaps for all ten
fingers. `NO DATA` / `STALE` indicate missing or outdated images. Use
`--force-max 15` to increase the fixed heatmap scale (default: 1 SDK unit;
physical units are unconfirmed). Press `q` / `Esc` or close the window to exit;
the launch remains running. The viewer never connects to the SDK directly.

See the [read-only tactile test results](docs/tactile_read_test_2026-09-29.json)
and [real-data preview](docs/images/tactile_live_2026-09-29.png).

## GUI control

Keep the hardware launch running and start the GUI in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run dual_sharpa_wave gui_control.py
```

The GUI enables each panel independently after it receives valid feedback and finds that hand's command subscriber. An offline hand does not block the other panel. Click `Load Current Pose` first, adjust one joint by a small amount, and then click `Start Sending`. Closing the GUI or clicking `Stop Sending` does not return the hands to zero and is not an emergency stop.

## Waveform control

Do not run multiple tools that publish to the same command topic. Sine example:

```bash
ros2 run dual_sharpa_wave sine_control.py \
  --axes thumb_CMC_FE,index_MCP_FE,middle_MCP_FE \
  --amplitude 0.1 --frequency 0.25 --cycles 2 --repeat 1
```

Step example:

```bash
ros2 run dual_sharpa_wave step_control.py \
  --axes thumb_CMC_FE,index_MCP_FE,middle_MCP_FE \
  --step-size 0.1 --base-seconds 1 --high-seconds 1 \
  --cycles 2 --repeat 1
```

The SDK-sample-style motion for both hands is available at
`script/sharpa_wave_example.py`. It follows the sample's joint angle ranges,
three 1 Hz sine cycles, and skipped MCP joints, but publishes through the ROS
topics instead of calling the SDK directly:

```bash
python3 ~/ws_fanuc/src/dual_sharpa_wave_ros2/packages/dual_sharpa_wave/script/sharpa_wave_example.py
```

Useful overrides include `--range-scale 0.5`, `--cycles 1`, and
`--frequency 0.5`. The script waits for both feedback streams, holds the
starting pose for the skipped joints, and returns both hands to their starting
pose after the motion.

For a new real hand setup, first use the GUI for a single-side, single-joint, small-amplitude test. Then use the waveform tools. These GUI/waveform tools require positions within the URDF joint limits. The MIT adapter clips outgoing targets to those limits while preserving raw feedback.

## Control semantics

MIT is the default when launching `backend:=sharpa_sdk` without a custom YAML.
This selects `config/dual_sharpa_mit.yaml`, applies fixed baseline gains ×
separate `mit_kp_ratio` / `mit_kd_ratio` values once at startup (both default 0.6, no compounding on restart), and
sends positions through `set_mit_control` with zero target velocity and zero
feedforward torque. See [real hardware configuration](#real-hardware-configuration)
and [default-pose operation](docs/default_pose.md).
Hardware motion through this ROS path has not yet been tested; both hands' saved
gains and current MIT mode were verified through read-only SDK connections.

For explicit runtime gain edits, run
`ros2 run dual_sharpa_wave adjust_mit_gains_gui.py --side left` (or `right`),
click **Read device gains**, edit Kp/Kd or use **Kp ratio** and **Kd ratio**
(each defaults to 1.0), then **Apply**. Startup and GUI share the hard-coded tuning
baseline in `mit_gains.py`. Read updates device values only; GUI ratio 1 always
means the original baseline, even after startup at 0.6. The driver schedules
two steps (50%, then 100%) on a 200 ms timer. Measured SDK writes take about
300 ms, giving an estimated total of 0.8–0.9 seconds, not a guaranteed deadline.
This minimal synchronous tool pauses ROS callbacks; use it for stationary tuning. Same-value writes on both
hands were verified.

- A command must contain 22 finite position values. NaN, Inf, and 21/23-value commands are rejected.
- An empty `name` array means canonical joint order. With names, the node validates and reorders the command.
- State comes from the backend's actual position readback; the last target is never used as fabricated feedback.
- The mock default command timeout is 0.5 seconds; the position-mode hardware configuration sets it to `0.0`, so the SDK session remains connected until the launch/node is closed.
- MIT treats a 0.5 s command gap as idle: the SDK session and feedback remain active. The next command reacquires the measured pose with the initial 0.1 rad target check; normal pauses do not require a restart. Idle does not disable motors or clear the last MIT target. SDK faults still close and latch the session.
- Actual MIT feedback outside the model by more than 5 degrees latches ordinary control without disconnecting. Run the existing default-pose script with `--execute` to request the dedicated recovery action. Successful recovery unlocks control. Cancellation, heartbeat loss or settling failure ends recovery; ordinary control resumes if the measured pose is within the 5-degree limit tolerance. Only measured overshoot causes limit locking. Persistent worsening during recovery or SDK errors close the session. See [recovery behavior](docs/default_pose.md).
- Mock commands update the reported joint positions immediately; there is no mock joint speed limit.
- Mock timeout keeps the last position and continues publishing it.
- With the provided position-mode hardware configuration, no command timeout is enabled; closing the launch/node performs the SDK stop and disconnect.
- SDK `stop()` is not a validated physical emergency stop.

## Joint order

`dual_sharpa_wave/joint_names.py` is the single source of truth for canonical joint order. Each hand has 22 revolute joints. The SDK adapter converts between SDK order and ROS order and converts SDK degree feedback to radians.

For the full joint list and model information, see:

- [third_party/sharpa_models/README.md](third_party/sharpa_models/README.md)
- [dual_sharpa_wave/joint_names.py](dual_sharpa_wave/joint_names.py)

## RViz preview

Start the RViz preview with the mock backend:

```bash
ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=mock use_rviz:=true
```

RViz is only a model preview. It is not a physics simulator or a CRX flange calibration.

## Related files

- `dual_sharpa_wave/hand_node.py`: ROS node and backend scheduling.
- `dual_sharpa_wave/mock_hand.py`: mock backend.
- `dual_sharpa_wave/sharpa_sdk_hand.py`: Sharpa SDK adapter.
- `launch/dual_sharpa.launch.py`: unified mock/hardware launch file; switch with `backend:=mock` or `backend:=sharpa_sdk`.
- `config/dual_sharpa_mit.yaml`: default hardware MIT serials and parameters.
- `config/dual_sharpa_hardware.yaml`: explicit legacy POSITION configuration.
- [docs/default_pose.md](docs/default_pose.md): default-pose operation and recovery behavior.
- [docs/migration.md](docs/migration.md): migration from the separate interface package.
