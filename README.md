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

Build from the repository root to avoid mixing multiple `install/` trees:

```bash
cd ~/ws_fanuc/dual_sharpa_wave_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dual_sharpa_wave
source install/setup.bash
```

Check that the current installation is being used:

```bash
ros2 pkg prefix dual_sharpa_wave
ros2 pkg executables dual_sharpa_wave
```

## Mock tests

Run the core offline tests:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest -q \
  test/test_mock_hand.py \
  test/test_joint_validation.py \
  test/test_control_model.py \
  test/test_sdk_hand.py \
  test/test_hand_node.py
```

Start the mock hands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=172

ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=mock
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

The hardware configuration is [config/dual_sharpa_hardware.yaml](config/dual_sharpa_hardware.yaml). It contains the serial numbers confirmed by SDK discovery:

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

Before the first command, `Command timeout: no new target sent` is expected. The node waits for a command and does not close the hardware connection just because it is waiting.

## GUI control

Keep the hardware launch running and start the GUI in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run dual_sharpa_wave gui_control.py
```

The GUI enables interaction after it receives feedback from both hands. Click `Load Current Pose` first, adjust one joint by a small amount, and then click `Start Sending`. Closing the GUI or clicking `Stop Sending` does not return the hands to zero and is not an emergency stop.

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
python3 script/sharpa_wave_example.py
```

Useful overrides include `--range-scale 0.5`, `--cycles 1`, and
`--frequency 0.5`. The script waits for both feedback streams, holds the
starting pose for the skipped joints, and returns both hands to their starting
pose after the motion.

For a new real hand setup, first use the GUI for a single-side, single-joint, small-amplitude test. Then use the waveform tools. All positions must be within the URDF joint limits.

## Control semantics

- A command must contain 22 finite position values. NaN, Inf, and 21/23-value commands are rejected.
- An empty `name` array means canonical joint order. With names, the node validates and reorders the command.
- State comes from the backend's actual position readback; the last target is never used as fabricated feedback.
- The mock default command timeout is 0.5 seconds; the real hardware configuration sets it to `0.0`, so the SDK session remains connected until the launch/node is closed.
- Mock timeout keeps the target and continues publishing simulated position.
- With the provided real hardware configuration, no command timeout is enabled; closing the launch/node performs the SDK stop and disconnect.
- SDK `stop()` is not a validated physical emergency stop.

## Joint order

`dual_sharpa_wave/joint_names.py` is the single source of truth for canonical joint order. Each hand has 22 revolute joints. The SDK adapter converts between SDK order and ROS order and converts SDK degree feedback to radians.

For the full joint list and model information, see:

- [third_party/sharpa_models/README.md](third_party/sharpa_models/README.md)
- [docs/hardware.md](docs/hardware.md)
- [realrobotplan.md](realrobotplan.md)

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
- `config/dual_sharpa_hardware.yaml`: hardware serials and parameters.
- [docs/hardware.md](docs/hardware.md): SDK adapter details.
- [realrobotplan.md](realrobotplan.md): hardware investigation and implementation record.
