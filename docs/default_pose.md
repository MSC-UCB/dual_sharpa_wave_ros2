# Move both hands to the default pose

The default matches retargeting's CRX+Sharpa configuration: all 22 joints of each
hand are zero radians, in canonical URDF order. Both zero poses pass model limits.
This is a joint target, not an SDK calibration or encoder reset.

`script/move_to_default_pose.py` is a source-only script; run it with Python,
not `ros2 run`. It uses the existing installed `dual_sharpa_wave` package and
does not require a rebuild. Source ROS and the workspace first:

```bash
source /opt/ros/jazzy/setup.bash
source /home/msc-crx/ws_fanuc/install/setup.bash
cd /home/msc-crx/ws_fanuc/src/dual_sharpa_wave_ros2
python3 script/move_to_default_pose.py
```

The above only prints targets; it creates no ROS node and opens no SDK connection.
With both hand drivers already running, stop teleop, GUI sending, and other hand
command publishers, clear the fingers' path, then execute:

```bash
python3 script/move_to_default_pose.py --execute --rate 100
```

The script reuses `CommandClient`, sending complete named JointState targets to
`/sharpa/left_hand/joint_command` and `/sharpa/right_hand/joint_command`. The two
messages share a timestamp but are not an atomic or hardware-synchronized command.
It does not control the CRX arms or change driver speed/current configuration.

Defaults:

- Minimum approach duration: 3 seconds, extended to meet target limits of
  10 degrees/s speed and 20 degrees/s² acceleration using quintic interpolation.
- Wait for both feedback streams and subscribers: up to 15 seconds.
- Feedback expiry: 0.5 seconds, measured by local receipt time.
- Arrival: all 44 joints within 2 degrees of zero continuously for 1 second.
- Settling timeout after the approach: 15 seconds.

Start, target, feedback, and published samples are checked against the installed
URDF limits. Invalid feedback, stale feedback, lost subscribers, a stalled loop,
or a discovered competing command publisher stop new targets and produce a failure
exit. Publisher counting is not exclusive ownership and cannot detect SDK clients
or commands through other interfaces. The trajectory bounds apply to commanded
positions, not a guarantee of physical motor speed or acceleration.

Successful arrival exits with code 0; errors exit with 1; Ctrl+C exits with 130.
Exiting does not disable motors or emergency-stop the hands. With the provided
hardware config (`command_timeout_sec: 0.0`), the driver session remains connected
and the last command is retained. Other driver timeout settings may stop the SDK
session after sending ends. This script provides no collision or contact checking.

Offline tests, without ROS or hardware:

```bash
env -u PYTHONPATH /home/msc-crx/retargeting_crx/.venv/bin/python -m pytest test/test_default_pose.py -q
```
