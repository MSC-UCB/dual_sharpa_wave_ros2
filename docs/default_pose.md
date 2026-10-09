# Move both hands to the default pose

The default matches retargeting's CRX+Sharpa configuration: all 22 joints of each
hand are zero radians, in canonical URDF order. Both zero poses pass model limits.
This is a joint target, not an SDK calibration or encoder reset.

`script/move_to_default_pose.py` can be run directly with Python or, after
building this package, with `ros2 run dual_sharpa_wave move_to_default_pose.py`.
It uses the installed `dual_sharpa_wave` package. Source ROS and the workspace first:

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

The script reads both drivers' `control_mode`. In POSITION mode it reuses
`CommandClient`, sending complete named JointState targets to
`/sharpa/left_hand/joint_command` and `/sharpa/right_hand/joint_command`.
In MIT it requests each driver's `recover_default` action using
`dual_sharpa_wave/action/RecoverDefault`. Both goals must be
accepted before the client sends matching goal-ID heartbeats to start motion.
The drivers generate the zero trajectories and reject ordinary commands during
recovery. This is not an atomic or hardware-synchronized dual-hand move.
Both hands must use the same mode. The hands-only script does not control CRX arms
or change driver speed/current configuration.

Defaults:

- Minimum approach duration: 3 seconds, extended to meet target limits of
  10 degrees/s speed and 20 degrees/s² acceleration using quintic interpolation.
- Wait for both feedback streams and subscribers: up to 15 seconds.
- Feedback expiry: 0.5 seconds, measured by local receipt time.
- Arrival: each hand's 22 joints within 2 degrees of zero continuously for 1 second.
  MIT also requires actual positions within URDF limits plus 5 degrees.
- Settling timeout after the approach: 15 seconds.

The trajectory starts from original measured positions, including positions
outside the URDF range. The MIT adapter clips outgoing samples to the URDF range,
without changing raw feedback or encoder calibration. Initial clipping can cause
a target step; the quintic derivative bounds do not cover that step.
All positions must still contain 22 finite values. Invalid feedback,
stale feedback, lost subscribers, a stalled loop,
or a discovered competing command publisher stop new targets and produce a failure
exit. Publisher counting is not exclusive ownership and cannot detect SDK clients
or commands through other interfaces. The trajectory bounds apply to commanded
positions, not a guarantee of physical motor speed or acceleration.

Successful arrival exits with code 0; errors exit with 1; Ctrl+C exits with 130.
Exiting does not disable motors or emergency-stop the hands. With the provided
hardware config (`command_timeout_sec: 0.0`), the driver session remains connected
and the last command is retained. Other driver timeout settings may stop the SDK
session after sending ends. This script provides no collision or contact checking.

The MIT configuration treats normal command gaps as idle, retaining the SDK
session and live feedback. Actual feedback outside URDF limits plus 5 degrees
locks ordinary commands, but the explicit recovery action remains available.
Recovery takes over from fresh feedback even when the hand is limit-locked.
The 0.1 rad first-target check still applies to ordinary joint commands, not to
the dedicated driver-owned zero trajectory.

Normal completion unlocks MIT control and needs no driver restart. Ctrl+C,
client heartbeat loss for 0.5 s, or settling failure ends recovery without itself
locking ordinary control. Fresh measured overshoot beyond the 5-degree limit
tolerance still locks; otherwise ordinary control resumes with the existing
first-target check. The action still reports failure when zero was not reached. If one hand fails, the
client cancels the other's unfinished goal. A hand that already completed stays
unlocked. Locks/cancellation do not disable motors or clear the last MIT target.
Persistent worsening of limit violation during recovery and SDK faults close
and latch the session; these require a driver restart and investigation.
See the [control semantics](../README.md#control-semantics) and
[`dual_sharpa_mit.yaml`](../config/dual_sharpa_mit.yaml) for recovery configuration.

The action is generated as part of `dual_sharpa_wave`. Build and source the
workspace before use (see [migration](migration.md) for an existing installation):

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dual_sharpa_wave
source install/setup.bash
```

During settling/recovery the script prints, once per second, each hand's count of
joints outside tolerance and the worst joint's actual angle, target and error
in degrees (MIT reports the worst zero error). A settling timeout includes diagnostics; a startup timeout identifies
the side(s) without fresh feedback and a command subscriber. The default 2-degree
criterion is unchanged. In MIT, saved compliant gains do not guarantee the same
static position accuracy as POSITION mode. Inspect the residual errors before
changing gains or selecting a tolerance appropriate to the task.

Offline tests, without ROS or hardware:

```bash
env -u PYTHONPATH /home/msc-crx/retargeting_crx/.venv/bin/python -m pytest test/test_default_pose.py -q
```
