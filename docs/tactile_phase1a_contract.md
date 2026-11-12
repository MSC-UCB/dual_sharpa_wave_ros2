# Tactile frame contract, version 1

One message contains the available F6, CONTACT_POINT and DIST_FORCE blocks
from **one SDK callback for one fingertip**. It does not synchronize all five
fingers into one sample. Physical interpretation is deliberately unspecified.

Implementation and software validation status are tracked in
[the validation report](tactile_phase1a_validation.md). The old ten-finger
capture in [the baseline report](tactile_phase1a_baseline_validation_2026-11-11.md)
does not verify the new transport.

## Topics and mapping

Type: `dual_sharpa_wave/msg/TactileFrame`. Definitions:
[TactileFrame.msg](../msg/TactileFrame.msg), [TactileBlock.msg](../msg/TactileBlock.msg).

| Finger | Right channel | Left channel |
| --- | --- | --- |
| pinky | 0 | 5 |
| ring | 1 | 6 |
| middle | 2 | 7 |
| index | 3 | 8 |
| thumb | 4 | 9 |

Absolute topic: `/sharpa/<side>_hand/tactile/<finger>/frame` under the standard
launch namespaces. `side` is `right` or `left`; unsupported channels are rejected.
The installed SDK `include/touch.h` agrees with this canonical mapping.

The new publisher uses BEST_EFFORT, KEEP_LAST, depth 1, VOLATILE. Subscribe
with best-effort reliability. `tactile_enabled=false` creates no tactile
publishers, including frame publishers. The existing default is false in
the node; the hardware YAML files enable it and request 30 Hz.

All legacy `f6`, `raw`, `deform`, `deform3`, `deform3_se` and `dist_force`
topics retain their types, values/conversions, encodings, naming and QoS.
Legacy F6 remains `Float64MultiArray` without a header. Legacy images retain
host ROS receipt headers. No command, watchdog, gain or safety paths are changed.

## Frame fields and clocks

| Fields | Types | Meaning |
| --- | --- | --- |
| `schema_version` | uint16 | 1 |
| `source`, `side`, `finger`, `channel` | strings, uint8 | `sharpa_sdk`, canonical identity |
| `header` | std_msgs/Header | ROS clock at callback receipt; `frame_id` deliberately empty because physical axes/link are unverified |
| `publish_stamp` | builtin_interfaces/Time | ROS clock immediately before calling publish, after message construction |
| `session_id` | string | New UUID on cache creation/driver session; not a hardware boot ID |
| `receipt_sequence` | uint64 | Per-finger callback sequence, starting at 1; includes suppressed duplicates |
| `sdk_frame_id_valid`, `sdk_frame_id`, `sdk_frame_id_raw` | bool, int64, string | SDK signed int32 preserved in int64, plus original Python representation; no unsigned reinterpretation or truncation |
| `sdk_timestamp_valid`, `sdk_timestamp_raw`, `sdk_timestamp_repr` | bool, float64, string | Finite SDK `ts` plus original Python representation |
| `sdk_timestamp_unit`, `sdk_timestamp_epoch` | strings | Both `unknown`; do not subtract SDK time from ROS time |
| `receive_monotonic_ns` | uint64 | Host monotonic clock at callback receipt; comparable only on the same host/boot |
| `receive_to_publish_sec` | float64 | Measured callback receipt to pre-publish assembly completion using host monotonic time |
| `overwritten_frames`, `duplicate_callbacks` | uint64 | Cumulative per-finger counts as of this retained callback |
| `identity_warnings` | string[] | Observed counter/time discontinuities, not physical restart classifications |
| `f6`, `contact_points`, `dist_force` | TactileBlock | All taken from this callback, never from independent ROS topic caches |
| `validation_errors` | string[] | Block/metadata name and error; may also include legacy image errors |

When a frame ID or timestamp is missing/invalid, its validity flag is false,
the numeric wire field is zero solely as an unused placeholder, and the raw
representation plus error is retained. Consumers must check the validity flag.
Valid images/other blocks are still published. A malformed outer callback or
unsupported channel is rejected and reported through the driver's warning path.

`receive_to_publish_sec` excludes DDS publication/serialization and network
delivery after the publish call, and is **not sensor acquisition latency**.
The recorder also measures subscriber ROS time minus `header.stamp`: this is
host-callback-to-subscriber latency only when ROS clocks share a synchronized
domain. Negative values can expose clock adjustment/mismatch. Neither measure
establishes SDK measurement time. The driver imposes no MPC freshness threshold.

## Block fields and validation

| Field | Type | Meaning |
| --- | --- | --- |
| `status` | uint8 | MISSING=0, VALID=1, EMPTY=2, INVALID=3 |
| `shape` | bounded uint32[] (8 dims) | Validated original logical shape, preserving singleton dimensions |
| `shape_declared` | bool | Whether the SDK provided a shape entry; otherwise inferred from the Python array |
| `declared_shape_json` | string | Original declaration including malformed dimensions; declarations over 1024 characters are truncated with explicit error |
| `dtype` | string | NumPy dtype of the Python callback payload, not necessarily the native SDK buffer dtype |
| `raw_values` | bounded float64[] (262144 values) | Full C-order values, widened to float64; no scaling, axes transform or physical conversion |
| `error` | string | Validation reason for this block |

- MISSING: no block or `None`; values absent. Missing does not imply no contact.
- VALID: finite numeric data with consistent shape.
- EMPTY: a valid zero-sized CONTACT_POINT array (e.g. `[0, 3]`); it is different
  from a missing block. No physical interpretation of an empty result is imposed.
- INVALID: malformed/nonfinite/oversized/unsupported data; values absent,
  declared shape and error retained. Never replace invalid data with zeros.

F6 requires exactly six finite values; declared shape is retained even when
it is `[1, 1, 6]`. CONTACT_POINT permits non-scalar shapes of up to eight
dimensions, including singleton axes, multiple rows and extra columns; none
are squeezed or discarded. Shape dimensions must be integral and nonnegative.
Payloads are bounded to 262144 values; integers outside the exactly representable
float64 range and floating dtypes wider than float64 are rejected.

DIST_FORCE contract v1 accepts floating `(60, 60, 3)` data within float32 range.
This is the documented/observed grid, **not a general assumption that all image
grids are equivalent**. Other grids report INVALID on the new frame interface
until explicitly documented/supported. The legacy image path remains independent
and can still publish a grid it previously supported. DEFORM3 240x240 and
DEFORM3_SE 60x60 do not establish other valid DIST_FORCE variants.

Float32 source values widen exactly to float64. Python float lists appear as
`dtype=float64`; the binding may have converted a native float buffer before
calling Python, so this field does not claim to recover the native dtype.
Using one numeric wire type avoids narrowing contact-point values or changing
legacy conversions. A standard DIST_FORCE block contributes 86400 payload bytes
per frame; the complete message is benchmarked in the validation report.

Records, metadata maps and arrays are immutable and own their contents.
The arrays use immutable byte backing; retaining or modifying SDK buffers
after callback return cannot change a pending record.

## Delivery and identity policy

Only the newest pending callback per finger is kept. The timer drains pending
frames once; it never republishes a cached stale frame. Faster SDK callbacks
can replace pending samples (`overwritten_frames`), and best-effort DDS can
also drop samples. There is no guarantee of delivery for every SDK callback.

Consecutive callbacks with the same valid `(sdk_frame_id, sdk_timestamp_raw)`
are suppressed and counted. A later recurrence after another identity is
accepted and diagnosed relative to the preceding valid identity. Invalid
identities are never deduplicated. Counters in a published frame are snapshots;
duplicates after that callback become visible on the next retained callback.

All other arrivals, including old/out-of-order frames, are accepted as whole
records in arrival order. Numeric ID regression warns of possible wrap,
restart or reordering without guessing which occurred. Forward gaps, repeated
IDs with changed timestamps and timestamp regressions/repetition are also
flagged. Source ID is not rewritten into a fabricated continuous counter.
Session UUID and local receipt sequence provide separate receipt identity.

Warnings describe the retained callback's relation to its predecessor; warnings
on overwritten callbacks may themselves be skipped. The recorder's observed
sequence gaps include duplicates, cache replacement and DDS delivery gaps;
they are not an exact hardware packet-loss measurement. Consumers decide
their own age limits and response to source discontinuities.

## Build and read-only recording

From a fresh terminal, build the package with ROS Jazzy's Python:

```bash
cd /home/hao-fang/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_sharpa_wave \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 interface show dual_sharpa_wave/msg/TactileFrame
```

The existing ament_cmake package already generates action/service interfaces;
the two messages use that same build/install convention. If generated message
support is absent, tactile-enabled startup fails with a build instruction
**before connecting hardware**, rather than silently using only legacy topics.

The operator starts/restarts the documented sensing driver when ready, with
the matching ROS domain and SDK environment from the README:

```bash
ros2 launch dual_sharpa_wave dual_sharpa.launch.py \
  backend:=sharpa_sdk read_only:=true use_rviz:=false
```

The recorder does not start/stop a driver, import the SDK, calibrate, change
parameters, invoke services/actions or publish motion commands. It only
subscribes to frame topics (rclpy still has standard ROS discovery and
parameter-event infrastructure).

```bash
ros2 topic type /sharpa/right_hand/tactile/index/frame
ros2 topic echo --once --qos-reliability best_effort \
  /sharpa/right_hand/tactile/index/frame
ros2 topic hz /sharpa/right_hand/tactile/index/frame
ros2 run dual_sharpa_wave capture_tactile_frames.py \
  --side both --duration 10 --output /tmp/sharpa_frames_summary.json
```

Expected type: `dual_sharpa_wave/msg/TactileFrame`. Replace `index` with any
finger in the mapping table and `right` with `left` as needed. Echo prints
the full DIST_FORCE values; the summary recorder avoids huge image payloads.
It reports per-finger rates, availability, last shape/dtype, validation errors,
timing, identity discontinuities and raw numeric ranges (including each F6
component). Ranges have no physical units assigned. It returns 2 if any selected finger has
no accepted messages (or wrong metadata); missing optional blocks are reported,
not synthesized. Existing output files are not overwritten. A summary is not
a replayable raw recording. To retain full frames, use subscriber-only rosbag:

```bash
ros2 bag record -o /tmp/sharpa_frames_bag \
  /sharpa/right_hand/tactile/pinky/frame \
  /sharpa/right_hand/tactile/ring/frame \
  /sharpa/right_hand/tactile/middle/frame \
  /sharpa/right_hand/tactile/index/frame \
  /sharpa/right_hand/tactile/thumb/frame
```

Store large bags outside Git. Manual-touch validation is performed by the
operator; this package never moves a finger to create contact or tares automatically.

## Phase 1B handoff

Ada needs only the installed ROS message package and the five right-hand
frame topics; it must not import SDK objects or reconstruct association from
legacy topics. Subscribe best-effort depth 1, check `schema_version`, side,
finger and channel, then validate the required blocks and identity flags.
Reshape each block using its own `shape` and C-order `raw_values`.

Use `(session_id, channel, receipt_sequence)` as receipt identity. Preserve
SDK identity/time for diagnostics; apply application-specific freshness and
discontinuity policy using clocks whose relationship is known. A downstream
application requiring all three blocks must explicitly reject incomplete
frames, rather than carrying values from a preceding callback forward.

Still unknown (not silently resolved by publishing floats): F6 physical units,
component order, coordinate frame, sign/direction and reference origin;
CONTACT_POINT third/additional component meanings and pixel-grid relationship;
DIST_FORCE channels/units; `deform_map_uv()` position unit and exact URDF
link-frame equivalence. SDK samples draw the first two CONTACT_POINT components
as image x/y, which does not establish a physical coordinate transform.
Physical calibration, contact reconstruction, FK and MPC are outside Phase 1A.

In the 2026-11-11 live validation, all ten fingers delivered valid F6 and
DIST_FORCE, while CONTACT_POINT was MISSING with a declared shape `[]` in all
4209 recorded frames. This is an observed limitation of that session, not a
claim that the device can never produce contact points. Manual-touch recording
was skipped at the operator's request. Phase 1B must handle this missing block
explicitly; it cannot treat it as a measured absence of contact.
