# Phase 1A baseline verification — 2026-11-11

Historical baseline before the frame-interface implementation. See the
[current validation report](tactile_phase1a_validation.md) and
[implemented contract](tactile_phase1a_contract.md) for subsequent work.

Status: **existing dual-hand read-only reception verified; Phase 1A incomplete**.
This report records verification of the existing code, not a new tactile contract.
No driver source, control configuration, or installed workspace was modified.

## Checkout and environment

- Repository: `dual_sharpa_wave_ros2`, branch `main`.
- HEAD: `b460b6a10ec35fcfa83e136dbdf69f155cb1e2d2`.
- Working tree was clean before verification.
- ROS Jazzy; system Python 3.12.3.
- SDK `/opt/sharpa-wave-sdk`: version 5.0.10, build 5.0.10.6.
- Live ROS discovery environment: domain 87, discovery range SUBNET.
- Temporary build/install: `/tmp/sharpa_phase1a_build`.

## Hardware observations and boundary

SDK heartbeat discovery (`get_instance()` and `get_all_devices()`, no
`connect()` or device `start()`/`stop()` calls) found both hands:

| Side | Serial | IP | Firmware |
| --- | --- | --- | --- |
| left | CD52943BCD53 | 192.168.10.10 | 3.0.10 |
| right | C956943BC957 | 192.168.10.20 | 3.0.10 |

Initial checks found no hand driver. After the operator started the drivers,
domain 87 exposed `/sharpa/left_hand/hand_node` and
`/sharpa/right_hand/hand_node`, both with `read_only=true` (queried using
`ros2 param get`). Live subscription then confirmed receipt of the legacy
tactile streams. Other ROS domains were not surveyed.

No physical joint command, calibration/tare, firmware update, control-setting
write, or motor enable/disable was issued. Software tests use fake SDKs or
mock backends on isolated localhost ROS domains; their simulated commands
do not go to the physical hands.

## Live legacy-topic reception

The subscriber ran for 10.0096 seconds, ending at 2026-11-11 20:58:24 PST
(2026-11-12 04:58:24 UTC). It subscribed to F6, DIST_FORCE and RAW for all ten
fingertips, using best-effort QoS with depth 1. It made no SDK connection and
created no motion publisher. Both `/sharpa/<side>_hand/joint_command` topics
had zero publishers and zero subscribers at the graph check after capture.

The preserved [JSON summary](tactile_read_test_2026-11-11.json) contains counts,
rates, last-sample shapes/dtypes and small raw-value summaries, without image
payloads. All 30 subscribed streams delivered samples; none of the received
samples failed image decoding/finite-value or six-finite-F6 validation.
These checks do not measure blocks discarded upstream by the driver.

| Side | Finger | F6 count / Hz | DIST_FORCE count / Hz | RAW count / Hz |
| --- | --- | --- | --- | --- |
| left | pinky | 275 / 27.49 | 274 / 27.39 | 275 / 27.49 |
| left | ring | 284 / 28.40 | 284 / 28.39 | 284 / 28.39 |
| left | middle | 281 / 28.19 | 281 / 28.19 | 282 / 28.20 |
| left | index | 281 / 28.09 | 281 / 28.09 | 281 / 28.09 |
| left | thumb | 284 / 28.40 | 284 / 28.40 | 284 / 28.40 |
| right | pinky | 290 / 29.09 | 289 / 28.99 | 289 / 28.99 |
| right | ring | 288 / 28.80 | 288 / 28.80 | 288 / 28.80 |
| right | middle | 290 / 29.09 | 290 / 29.09 | 290 / 29.09 |
| right | index | 284 / 28.40 | 284 / 28.40 | 284 / 28.40 |
| right | thumb | 282 / 28.19 | 281 / 28.09 | 282 / 28.19 |

Rates are `(received_count - 1) / (last_receive_monotonic - first_receive_monotonic)`;
they describe this subscriber's short observation, not guaranteed SDK throughput.
Counts are independent ROS message counts, not coherent three-block frame counts.
Small count differences cannot distinguish window boundaries from transport loss.

Last observed formats for every finger:

- F6: six float64 ROS values, with no physical-unit interpretation.
- DIST_FORCE: `(60, 60, 3)` float32, image encoding `32FC3`.
- RAW: `(240, 320)` uint8, image encoding `8UC1`.

There were no repeated consecutive image header stamps recorded by the probe.
SDK frame IDs and timestamps are not carried by these legacy topics, so SDK
continuity, restart/wrap, callback coherence and exact delivery gaps remain
unverified. Latency was not measured; it cannot be inferred from these rates.
CONTACT_POINT availability at the SDK callback remains unobserved because
the existing driver does not expose it. No controlled manual-contact sequence
was recorded, and physical calibration was not assessed.

## Build and tests

| Check | Result |
| --- | --- |
| Fresh existing package and action/service interface build | PASS |
| `test_tactile.py`, `test_hand_node.py`, `test_sdk_hand.py` | 79 passed |
| Complete existing suite | 295 passed, 4 failed, 7 skipped |
| New Phase 1A message generation/import | Not applicable: interface absent |
| Live legacy tactile reception | PASS: 30 streams across ten fingertips |
| Live coherent Phase 1A frame capture | Not run: interface absent |

Three complete-suite failures are Tk GUI tests reporting
`no display name and no $DISPLAY environment variable`. The fourth is
`test_dual_mock_over_dds[True]`: RViz cannot connect to a display and exits.
These failures remain unresolved; the full suite is not reported as passing.
The seven skipped cases are six opt-in single-hand mock launch checks and
one gain GUI check skipped because a Tk display is unavailable.

Two initial environment issues were resolved without source edits:

- Running pytest from the source package directory hid generated ROS action
  modules. Running from `/tmp` against the fresh installation passes all 79
  focused checks.
- CMake initially selected a local Python 3.10 without `em`. Explicitly choosing
  `/usr/bin/python3` for both Python CMake variables makes the build succeed.

Actual result files (temporary artifacts):

- `/tmp/sharpa_phase1a_verified_tests.xml`
- `/tmp/sharpa_phase1a_build/build/dual_sharpa_wave/test_results/dual_sharpa_wave/sharpa.xunit.xml`
- `/tmp/sharpa_phase1a_contract_probe.json`

## Contract audit

Synthetic cache probes and source inspection establish:

- Mapping matches SDK `include/touch.h`: right channels 0–4, left channels 5–9;
  each is ordered pinky, ring, middle, index, thumb.
- Six F6 values, including signs, survive a callback copy. Invalid F6 does not
  discard otherwise valid images (covered by existing tests).
- Float32 DIST_FORCE `(60, 60, 3)` survives; finite-value validation exists.
- SDK frame ID and timestamp are retained in the internal cache, but are not
  transported in the existing ROS F6/image interfaces.
- CONTACT_POINT is ignored by the cache, even when provided in a synthetic
  callback. This says nothing about its availability on the physical hands.
- Original declared shapes are not retained for every block; F6 is flattened.
- Missing and empty DIST_FORCE produce identical absent-block/no-error states.
- Cached arrays remain writable and the Frame dataclass is mutable.
- No versioned atomic tactile frame ROS message/topic exists. Legacy F6 uses
  `std_msgs/msg/Float64MultiArray` without a header. Image headers carry the
  host ROS receipt stamp. Legacy topics/QoS were not changed.
- Latest-frame buffering and immediate identity duplicate suppression exist;
  dropped/replaced frame accounting and explicit restart/out-of-order provenance
  are not exposed in a frame interface.

SDK `include/def.h` documents frame IDs as signed 32-bit integers which can
overflow and reset on hardware restart. Its `double ts` is described only as
"time stamp"; the inspected declaration does not establish unit or epoch.
The Python callback sample uses dictionary `content`/`shape` payloads. The
compact C++ API also exposes CONTACT_POINT data and shape. Actual callback
contents for these devices still require live observation.

F6 physical units, component order, axes, signs and reference origin remain
unverified. CONTACT_POINT columns beyond the sample's first two visualization
coordinates remain unverified, as does its relationship to the DIST_FORCE grid.
DIST_FORCE channel meanings/units and `deform_map_uv()` position units/URDF
frame equivalence remain unverified. No physical conversion was introduced.

## Remaining Phase 1A acceptance work

The following work is required by `sharpa_tactile_phase1a_agent.md` and has
not been delivered by the baseline verification:

1. Immutable callback-frame capture of CONTACT_POINT, F6 and DIST_FORCE with
   original shapes, raw SDK identity/time, receipt timing and per-block status.
   Distinguish missing, valid empty and invalid blocks; preserve extra columns.
2. A versioned coherent ROS frame interface and actual per-finger frame topics,
   preserving legacy topics, encodings, QoS and control behavior.
3. Explicit frame/timing behavior for duplicate, stale, out-of-order and
   restarted/wrapped identities, with bounded buffering and documented skips.
4. New synthetic contract/coherence/compatibility tests and fresh generation
   and import checks for the new interface. Existing 79 passing checks do not
   test an interface that has not been implemented.
5. A reproducible read-only capture tool and validation of the new live frame
   transport, including availability, observed latency/rates and continuity.
   Operator-performed gentle fingertip contact remains untested.
6. `docs/tactile_phase1a_contract.md`, exact new-topic commands, the completed
   validation report and an SDK-independent Phase 1B subscriber handoff.

F6 physical calibration, contact reconstruction, SDK pixel-to-3D conversion,
Pinocchio FK, Ada ContactObservation and MPC control are **not Phase 1A
completion requirements**. Phase 1A must preserve raw values and document
unknown physical semantics; publishing finite numbers does not verify units.

## Reproduction

Build without replacing the live workspace installation:

```bash
source /opt/ros/jazzy/setup.bash
cd /tmp
colcon --log-base /tmp/sharpa_phase1a_build/log build \
  --base-paths /home/hao-fang/ws_fanuc/src/dual_sharpa_wave_ros2 \
  --build-base /tmp/sharpa_phase1a_build/build \
  --install-base /tmp/sharpa_phase1a_build/install \
  --packages-select dual_sharpa_wave \
  --cmake-args -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source /tmp/sharpa_phase1a_build/install/setup.bash
export ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_HOME=/tmp/sharpa_phase1a_tests_ros
export ROS_LOG_DIR=/tmp/sharpa_phase1a_tests_ros/log OPENBLAS_NUM_THREADS=1
/usr/bin/python3 -m pytest -q \
  /home/hao-fang/ws_fanuc/src/dual_sharpa_wave_ros2/test/test_tactile.py \
  /home/hao-fang/ws_fanuc/src/dual_sharpa_wave_ros2/test/test_hand_node.py \
  /home/hao-fang/ws_fanuc/src/dual_sharpa_wave_ros2/test/test_sdk_hand.py
```

Operator startup for live sensing, in a separate terminal (do not reuse the
isolated test domain or temporary install):

```bash
cd /home/hao-fang/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export PYTHONPATH="/opt/sharpa-wave-sdk/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/sharpa-wave-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ros2 launch dual_sharpa_wave dual_sharpa.launch.py \
  backend:=sharpa_sdk read_only:=true use_rviz:=false
```

Existing read-only topic checks, using the same ROS environment:

```bash
ros2 topic type /sharpa/right_hand/tactile/index/f6
ros2 topic echo --once /sharpa/right_hand/tactile/index/f6
ros2 topic hz /sharpa/right_hand/tactile/index/dist_force
```

These legacy topics cannot establish the proposed Phase 1A frame association
or serve as the completed Phase 1B handoff contract.
