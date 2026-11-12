# Phase 1A implementation validation

Status: **Phase 1A raw-data contract delivered and verified**, with the observed
CONTACT_POINT absence preserved. The operator explicitly requested skipping
manual-contact recording; contact response is not claimed as tested.
The precise wire/API contract and commands are
in [tactile_phase1a_contract.md](tactile_phase1a_contract.md).

## Delivered

- `msg/TactileFrame.msg` and `msg/TactileBlock.msg`, built in the existing
  ament_cmake/rosidl package alongside its action/service interfaces.
- Immutable per-callback records with original block shapes, raw F6/contact
  columns/DIST_FORCE, missing/empty/invalid states, SDK identity, ROS receipt
  and monotonic timing. Bounded latest-frame cache and duplicate/overwrite counts.
- One `/sharpa/<side>_hand/tactile/<finger>/frame` atomic topic per fingertip;
  both side mappings supported. Legacy topic contents/QoS and control pathways
  retained; tactile-disabled behavior retained.
- `capture_tactile_frames.py`, a subscriber-only summary recorder with no SDK
  connection or motion path, plus documented rosbag commands for full payloads.
- Synthetic contract, ROS serialization, all-finger DDS coherence, compatibility,
  availability, identity, immutability and diagnostic-safety checks.
- Phase 1B subscriber handoff and explicit unresolved physical semantics.

## Software verification

Initial build/install and software verification were isolated under
`/tmp/sharpa_phase1a_impl`. The operator subsequently started the new read-only
hardware drivers from that installation. After validation, the package was
also built into `/home/hao-fang/ws_fanuc/install` for normal future launches;
the running hardware session was not restarted or signaled by the agent.
ROS Jazzy system Python 3.12 was selected explicitly. A fresh build generated
and imported both messages; `ros2 interface show` and the installed recorder's
`--help` succeeded. CDR roundtrip preserves IDs, timestamps, shapes and values.

The regression command (run from `/tmp`, after sourcing the isolated install):

```bash
export ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_HOME=/tmp/sharpa_phase1a_impl/ros
export ROS_LOG_DIR=/tmp/sharpa_phase1a_impl/ros/log OPENBLAS_NUM_THREADS=1
python3 -m pytest -q /home/hao-fang/ws_fanuc/src/dual_sharpa_wave_ros2/test \
  --ignore=/home/hao-fang/ws_fanuc/src/dual_sharpa_wave_ros2/test/test_gui_control.py \
  -k 'not test_dual_mock_over_dds[True]' \
  --junitxml=/tmp/sharpa_phase1a_impl/regression.xml
```

Implementation regression: **338 passed, 7 skipped, 1 deselected**, with
the three Tk GUI tests excluded at collection. The four display-dependent
tests previously failed because no display is available; they are not claimed
to pass. The seven skips are six opt-in single-hand mock launch tests and one
gain GUI test without a Tk display.

After adding explicit diagnostics for absent type-support libraries, the final
focused run passed **124/124** tests (`test_tactile_contract.py`,
`test_tactile.py`, `test_sdk_hand.py`, `test_hand_node.py`). Its JUnit report is
`/tmp/sharpa_phase1a_impl/final_focused.xml`. This includes two additional
preflight-error checks beyond the preceding full regression run; the hardware
backend is never created when message support is unavailable.

The subsequent recorder-only raw-range addition passed all **45** contract and
recorder tests (`/tmp/sharpa_phase1a_impl/recorder_ranges.xml`), without changes
to the running driver logic.

The installed recorder also passed a two-second synthetic DDS integration
capture of all five right fingers: 281 total frames, with empty CONTACT_POINT
explicitly reported as EMPTY and zero validation errors. This is software
evidence, not hardware evidence. The preserved
[summary](tactile_phase1a_synthetic_capture.json) is labeled SYNTHETIC.

Fake SDK tests and isolated mock ROS domains exercise control compatibility
without commands to physical hands. Recorder tests forbid SDK import, service
clients and all publishers except rclpy's standard parameter-event publisher.
No calibration, motor-control transition, firmware update or hardware command
is introduced by the diagnostic path.

## Atomic-transport cost

A 200-iteration synthetic microbenchmark (after 20 warmup iterations) used F6,
one three-column CONTACT_POINT, 60x60x3 DIST_FORCE, plus legacy RAW 240x320 and
DEFORM3 240x240x3 in the input callback. It generated an **86912-byte** CDR frame.

| Stage | Median ms | p95 ms |
| --- | --- | --- |
| Cache receive/copy/validation | 0.0835 | 0.1191 |
| Drain and frame-message construction | 0.0248 | 0.0384 |
| CDR serialization | 0.0153 | 0.0238 |

This measures local synthetic processing, not live hardware throughput or DDS
delivery. It supports retaining the simpler atomic transport; no split-topic
synchronization workaround was introduced. Float64 payload transport preserves
numeric precision without physical conversion. Benchmark output is retained
in [tactile_phase1a_synthetic_benchmark.json](tactile_phase1a_synthetic_benchmark.json).

## Live atomic-frame verification

The [2026-11-11 legacy capture](tactile_read_test_2026-11-11.json) confirmed both
powered hands and ten fingers publishing F6, DIST_FORCE and RAW at approximately
27.4–29.1 Hz during a 10-second observation, with no invalid received samples.
Both drivers reported `read_only=true`; both joint-command topics had zero
publishers/subscribers. **That capture predates the new message implementation.**

After operator startup of the new drivers, both `read_only` parameters were
again confirmed true. Ten `/frame` topics were discovered with the new type;
the queried right-index publisher used BEST_EFFORT and VOLATILE QoS. Discovery
did not expose a numeric history depth, so depth 1 is established by code/tests,
not claimed as a measured endpoint property.

The [15.0085-second live capture](tactile_phase1a_live_frames_2026-11-11.json)
ended at 2026-11-11 21:34:13 PST (2026-11-12 05:34:13 UTC), receiving **4209
atomic frames** across all ten fingers. Every received frame had valid F6 and
DIST_FORCE, zero validation errors, and **missing CONTACT_POINT**. Missing is
faithfully transported; no contact points were synthesized.

| Side | Finger | Frames | Hz | Receipt→pre-publish mean ms | Receipt→subscriber mean ms | Receipt-sequence gaps |
| --- | --- | --- | --- | --- | --- | --- |
| left | pinky | 413 | 27.53 | 16.53 | 16.72 | 40 |
| left | ring | 423 | 28.39 | 16.94 | 17.15 | 40 |
| left | middle | 423 | 28.39 | 16.67 | 16.89 | 40 |
| left | index | 428 | 28.72 | 17.23 | 17.47 | 34 |
| left | thumb | 421 | 28.25 | 17.06 | 17.31 | 41 |
| right | pinky | 429 | 28.60 | 17.91 | 18.12 | 37 |
| right | ring | 412 | 27.46 | 17.02 | 17.24 | 42 |
| right | middle | 422 | 28.07 | 16.66 | 16.90 | 45 |
| right | index | 422 | 28.13 | 17.06 | 17.30 | 43 |
| right | thumb | 416 | 27.73 | 16.31 | 16.57 | 49 |

Rates use subscriber monotonic arrival intervals. Pre-publish timing uses the
driver's monotonic clock; subscriber timing uses ROS time on the same host.
Neither is SDK sensor-acquisition latency. Min/max/mean values are preserved
in JSON. Source-ID forward gaps were observed (34–52 events per finger), with
no source-ID regressions, session changes or repeated/regressed receipt
sequences during this short capture. Session counters record latest-frame
replacement; sequence gaps also include possible DDS loss and cannot be
interpreted as exact hardware packet loss. Restart/wrap/reordering handling
was tested synthetically, not induced on hardware.

An independent [three-second payload check](tactile_phase1a_live_payload_check_2026-11-11.json)
received frames from every finger, checked array sizes against shapes and
finite values, and found no invalid payloads. It observed:

- F6 declared shape `[1, 1, 6]`, six values, Python payload dtype `float64`.
- DIST_FORCE declared shape `[60, 60, 3]`, 10800 values, Python payload dtype
  `float64`. This reflects Python binding numeric values; legacy ROS images
  remain float32 `32FC3` as before.
- CONTACT_POINT status MISSING, with a declared empty shape `[]` and no values.
  This is not a valid zero-point contact result and not evidence of no contact.
- Valid signed SDK frame IDs and finite raw timestamps. SDK `ts` was numerically
  close to Unix seconds in this sample, but unit/epoch are not authoritatively
  established and remain `unknown` in the contract.
- Zero joint-command publishers and zero subscribers for both hands.

The completed [legacy compatibility probe](tactile_phase1a_live_legacy_compatibility_2026-11-11.json)
matched **550** right-hand atomic DIST_FORCE blocks with legacy images by their
identical host receipt stamps (115 pinky, 106 ring, 110 middle, 110 index, 109
thumb). Casting the atomic values to the legacy float32 representation yielded
exactly equal image values in every pair: **zero mismatches**. This does not
invent a timestamp for the headerless legacy F6 topic; F6 content compatibility
is covered by the existing synthetic tests.

Manual contact recording was **skipped at the operator's explicit request**.
No controlled fingertip-touch sequence or contact-induced signal response is
claimed. No further live sampling was started after that request. The recorder
includes raw component ranges for an optional future manual check, without
robot movement or automatic tare.

F6 units/order/axes/sign/reference origin, CONTACT_POINT extra-column meanings,
DIST_FORCE physical interpretation, SDK timestamp epoch/unit and deform-map
position units/URDF equivalence remain unverified. Missing blocks remain
missing; successful transport is not calibrated-force or MPC certification.
