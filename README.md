# Motion Tennis Physics Engine

The fusion and physics component of **Motion Tennis**, a multiplayer tennis demonstrator for up to four players. It runs on the available **Raspberry Pi 3 Model B**, combines wireless controller measurements with laptop webcam observations, and calculates hit results and ball motion using state requested from the PC game service.

## Status and execution environment

| Status | Meaning |
| --- | --- |
| **Implemented** | Documentation and an MIT license only; no simulation, fusion, transport, build configuration or test suite exists |
| **Agreed design** | Pi physics, four wireless battery controllers, webcam-first calibrated 2.5D gameplay, and an explicit ball-state request/response with the Windows game service |
| **Proposed reference** | Native C++20/CMake on 64-bit Raspberry Pi OS Lite; bounded buffers, nonblocking I/O, 5 ms scheduling and ordered atomic batches |
| **Open** | Verified OS/kernel/toolchain versions, calibrated error thresholds, measured Pi capacity and eventual performance-driven upgrades |

The Pi 3 Model B is confirmed hardware; its ability to meet the proposed targets is not confirmed. Keep camera inference, rendering and critical-path disk logging off the Pi. Wire it to the dedicated LAN rather than using its onboard Wi-Fi as the controller access point. See [master plan](https://github.com/motion-tennis/documentation/blob/main/docs/master-plan.md), [hardware](https://github.com/motion-tennis/documentation/blob/main/docs/hardware.md), and [networking](https://github.com/motion-tennis/documentation/blob/main/docs/networking.md).

## Responsibilities and fusion boundary

Physics owns controller/person association, clock-mapped input selection, calibrated plane mapping, sensor fusion, swing classification, collision and all authoritative ball calculations. The Windows game service stores the sole committed ball state. The renderer must never create a competing ball history.

Retain 256 IMU samples/controller and eight pose frames/person. Align each delayed camera observation to historical IMU samples near its capture time, update plane position and replay orientation to the current evaluation instant. Keep current and historical sample IDs, pose frame, association/calibration and uncertainty in the trace. Never pretend a recent network arrival is a recent exposure.

The initial position estimate is explicitly `plane_2_5d`, obtained from calibrated image rays and a declared player interaction plane. It is not measured free-space wrist depth. Use bounded plane-motion prediction and calibrated gyro orientation; do not double-integrate acceleration into long-term racket position or claim that a six-axis IMU observes absolute yaw. See [fusion and collision design](https://github.com/motion-tennis/documentation/blob/main/docs/physics.md) and [coordinates/time](https://github.com/motion-tennis/documentation/blob/main/docs/coordinates-and-time.md).

## Required query and proposed batch execution

Four 200 Hz controllers offer 800 distinct measurement transactions/s. Every 5 ms release selects an ordered batch, normally four inputs, with a maximum of eight and at most two from any controller. Keep four queued pending inputs/controller and only one state batch in flight.

1. Validate identity, association, source timing, calibration and temporal eligibility. Reject unsuitable input explicitly.
2. Send **`ball_state_request` to the Windows game service** with one logical query entry for each selected transaction.
3. Receive **`ball_state_response`**, echoing the query entries and one atomic common versioned snapshot. Batching preserves both required directions and each original measurement identity.
4. Integrate once and evaluate the ordered inputs sequentially against the snapshot and preceding candidate changes. Recheck freshness/expiry after network waiting and before submission.
5. Send one `state_update` containing the final ball state and ordered decisions; the game accepts or rejects the complete batch atomically against its base version.
6. After accepted `state_commit_result`, send individual correlated `feedback_result` messages. Each result retains batch version and event index. Failures are explicit outcomes, not ordinary misses.

Periodic-only batches keep the ball moving without controller input through the same query/commit path. A busy writer counts skipped releases instead of spawning another writer. Limit catch-up to four logical steps and pause when simulation lag exceeds 20 ms. Uncertain commits pause the writer for status recovery; do not invent a new ID and repeat a possibly accepted hit. Full arbitration, expiry and deduplication rules are in [physics](https://github.com/motion-tennis/documentation/blob/main/docs/physics.md) and [protocol](https://github.com/motion-tennis/documentation/blob/main/docs/protocol.md).

## Inputs, outputs, and profile

The proposed [`0.2-proposed` schema](https://github.com/motion-tennis/documentation/blob/main/spec/motion.proto) uses bounded UDP/Protobuf live messages, at most 1,200 bytes including envelope; TCP handles control/recovery. No listener is implemented yet.

| Direction | Interface | Component |
| --- | --- | --- |
| Input / output | `controller_input` on UDP 41000 / `feedback_result` to negotiated endpoint | ESP32 controllers |
| Input | Per-person `skeleton_frame` and common-frame `frame_status` on UDP 41020 | Windows tracking service |
| Output / input | `ball_state_request` / `ball_state_response` at UDP 41010 | Windows game state service |
| Output / input | `state_update` / `state_commit_result` at the same state endpoint | Windows game state service |
| Control | TCP 41040 locally, plus game/tracking control connections | Setup, pairing, clock mapping and recovery |
| Diagnostics | Buffered stage records, queue/skip/error counters | `debug-units` |

Candidate gates are current IMU age 30 ms, pose age 80 ms, historical capture alignment 25 ms, position prediction 50 ms, network-clock uncertainty 2 ms/source and camera-timestamp uncertainty 8 ms. All limits include applicable propagated uncertainty. The [reference profile](https://github.com/motion-tennis/documentation/blob/main/spec/reference-profile.json) is canonical; do not duplicate incompatible defaults in code.

The batch service target is p99 at most 4 ms; controller response p99 at most 30 ms, eligible electrical feedback p99 at most 40 ms, and local transaction expiry 50 ms. These are unverified service targets. A 5 ms scheduler or PREEMPT_RT kernel cannot guarantee the complete wireless/Windows transaction. See [timing](https://github.com/motion-tennis/documentation/blob/main/docs/timing.md).

## Prerequisites and setup

Git and a Markdown viewer are sufficient today. Future implementation requires the selected C++20 compiler, CMake, generated bounded codecs and a verified Pi OS profile. No supported package-install, build or run command exists yet.

Clone using the verified remote:

```sh
git clone git@github.com:motion-tennis/motion-tennis-physics-engine.git
cd motion-tennis-physics-engine
```

Configured GitHub access is required. Planned modules are nonblocking transport/control, bounded input histories, association/calibration, time mapping, fusion/swing state, deterministic physics, batch-state handling and trace export. Add actual manifests, source paths and commands when implemented. Follow the [integrated startup plan](https://github.com/motion-tennis/documentation/blob/main/docs/development.md).

## Planned smoke test and current checks

Start with synthetic controller/pose streams and a game-state simulator on the actual Pi. Trace four simultaneous inputs through their logical queries, deterministic ordering, one atomic commit and four matching results. Exercise stale state, removed-invalid entries, duplicates, lost commit replies, restart, queue overflow and periodic-only movement. Then integrate real sensors and run the [one-to-four-controller matrix](https://github.com/motion-tennis/documentation/blob/main/docs/testing.md).

No component or hardware tests have run. Current documentation check:

```sh
git diff --check
```

Use the central [development checker](https://github.com/motion-tennis/documentation/blob/main/docs/development.md) for cross-repository links. The [debug specification](https://github.com/motion-tennis/documentation/blob/main/docs/debugging.md) covers correlated traces, packet evidence, failure denominators and replay; synthetic checks do not establish runtime performance.

## Layout, limitations, and contribution

```text
README.md  Component reference profile and implementation status
LICENSE    Existing MIT license
```

No source establishes timing, fusion accuracy or collision behavior. Calibrated geometry, clock uncertainty, heading drift and four-player capacity remain acceptance gates. See [open decisions](https://github.com/motion-tennis/documentation/blob/main/docs/roadmap.md).

All project edits must be in English, including code, identifiers, comments, docstrings, examples and diagrams. New C/C++ comments use `//`. Keep the shared schema/profile, architecture and related README contracts consistent.

Related repositories: [documentation](https://github.com/motion-tennis/documentation), [controller](https://github.com/motion-tennis/motion-tennis-controller), [camera tracking](https://github.com/motion-tennis/motion-tennis-camera-tracking), [game engine](https://github.com/motion-tennis/motion-tennis-game-engine), and [debug-units](https://github.com/motion-tennis/debug-units).

Licensed under the [MIT License](LICENSE), copyright 2026 Viktor Hofer. Preserve this attribution.
