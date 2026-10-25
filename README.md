# Motion Tennis Physics Engine

The fusion and physics component of **Motion Tennis**, a doubles tennis demonstrator for four people in two fixed teams. It runs on the available **Raspberry Pi 3 Model B**, combines wireless controller measurements with laptop webcam observations, and calculates automatic avatar movement, ball motion, serve/rally transitions and score changes using state requested from the PC game service.

Start with [repository orientation](docs/orientation.md), [staged workflow](docs/workflow.md) and the canonical [current project status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md). This README describes the intended component; it does not authorize implementing every planned feature at once. [AGENTS.md](AGENTS.md) defines local contributor boundaries.

## Status and execution environment

| Status | Meaning |
| --- | --- |
| **Implemented** | Documentation and an MIT license only; no simulation, fusion, transport, build configuration or test suite exists |
| **Agreed design** | Pi-owned doubles physics/rules, four wireless battery controllers, automatic avatar movement with human racket gestures, webcam-first calibrated 2.5D gameplay, and an explicit ball-state request/response with the Windows game service |
| **Proposed reference** | Native C++20/CMake on 64-bit Raspberry Pi OS Lite; bounded buffers, nonblocking I/O, 5 ms scheduling and ordered atomic batches |
| **Open** | Verified OS/kernel/toolchain versions, calibrated error thresholds, measured Pi capacity and eventual performance-driven upgrades |

The Pi 3 Model B is confirmed hardware; its ability to meet the proposed targets is not confirmed. Keep camera inference, rendering and critical-path disk logging off the Pi. Baseline A uses an external AP and wired Pi/laptop. Planned variant B runs the controller hotspot on the Pi while the laptop remains wired; measure the additional wireless/AP workload alongside physics. The [A/B comparison](https://github.com/motion-tennis/documentation/blob/main/docs/networking.md#external-ap-versus-pi-hotspot-comparison) keeps all application traffic, scheduling policies and markers unchanged and requires verified interface/control/debug reachability. See [master plan](https://github.com/motion-tennis/documentation/blob/main/docs/master-plan.md) and [hardware](https://github.com/motion-tennis/documentation/blob/main/docs/hardware.md).

## Responsibilities and fusion boundary

Physics owns controller/person association, clock-mapped input selection, calibrated body-relative gesture mapping, sensor fusion, swing classification, collision and every authoritative match calculation. This includes ball motion, four avatar roots/headings, serve/rally state and score changes. The Windows game service stores the sole committed state; the renderer must not create competing movement or rule results. The current schema is still ball-only and needs the versioned match-contract extension described below before full doubles implementation.

The controller's [independent threshold marker](https://github.com/motion-tennis/documentation/blob/main/docs/imu-trigger-markers.md) is joined to existing sample/transaction traces asynchronously. Physics preserves pose, query, batch, decision, commit and feedback identities so the viewer can follow the full path. The marker does not change gameplay payloads, admission, order or priority and is not proof of a swing/hit. Physics must not wait for marker delivery or a debug consumer.

Retain 256 IMU samples/controller and eight pose frames/person. Align each delayed camera observation to historical IMU samples near its capture time, update body-relative arm motion and replay orientation to the current evaluation instant. Keep current and historical sample IDs, pose frame, association/calibration and uncertainty in the trace. Never pretend a recent network arrival is a recent exposure.

The local gesture estimate is explicitly `plane_2_5d`, obtained from calibrated wrist/body-reference observations at a physical station. Combine this local racket displacement and IMU orientation with the Pi-owned avatar root and heading; absolute physical wrist position does not become virtual court position. Physical stations provide calibration/separation, not fixed virtual lanes or teams. Use bounded local-motion prediction; do not double-integrate acceleration into long-term position or claim that six-axis IMU data observes absolute yaw or webcam data measures free-space wrist depth. See [fusion and collision design](https://github.com/motion-tennis/documentation/blob/main/docs/physics.md) and [coordinates/time](https://github.com/motion-tennis/documentation/blob/main/docs/coordinates-and-time.md).

## Doubles movement, assignment and serving

The [gameplay contract](https://github.com/motion-tennis/documentation/blob/main/docs/gameplay.md) gives each human one persistent avatar. Physics moves those avatars automatically with bounded movement and deterministic teammate arbitration; human gestures still determine swing/contact attempts. For the assigned server, recognize a body-relative controller lift for the virtual toss and an overhead swing from ordinary pose/IMU history. The marker IRQ cannot authorize a toss or hit. All four continuous 200 Hz streams remain unchanged, including between points. Movement tuning, gesture thresholds and shot-direction/strength transfer remain unverified implementation choices.

The [assignment contract](https://github.com/motion-tennis/documentation/blob/main/docs/player-assignment.md) proposes slots `1=A1`, `2=A2`, `3=B1`, `4=B2`. Enforce one controller/person/avatar per slot with explicit operator confirmation, versioned installation acknowledgement and the resume barrier. Require all four slots ready for normal doubles; one-to-three-player tests are integration fixtures. Court-end and front/back changes preserve teams and identities. Roster/re-pair changes require the documented pause and invalidation rules; technical failures remain explicit failures, not tennis points.

## Pi reference clock mapping

The [agreed clock design](https://github.com/motion-tennis/documentation/blob/main/docs/coordinates-and-time.md#pi-reference-clock-mapping), recorded in [ADR 0009](https://github.com/motion-tennis/documentation/blob/main/docs/adr/0009-pi-reference-clock-mapping.md), makes one declared Pi monotonic domain the reference. Physics coordinates four-timestamp `ClockExchange` over existing TCP control connections, fits source offset and drift as `T_pi = a * t_source + b`, and owns validity/readiness decisions. Preserve raw exchange timestamps and clock/session identities; native clocks are never stepped. Publish immutable mapping versions with uncertainty, validity and source/Pi reference epochs asynchronously, so runtime fusion and debug consumers use the same evidence without a collector dependency.

A Pi restart/reference discontinuity starts a new reference epoch and invalidates all live mappings; a source restart invalidates that source's mapping. Unknown, expired or insufficiently accurate mappings fail required freshness/alignment gates and cross-device comparisons. Continue accounting for the unchanged 200 Hz controller streams and explicit failures; same-clock RTT remains measurable. Estimation, map publication and readiness integration are not implemented. No gameplay schema, port or numeric profile value changes with this decision.

## Required query and proposed batch execution

Four 200 Hz controllers offer 800 distinct measurement transactions/s. Every 5 ms release selects an ordered batch, normally four inputs, with a maximum of eight and at most two from any controller. Keep four queued pending inputs/controller and only one state batch in flight.

1. Validate identity, association, source timing, calibration and temporal eligibility. Reject unsuitable input explicitly.
2. Send **`ball_state_request` to the Windows game service** with one logical query entry for each selected transaction.
3. Receive **`ball_state_response`**, echoing the query entries and one atomic common versioned snapshot. Batching preserves both required directions and each original measurement identity.
4. Integrate once and evaluate the ordered inputs sequentially against the snapshot and preceding candidate changes. Recheck freshness/expiry after network waiting and before submission.
5. Send one `state_update` against the exact base version; the game accepts or rejects the complete batch atomically. The existing schema covers final ball state and ordered decisions; the required versioned extension must also cover avatar, serve/rally, roster/rules references and score effects.
6. After accepted `state_commit_result`, send individual correlated `feedback_result` messages. Each result retains batch version and event index. Failures are explicit outcomes, not ordinary misses.

Periodic-only batches advance automatic movement and ball simulation through the same query/commit path without fabricating a human swing. A busy writer counts skipped releases instead of spawning another writer. Limit catch-up to four logical steps and pause when simulation lag exceeds 20 ms. Uncertain commits pause the writer for status recovery; do not invent a new ID and repeat a possibly accepted hit or score change. Full arbitration, expiry and deduplication rules are in [physics](https://github.com/motion-tennis/documentation/blob/main/docs/physics.md) and [protocol](https://github.com/motion-tennis/documentation/blob/main/docs/protocol.md).

## Inputs, outputs, and profile

The proposed [`0.2-proposed` schema](https://github.com/motion-tennis/documentation/blob/main/spec/motion.proto) uses bounded UDP/Protobuf live messages, at most 1,200 bytes including envelope; TCP handles control/recovery. No listener is implemented yet.

The full doubles snapshot/update/publication contract is not encoded by that ball-only proposal. Before implementing match state, review an explicit schema/profile extension for roster, avatar movement, serve/rally and scoring, including atomicity and bounded-size fixtures. This documentation decision changes no current schema field, numeric profile value, port or stream behavior.

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

Configured GitHub access is required. Planned modules are nonblocking transport/control, bounded input histories, association/calibration, time mapping, fusion/swing state, automatic avatar movement, deterministic ball/serve/rules processing, atomic match-batch handling and trace export. Add actual manifests, source paths and commands when implemented. Follow the [integrated startup plan](https://github.com/motion-tennis/documentation/blob/main/docs/development.md).

## Planned smoke test and current checks

These planned checks span future stages. Select one authorized slice through the [workflow](docs/workflow.md); they are not a single current implementation task.

Start with synthetic controller/pose streams and a game-state simulator on the actual Pi. Trace four simultaneous inputs through their logical queries, deterministic ordering, one atomic commit and four matching results. Exercise stale state, removed-invalid entries, duplicates, lost commit replies, restart, queue overflow and periodic-only movement. After the match-contract extension, add four-slot readiness, automatic movement with manual swings, serving, score transitions and duplicate-point prevention. Integrate real sensors using the [one-to-four-controller matrix](https://github.com/motion-tennis/documentation/blob/main/docs/testing.md); only the four-player case is a normal doubles match.

No component or hardware tests have run. Current documentation check:

```sh
git diff --check
```

Use the central [development checker](https://github.com/motion-tennis/documentation/blob/main/docs/development.md) for cross-repository links. The [debug specification](https://github.com/motion-tennis/documentation/blob/main/docs/debugging.md) covers correlated traces, packet evidence, failure denominators and replay; synthetic checks do not establish runtime performance.

## Layout, limitations, and contribution

```text
README.md            Component reference profile and implementation status
AGENTS.md            Contributor scope and contract rules
docs/orientation.md  Role, reading order and bounded entry tasks
docs/workflow.md     Local contribution to shared stages and handoffs
LICENSE              Existing MIT license
```

No source establishes timing, fusion accuracy or collision behavior. Calibrated geometry, clock uncertainty, heading drift and four-player capacity remain acceptance gates. See [open decisions](https://github.com/motion-tennis/documentation/blob/main/docs/roadmap.md).

All project edits must be in English, including code, identifiers, comments, docstrings, examples and diagrams. New C/C++ comments use `//`. Keep the shared schema/profile, architecture and related README contracts consistent.

Related repositories: [documentation](https://github.com/motion-tennis/documentation), [controller](https://github.com/motion-tennis/motion-tennis-controller), [camera tracking](https://github.com/motion-tennis/motion-tennis-camera-tracking), [game engine](https://github.com/motion-tennis/motion-tennis-game-engine), and [debug-units](https://github.com/motion-tennis/debug-units).

Licensed under the [MIT License](LICENSE), copyright 2026 Viktor Hofer. Preserve this attribution.
