# S1 deterministic batch core

[Repository home](../README.md) · [S0 review and S1 handoff](s0-contract-review.md) · [Local workflow](workflow.md) · [Central status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md)

## Task record

| Field | Value |
| --- | --- |
| Stage | S1 — isolated component slice; this task does not change the central stage |
| Scope | Deterministic fixed-step ball/batch core with fake controller, pose and state-service adapters, as defined in the [first bounded S1 task](s0-contract-review.md#first-bounded-s1-task) |
| Contract | `motion.v03`, protocol `0.3-proposed`, profile `webcam-pi3-doubles-v3`; read from `documentation@3260994598f492def82abba6046b8fda78257bc6` |
| Contract hashes | Schema `b6728e240316e957f5c5addb973f12f9a57fffbcdacc4331f3db5e06895f360b`, profile `a3533462fdebfd4c7c2f87eaa3bd579d3056170e3dcfc6a6c68ed34474c1d969`, matching the [S0 gate record](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md#s0-accepted--2026-10-26) |
| Base revision | `motion-tennis-physics-engine@7184926` (`origin/main` fetched before branching) |
| Excluded | Real UDP/TCP transport, generated C++ codecs, sensor fusion, clock estimation, the avatar planner, collision geometry, serve/rally/scoring rules and every Pi or network measurement |

The schema/profile hashes are computed over the Git blobs. A Windows checkout with `core.autocrlf=true` converts line endings and yields different file hashes; that is a checkout artifact, not a contract change.

## Implemented

- [`contract.hpp`](../include/motion_tennis/physics/contract.hpp): typed in-memory mirror of the v0.3 messages the core produces and consumes, with fixed-capacity collections matching the static codec bounds (8 queries/decisions, 16 rule events, 5 completed sets, 4 players). It is not a wire codec.
- [`simulation.hpp`](../include/motion_tennis/physics/simulation.hpp): fixed 5 ms semi-implicit Euler ball step with gravity 9.81 m/s², ground restitution 0.75 and a swept ground check that reflects penetration instead of tunnelling; a bounded, teleport-free avatar root step toward its committed target; a 20 m/s outgoing-speed cap. Builds disable floating-point contraction so replays do not depend on fused multiply-add choices.
- [`input_scheduler.hpp`](../include/motion_tennis/physics/input_scheduler.hpp): four queued inputs per controller, explicit unknown/duplicate/overflow rejection, a rotating start slot per release, at most two inputs per controller and eight per batch, ordered by Pi arrival sequence with controller/session/request tie-breakers.
- [`batch_core.hpp`](../include/motion_tennis/physics/batch_core.hpp): the single-writer state machine `Idle -> QueryPending -> Compute -> CommitPending -> Idle`, with `Paused` for errors. It builds one `ball_state_request` per release (empty for periodic-only work), validates the echoed queries and snapshot identity, integrates the required ticks once and evaluates inputs in order so later inputs see earlier candidate changes. Inputs are rechecked after the response and immediately before submission; a newly invalid input produces explicit failure feedback and the whole candidate is rebuilt from the queried snapshot. Swing and cue identities are staged and installed only after `ACCEPTED`. Identical resends at most twice, then an ambiguity pause that only a commit lookup resolves. Late or stray replies are counted and ignored. Catch-up is limited to four ticks; a larger lag pauses instead of integrating the gap.
- [`adapters.hpp`](../include/motion_tennis/physics/adapters.hpp) and [`fakes.hpp`](../include/motion_tennis/physics/fakes.hpp): pose, contact-evaluation and state-service boundaries. The fake state service follows the offline oracle in `documentation/scripts/match_contract.py`: fenced session/epoch, retained query and commit outcomes, exact-version acceptance, changed-payload rejection and whole-batch rejection. The scripted contact evaluator replaces the real swing/collision model.
- [`tests/fixtures.hpp`](../tests/fixtures.hpp) mirrors the shared `build_setup`/`build_snapshot` fixture values.
- The later [typed tracking consumer](s1-tracking-consumer.md) adds complete source evidence and map validation. `BatchCore` rejects permissive legacy pose defaults and independently checks source/session/calibration, required joints, map applicability and conservative age both after the query and before commit. The calculation retains its immutable selected pose while current source invalidation is checked again. Pose capture values in decisions remain original native source times.

The core has no heap allocation in its batch path: all collections are fixed-capacity, and feedback and trace buffers count losses instead of growing. The fakes may allocate.

## Build and checks

Verified host toolchain on 2026-11-05: Windows 11, MinGW-w64 GCC 13.1.0 (`x86_64-msvcrt-posix-seh`), CMake 3.26.4, Ninja 1.11.1. Warnings are errors (`-Wall -Wextra -Wpedantic -Wconversion`). No external dependencies.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

The initial 2026-11-05 Release and Debug evidence passed these 23 checks:

| Acceptance item from the S1 handoff | Fixture |
| --- | --- |
| Known-answer movement from a fixed state | `free_fall_known_answer`, `ground_bounce_restitution`, `avatar_bounded_step` |
| Replay determinism | `replay_is_bit_identical` (40-tick scenario run twice; snapshots, feedback and side-state compared field by field) |
| Stable ordering of simultaneous inputs; eight inputs, one atomic commit | `eight_inputs_one_atomic_commit`, `later_event_sees_earlier_candidate`, `outgoing_speed_cap` |
| Periodic-only progress | `periodic_only_batch`, `catch_up_up_to_four_ticks` |
| Rollback after one rejected member | `input_invalidated_before_submission_rebuilds` (rebuilt candidate equals a batch that never contained the input), `tracking_unavailable_is_not_a_miss` |
| Stale version and epoch/session rejection with full-state comparison | `stale_base_version_rolls_back_everything`, `wrong_writer_epoch_is_fenced`, `wrong_game_session_is_rejected` |
| Identical retry versus conflicting payload | `identical_retry_returns_retained_outcome`, `changed_payload_under_same_identity_is_rejected` |
| Ambiguous and unknown commit outcomes | `ambiguous_commit_pauses_until_resolved`, `unknown_outcome_after_retention_pauses` |
| Bounded queues and one batch in flight | `per_controller_limit_and_queue_overflow`, `one_batch_in_flight`, `already_current_tick_does_not_release`, `lag_beyond_catch_up_pauses` |
| Preserved query/transaction/commit trace identities | `trace_preserves_identities` |

Recorded output of `build/physics_s1_tests.exe`, last line: `23/23 S1 batch-core checks passed`.

The 2026-11-11 MSVC Release rerun retains all 23 passes; the new `tracking_bridge_tests` target passes 34 additional checks. See the [consumer validation record](s1-tracking-consumer.md#validation-and-handoff) for current toolchain, actual-producer evidence and remaining limits.

### Controller transaction replay — 2026-11-19

The S1 repair on `task/s1-transaction-replay` uses physics base `d6d7143` and shared contract documentation `afcb301`. The earlier scheduler could admit one controller request again during its query and after its commit, allowing three committed contacts. Admission now matches controller ID, controller session and request sequence against queued work, the selected batch and retained terminal outcomes. A duplicate awaiting its original decision receives the duplicate admission status while the original request continues. After completion, repeats receive the same feedback, including its batch, event, cue and bounded reason. This also covers rejected inputs and queue overflow. The receiver can identify repeated feedback from the original transaction and cue identities.

Each of the four bound controller sessions has a fixed history of its 16 most recently completed outcomes. This local capacity covers 16 completions per controller, equivalent to 80 ms at the nominal 200 Hz. Active keys are tracked separately for the four queued and at most two selected requests per controller. Eviction advances a per-session watermark to the highest evicted sequence. The scheduler checks active and retained identities first. Other sequences at or below the watermark return `OutsideReplayWindow` and increment a dedicated counter, including previously unseen packets that arrive that late. Newer unseen packets may arrive out of sequence and retain their Pi arrival order. The history lives in the current core and roster. Receiver recovery across process restarts still requires session coordination.

The fresh Windows x64 Release build using the [host build commands](s1-tracking-consumer.md#validation-and-handoff) passed 26 batch-core and 34 tracking-bridge checks. The contact regression checks that queue, query, submission and commit-ambiguity replays produce one committed contact. A second case replays the original failure after the adapter's reason buffer changes. The third exercises cache pressure, eviction, reordered arrivals and controller session boundaries. The compatibility driver measured `sizeof(BatchCore)` at 90,040 bytes, up from the earlier 68,968-byte host layout. It measured `TrackingPoseBridge` at 18,144 bytes and `PoseObservation` at 704 bytes. Pi memory headroom remains unmeasured.

### Shared physics and game fixture

The [fixture producer](../tests/s1_game_fixture.cpp) runs the S1 core with three synthetic inputs and exports its actual query, initial response snapshot, rebuilt update and failed-input evidence. The first and middle inputs produce hits in the initial candidate. The middle input ages from 25 ms at query response to 31 ms at submission. The core rebuilds against the original snapshot, preserving the first hit and the last miss as decision indices 0 and 1. The candidate contains one contact. The rejection record joins the uncommitted terminal feedback to its `InputFailed` trace by transaction, batch, writer epoch and query identity, retaining the observation time of 521,000 Pi microseconds.

The `s1_game_fixture` executable is a CTest target. It writes JSON to standard output and accepts an optional expected configuration SHA-256 in lowercase hex. The [shared check](https://github.com/motion-tennis/documentation/blob/main/scripts/check_s1_shared_batch.py) also validates the emitted typed configuration against the deterministic v0.3 encoding and passes the update and rejected transaction to the C# store. Run that entry point to reproduce both components together. The [shared handoff](https://github.com/motion-tennis/documentation/blob/main/docs/s1-physics-game-fixture.md) records the host prerequisites, accounting checks and evidence boundaries. The rejected-input list is supplied separately to the store's existing API.

## Limits and evidence boundary

These are host fixtures against fakes. They establish deterministic core semantics on this workstation only; they do not establish Pi 3 compute time, aarch64 float equivalence, network interoperability with the C# game store, clock accuracy or any latency target.

- **Typed contract.** The shared host fixture pins the canonical configuration digest. The C++ core compares typed structures for retry identity. A future transport adapter must verify the original canonical message bytes.
- **Placeholder physics.** No net, lines, racket disk, contact latch or spin. Hits come from the scripted evaluator, not from fused pose/IMU geometry. Avatar movement only follows the committed target; no target selection or teammate arbitration happens.
- **No rules.** The core never changes serve, rally phase or score and emits no rule events. The fake service still rejects score changes without an award event.
- **Declared time evidence.** Controller input still arrives through a fake already-Pi-time adapter. Tracking poses now require native source time plus a complete explicit immutable map, checked again by the core; no clock exchange or clock fitter is implemented or measured.
- **Startup handshake.** As in the game-store slice, the fixtures start from a complete version-one snapshot; the live writer registration, setup acknowledgement and initial-state handshake still need to be pinned with the game service.

## Handoff and next bounded options

The paired consumer is the C# store in `motion-tennis-game-engine` (`docs/s1-state-store.md`). The [shared fixture](#shared-physics-and-game-fixture) joins the two components for a controlled mixed batch. Remaining work includes:

1. Generate the v0.3 C++ codec from the accepted schema and replace the typed mirror at the adapter boundary, including canonical retry bytes and the real configuration digest.
2. Join controller, physics and game exports across the query, rejection, commit, publication and feedback boundaries with native correlation and timestamp evidence.
3. Repeat the build and replay on the Pi 3 toolchain (aarch64 GCC) and record float equivalence and compute time per batch.

The [central S1 ledger](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md#remaining-s1-work-at-this-review) tracks the remaining shared evidence and stage acceptance.
