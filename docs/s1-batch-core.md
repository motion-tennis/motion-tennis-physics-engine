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
| Excluded | Real UDP/TCP transport, generated codecs, sensor fusion, clock mapping, the avatar planner, collision geometry, serve/rally/scoring rules and every Pi or network measurement |

The schema/profile hashes are computed over the Git blobs. A Windows checkout with `core.autocrlf=true` converts line endings and yields different file hashes; that is a checkout artifact, not a contract change.

## Implemented

- [`contract.hpp`](../include/motion_tennis/physics/contract.hpp): typed in-memory mirror of the v0.3 messages the core produces and consumes, with fixed-capacity collections matching the static codec bounds (8 queries/decisions, 16 rule events, 5 completed sets, 4 players). It is not a wire codec.
- [`simulation.hpp`](../include/motion_tennis/physics/simulation.hpp): fixed 5 ms semi-implicit Euler ball step with gravity 9.81 m/s², ground restitution 0.75 and a swept ground check that reflects penetration instead of tunnelling; a bounded, teleport-free avatar root step toward its committed target; a 20 m/s outgoing-speed cap. Builds disable floating-point contraction so replays do not depend on fused multiply-add choices.
- [`input_scheduler.hpp`](../include/motion_tennis/physics/input_scheduler.hpp): four queued inputs per controller, explicit unknown/duplicate/overflow rejection, a rotating start slot per release, at most two inputs per controller and eight per batch, ordered by Pi arrival sequence with controller/session/request tie-breakers.
- [`batch_core.hpp`](../include/motion_tennis/physics/batch_core.hpp): the single-writer state machine `Idle -> QueryPending -> Compute -> CommitPending -> Idle`, with `Paused` for errors. It builds one `ball_state_request` per release (empty for periodic-only work), validates the echoed queries and snapshot identity, integrates the required ticks once and evaluates inputs in order so later inputs see earlier candidate changes. Inputs are rechecked after the response and immediately before submission; a newly invalid input produces explicit failure feedback and the whole candidate is rebuilt from the queried snapshot. Swing and cue identities are staged and installed only after `ACCEPTED`. Identical resends at most twice, then an ambiguity pause that only a commit lookup resolves. Late or stray replies are counted and ignored. Catch-up is limited to four ticks; a larger lag pauses instead of integrating the gap.
- [`adapters.hpp`](../include/motion_tennis/physics/adapters.hpp) and [`fakes.hpp`](../include/motion_tennis/physics/fakes.hpp): pose, contact-evaluation and state-service boundaries. The fake state service follows the offline oracle in `documentation/scripts/match_contract.py`: fenced session/epoch, retained query and commit outcomes, exact-version acceptance, changed-payload rejection and whole-batch rejection. The scripted contact evaluator replaces the real swing/collision model.
- [`tests/fixtures.hpp`](../tests/fixtures.hpp) mirrors the shared `build_setup`/`build_snapshot` fixture values.

The core has no heap allocation in its batch path: all collections are fixed-capacity, and feedback and trace buffers count losses instead of growing. The fakes may allocate.

## Build and checks

Verified host toolchain on 2026-11-05: Windows 11, MinGW-w64 GCC 13.1.0 (`x86_64-msvcrt-posix-seh`), CMake 3.26.4, Ninja 1.11.1. Warnings are errors (`-Wall -Wextra -Wpedantic -Wconversion`). No external dependencies.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Release and Debug builds both pass all 23 checks:

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

## Limits and evidence boundary

These are host fixtures against fakes. They establish deterministic core semantics on this workstation only; they do not establish Pi 3 compute time, aarch64 float equivalence, network interoperability with the C# game store, clock accuracy or any latency target.

- **Typed, not wire, contract.** No generated Protobuf/nanopb code is used yet. The configuration digest is an opaque placeholder in the fixtures; the real SHA-256 over deterministic encoding needs the generated codec. Retry identity is compared on typed structures here, while production must compare the original canonical bytes.
- **Placeholder physics.** No net, lines, racket disk, contact latch or spin. Hits come from the scripted evaluator, not from fused pose/IMU geometry. Avatar movement only follows the committed target; no target selection or teammate arbitration happens.
- **No rules.** The core never changes serve, rally phase or score and emits no rule events. The fake service still rejects score changes without an award event.
- **Fake time.** Inputs and poses arrive already in Pi time with a mapping version; no clock exchange or mapping validity is evaluated.
- **Startup handshake.** As in the game-store slice, the fixtures start from a complete version-one snapshot; the live writer registration, setup acknowledgement and initial-state handshake still need to be pinned with the game service.

## Handoff and next bounded options

The paired consumer is the C# store in `motion-tennis-game-engine` (`docs/s1-state-store.md`). Both slices use the same shared fixture shape, but they have not exchanged real messages. Next options, each a separate task:

1. Generate the v0.3 C++ codec from the accepted schema and replace the typed mirror at the adapter boundary, including canonical retry bytes and the real configuration digest.
2. Cross-check against the shared Python oracle by exporting the core's candidate updates as fixtures for `tests/match`.
3. Repeat the build and replay on the Pi 3 toolchain (aarch64 GCC) and record float equivalence and compute time per batch.

The central S1 checklist item for physics needs the shared S1 review; this local pass is component evidence only.
