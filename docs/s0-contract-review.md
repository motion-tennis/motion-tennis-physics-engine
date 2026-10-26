# S0 physics contract review and S1 handoff

[Repository home](../README.md) · [Local workflow](workflow.md) · [Central status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md) · [Shared contract](https://github.com/motion-tennis/documentation/blob/main/docs/protocol.md)

## Review scope and evidence

Task: S0 component-side contract review and bounded S1 handoff, 2026-10-26. The starting shared revision is `documentation@95c2592`; this review accompanies the S0 schema/profile revision identified by the exact hashes in the shared evidence. The [S0 baseline](https://github.com/motion-tennis/documentation/blob/main/docs/s0-baseline.md) pins the final contract hashes and codec/size results. The candidate is `motion.v03`, protocol `0.3-proposed`, profile `webcam-pi3-doubles-v3`; [ADR 0011](https://github.com/motion-tennis/documentation/blob/main/docs/adr/0011-full-match-contract-and-state-stream.md) records the full-match state channel. This document records an AI technical review, not a human stage decision or an implemented component. The central ledger alone releases S1.

Allowed work here is documentation only. Shared schema, codec fixtures and resource checks belong in `documentation`; runtime integration and hardware measurements retain their later stage gates. Proposed numerical targets remain unmeasured. No later-stage exception is claimed.

## Producer and consumer expectations

- Pi is the sole match calculator and serialized writer: ball, four avatars, serve/rally state, rule events and score form one candidate. The PC either commits all of it against one base version or rejects all of it.
- Preserve the required `ball_state_request`/`ball_state_response`, one query identity per selected transaction, deterministic order and at most eight entries with two/controller. Periodic-only batches use the same path with an empty query list.
- Send typed `MatchSetup` under the current `writer_epoch` and registered physics control session. Recover a lost ACK by the identical command/configuration digest; conflicting payload or an old writer cannot reinstall setup.
- Reference an installed immutable roster/configuration revision. One-to-one controller/session/person/avatar/slot bindings survive court-end and front/back changes. A changed roster or session cannot silently reinterpret queued samples or a candidate created under an earlier revision.
- Recheck identity, calibration, source capture age, clock-map applicability and expiry after waiting and before submission. Rejected/removed input requires a rebuilt candidate; rollback includes avatars, rules, score and events, not only the ball.
- Retain game session, writer epoch, batch ID and original canonical request content for retry/recovery. A retry cannot advance a point twice; ambiguous acceptance pauses the writer until the original identity is resolved. Confirm gameplay feedback only after accepted commit.
- Record original sample/pose/history and Pi map versions, uncertainty, roster/configuration and decision/event provenance. Diagnostic markers never create inputs or change processing order. Technical failures never become sporting faults or points.

The proposed state path uses dedicated TCP 41010 with a four-byte big-endian length prefix and an 8,192-byte envelope cap, separate from control TCP 41041. Controller and pose UDP remain capped at 1,200 bytes. Bounded partial-frame assembly, coalesced frames, connection loss and head-of-line deadline exposure need explicit runtime tests in S1/S2; the transport change establishes no latency guarantee.

## Required contract cases

| Case | Expected result and handoff evidence |
| --- | --- |
| Eight inputs across four controllers | Deterministic ordered decisions, one common snapshot, one final candidate and one atomic commit |
| Input invalidated after query | Rebuild from the original snapshot without its tentative ball/avatar/rule/score effects |
| Stale base version or wrong writer/session | Explicit rejection; candidate has no committed side effects or successful gameplay feedback |
| Duplicate accepted batch / same identity with altered payload | Retained result for the identical retry; reject the altered retry rather than treating it as new work |
| Periodic-only advance | Ball/avatar state may change without invented controller transactions or cues |
| Event retry, second fault, point transition | Stable committed identities prevent double advancement; ordinary sample `MISS` alone has no scoring authority |
| Source/reference restart | Invalidate affected live maps/bindings and pending work; historical provenance remains inspectable |
| Maximum match and decisions | Shared generated-codec size/resource proof, including explicit transport framing and bounded queues |

## First bounded S1 task

**Outcome:** a deterministic fixed-step ball/batch core with fake controller, pose and state-service adapters. It carries the complete contract state but does not implement the final avatar planner, sensor fusion or tennis rules. Allow a minimal C++20 build, the core/adapters and focused fixtures only.

**Dependencies:** accepted shared schema/profile and fixture hashes, deterministic ordering and state-copy/rollback conventions, and the paired game-store fixture contract. Pin actual compiler/build dependencies when implementation starts.

**Acceptance:** known-answer movement from a fixed seed/state; stable ordering for simultaneous inputs; periodic-only progress; rollback after one rejected member; stale-version and epoch/session rejection; identical retry versus conflicting payload; bounded queue/one-batch-in-flight overflow behavior; preserved query/transaction/commit trace identities. Compare complete state before/after failed commits. Save reproducible commands and output; no Pi throughput or real-network claim follows from host fixtures.

## Review disposition

The full-match boundary is technically coherent when atomic rollback, setup revision fences, deterministic event identity and bounded transport are applied together. Final schema/codec evidence and contract findings are recorded centrally. Real fusion, networking, clock estimation, movement/scoring and Pi timing remain unimplemented.
