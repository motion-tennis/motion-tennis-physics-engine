# Physics engine staged workflow

[Repository home](../README.md) · [Orientation](orientation.md) · [Project status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md) · [Canonical stages and gates](https://github.com/motion-tennis/documentation/blob/main/docs/project-stages.md) · [Shared task and handoff process](https://github.com/motion-tennis/documentation/blob/main/docs/contribution-workflow.md)

Read the central status to find the active stage. This local table explains the physics contribution to shared gates; it does not authorize every listed feature or create an independent schedule. A passing component task never automatically advances the project.

| Stage | Bounded local contribution | Evidence to hand off | Deferred scope |
| --- | --- | --- | --- |
| S0 — Shared contract/coordination baseline | Review versioned full-match state, profile compatibility, bounded transaction/commit fixtures and task boundaries | Agreed field/invariant cases, ordering/rollback expectations, size evidence plan and dependencies | New runtime implementation, real fusion and complete game rules |
| S1 — Isolated minimal component slices | Build an authorized deterministic fixed-step ball/batch core using fake controller, pose and state-service adapters | Replayable integration/order/rollback/periodic-only cases and preserved transaction/query/batch trace IDs | Real network/sensor integration, calibrated fusion and full doubles movement/scoring |
| S2 — Feasibility/transport | Exercise actual Pi/Windows state transport with synthetic capacity load and bounded queues; add scoped clock/marker/continuous-source probes | Declared 800-transaction/s synthetic workload, service/queue/drop measurements and reproducible probe evidence | An integrated playable game or inference that synthetic throughput proves four-human play |
| S3 — One-controller end-to-end diagnostic | Join real pose and IMU through the required ball query/response, atomic commit and controller feedback | Full correlated diagnostic path, Pi mapping versions/uncertainty, marker evidence and explicit failures | Full doubles, unverified multilateral gameplay and broad rule implementation |
| S4 — Four-controller 2v2 playable game | Add stable four-player binding, deterministic automatic movement, gesture serve, rally/scoring and full atomic match candidates | Four-human rule/identity/restart fixtures, human swing evidence and consistent state/feedback across both PC views | New transport alternatives and acceptance claims without measured complete outcomes |
| S5 — Matched WLAN evaluation | Compare external AP with wired Pi/laptop against Pi hotspot with wired laptop under the same game/input load | Matched configurations, CPU/queue/service/timing results and complete fault/loss accounting | Silent changes to samples, batching, marker behavior, deadlines or workload |
| S6 — Reproducible demonstrator | Stabilize verified deployment, startup, recovery and evidence capture on the declared Pi | Fresh-setup reproduction, pinned versions/configuration, commands and complete run bundle | Unrequested physics features or unproven real-time guarantees |
| S7 — Deferred true 3D/alternatives | Perform only an explicitly authorized research/migration slice | New scope, contract impact and evidence plan | Automatic progression from S6; unapproved depth/sensor/transport/hardware changes |

S0 reviews and verifies the v0.3 full-match contract before dependent runtime slices. S1 deliberately uses fakes and a minimal deterministic core; it does not promise the final fusion model, avatar planner or tennis rule engine. The real Pi is assessed in S2 without conflating hardware feasibility with S4 playability.

## Task boundaries and coordination

Before work, inspect the working tree and central status. Use the shared template to name the stage, owner, repository/files, contract revision, one bounded deliverable, dependencies, acceptance evidence and explicit deferred scope. Agree interface/fixture ownership with parallel human or AI contributors; no component may invent its own match schema or change timing semantics to make a local test pass.

Complete authorized work and appropriate checks without repeated approval requests for routine choices. Record missing dependencies, continue independent in-scope work and place broader ideas in the handoff. An explicit human request may authorize a bounded future-stage slice; record the exception and unresolved dependencies without advancing the global stage or implying that other components are ready.

If central status is unavailable, obtain verified pinned status/task evidence before runtime implementation. Otherwise keep to authorized inspection/planning; do not infer an open stage from this table, an old README or another contributor's branch. Stage transitions and common acceptance belong to the [central process](https://github.com/motion-tennis/documentation/blob/main/docs/contribution-workflow.md).

At handoff, list changed files, exact checks/results, evidence locations, dependency gaps and the next bounded option. Preserve unrelated work and do not commit/publish unless authorized. A local fixture pass is evidence for that slice, not permission to implement the next row.

## Verification boundary

The README lists the checks that exist now. Add real build/run/test commands alongside later source. Separate deterministic fixture correctness, measured Pi compute time, real query/commit service, camera freshness and complete controller latency. Preserve every started transaction and failure in denominators; low latency on rejected inputs does not prove useful gameplay. A scheduler tick or PREEMPT_RT configuration alone cannot establish a complete wireless/Windows timing guarantee.
