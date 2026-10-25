# Physics engine orientation

[Repository home](../README.md) · [Local workflow](workflow.md) · [Current project status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md) · [Shared stages](https://github.com/motion-tennis/documentation/blob/main/docs/project-stages.md)

## Purpose and ownership

This component runs on the available Raspberry Pi 3 Model B. It combines controller samples and continuously acquired webcam pose, owns runtime source-to-Pi clock mappings and calculates authoritative match changes. The Windows game service stores the versioned committed result. Camera inference and rendering stay on the laptop.

| Input/output | Responsibility |
| --- | --- |
| Controller samples and pose observations | Validate identity, calibration, capture age and uncertainty; preserve per-source/sample/frame provenance and bounded histories |
| Setup and control | Own confirmed controller/person/team associations and readiness; coordinate source-to-Pi clock mapping |
| Required game query/response | Send one logical query per selected transaction and receive an atomic versioned snapshot before calculation |
| Candidate match update/commit result | Integrate and evaluate ordered inputs; submit one atomic version-checked candidate and resolve acceptance before confirming gameplay feedback |
| Controller feedback and trace | Return correlated outcomes, retain local deadlines/failure accounting and export diagnostic evidence asynchronously |

Pi alone calculates ball motion, automatic avatar roots/headings, body-relative racket contacts, serve phases and score changes. The PC is not a second scorer or avatar controller. Periodic-only batches use the same query/commit path. The independent IMU threshold marker is diagnostic and never gates the continuous 200 Hz controller stream.

## What exists and what remains open

Repository inspection on 2026-10-25 found documentation and the MIT license only: no fusion/simulation runtime, transport, build files, dependency lock or tests. C++20/CMake and a verified 64-bit Pi OS profile remain proposed. Hardware availability does not establish timing capacity. Reinspect current files for each task and use the [central status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md) for project progress.

The proposed shared schema currently covers ball state and per-input decisions, not the complete doubles roster, avatar, serve/rally and score state. Its [versioned extension](https://github.com/motion-tennis/documentation/blob/main/docs/protocol.md#doubles-match-contract-extension), atomicity and bounded-size fixtures are prerequisites for dependent implementation. Do not invent private fields in this component or silently change the published stream/timing profile.

## Reading order

1. Read [project status](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md), [stage gates](https://github.com/motion-tennis/documentation/blob/main/docs/project-stages.md) and the [task/handoff process](https://github.com/motion-tennis/documentation/blob/main/docs/contribution-workflow.md) to identify the authorized slice.
2. Read this page, [local workflow](workflow.md) and the [README](../README.md).
3. Read the [physics design](https://github.com/motion-tennis/documentation/blob/main/docs/physics.md), [gameplay](https://github.com/motion-tennis/documentation/blob/main/docs/gameplay.md), [assignment](https://github.com/motion-tennis/documentation/blob/main/docs/player-assignment.md) and [coordinates/time](https://github.com/motion-tennis/documentation/blob/main/docs/coordinates-and-time.md).
4. Read the assigned [protocol](https://github.com/motion-tennis/documentation/blob/main/docs/protocol.md)/[schema](https://github.com/motion-tennis/documentation/blob/main/spec/motion.proto)/[profile](https://github.com/motion-tennis/documentation/blob/main/spec/reference-profile.json) boundary, [timing](https://github.com/motion-tennis/documentation/blob/main/docs/timing.md) and [debug evidence](https://github.com/motion-tennis/documentation/blob/main/docs/debugging.md).

## Bounded S0 task options

When central status authorizes S0, select one deliverable: review the full-match candidate/rollback invariants; specify deterministic ordering and periodic-only fixtures; enumerate bounded-size/query/commit/failure cases; or define fake state/pose/controller adapter contracts for the later minimal slice. Keep these contract and planning tasks separate from new simulation, networking, fusion, movement or scoring code. Shared-schema changes need an explicitly coordinated documentation-repository scope.

## Checks available now

From this repository, `git diff --check` checks tracked-change whitespace. Inspect new files for trailing whitespace and final newlines separately. With the documentation checkout available, run its existing checker there: `python3 scripts/check_docs.py`. It checks supported local documentation constraints, not Pi capacity, clock accuracy, fusion quality or protocol interoperability. Real build/test commands must accompany the implementation that makes them valid.
