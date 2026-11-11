# S1 typed tracking consumer and handoff

[Repository home](../README.md) · [Batch core](s1-batch-core.md) · [Shared stage](https://github.com/motion-tennis/documentation/blob/main/docs/project-status.md)

## Task and evidence boundary

This is the user-authorized offline S1 tracking input bridge and validation slice, prepared on `task/s1-tracking-consumer-compatibility` from physics `51f153593d83bf4872302f7ab286ce458389cd5f`. The shared status baseline is `documentation@683cd45352e38ac66738fdee3bd5dba1614a9ab0`; the tracking producer baseline is `0509fbd8cc15dd7b5a1a640814b66e890aef1ca3`. On 2026-11-11, the project user explicitly authorized committing, merging and pushing the tracking, physics and documentation changes to remote `main`. The containing commits and remote refs establish publication separately from validation. S1 remains active/incomplete; no later stage or hardware claim is released.

The accepted contract remains `motion.v03` / `0.3-proposed` / `webcam-pi3-doubles-v3`. Its schema and profile are unchanged. The bridge consumes typed mirrors of the actual producer's v0.3 fields. A separate compatibility script validates and serializes/decodes real producer envelopes using the generated Python codec before projecting those fields into C++. There is no C++ wire decoder, socket, camera access, temporal person-association algorithm, clock fitter or geometric fusion here.

## Typed input and explicit setup

[`TrackingPoseBridge`](../include/motion_tennis/physics/tracking_pose_bridge.hpp) implements `PoseSource`. The caller serializes access to it; this offline API does not introduce an I/O thread or a gameplay dependency on debug collection.

1. `start_session(source, reference, quality)` declares the exact tracking sender/session/run/clock and Pi reference session/clock. The quality policy requires an explicit finite confidence threshold greater than zero and at most one. The test threshold `0.5` is a synthetic fixture policy; a calibrated deployment threshold remains open.
2. `install_confirmed_bindings(bindings)` installs at most four explicit controller/person/slot/calibration/association bindings. Slots and person IDs are never inferred from discovery order or numeric proximity. Source session, unique slots/controllers/persons/avatars and nonzero identity/version fields are checked.
3. `install_clock_map(map)` installs an immutable caller-supplied Pi-owned map, including source and reference identities, map ID/version, anchors, rational rate, source/Pi validity windows, issuance/last-good times and uncertainty. A changed record under an existing map ID is rejected. Reinstalling the identical map does not refresh its last-good time.
4. `ingest_frame(batch, now_pi_us)` accepts one complete atomic `TrackingFrameBatch`: the common `FrameStatus`, its envelope and exactly the per-person `SkeletonFrame` packets named by that status. Partial UDP assembly, assembly timeout and packet buffering remain future transport work. The caller must check bounded-collection insertion failures while decoding or constructing a batch.

All 17 COCO joint indices, coordinates, scores and states are retained, together with image dimensions, calibration, original capture boundary/clock/uncertainty, native processing/send times and envelope identities. Coordinates stay normalized to the original rectified image. The wire does not carry a metric depth estimate or the producer manifest's model/preprocessing labels; those remain external profile/calibration evidence. No image coordinate is relabelled as metres or as a virtual avatar position.

## Eligibility, invalidation and loss

Every frame validates v0.3 envelope identity, nonzero bounded message/frame IDs, advancing frame/message order, exact status membership, unique people, consistent common-frame metadata, finite normalized coordinates and explicit joint states. An incomplete, contradictory, duplicate, reordered or otherwise rejected frame clears the old eligible cache. Its newer frame identity is fenced even when the frame is rejected, so a contradictory reconstruction cannot resurrect prior data.

A healthy empty frame, unavailable-camera status or removal of a person from the current status invalidates the corresponding cached poses immediately. A false camera-ready flag with nonempty people is contradictory. Source-reported loss totals cannot regress within a source session; source loss, source frame-ID gaps, rejected frames, unusable poses and dropped diagnostic records have separate counters. Frame-ID gaps are not inferred UDP-loss measurements.

Per-person eligibility requires the source's `TRACKED` state and tracked, sufficiently scored playing-hand shoulder/elbow/wrist plus both shoulder and hip anchors. Inferred or unavailable required joints remain unusable; optional unavailable joints are preserved. An unusable person can remain inspectable as `tracked=false`, and the core produces a tracking failure rather than an ordinary miss.

Changing the tracking source or Pi reference invalidates cached poses, maps and bindings. New bindings and applicable maps are required. Retired tracking sessions and Pi reference epochs cannot be reactivated. Same-source policy/reference reconfiguration preserves source frame/message/loss high-water marks; it does not reset producer identities. The bounded history rejects further transitions when full rather than silently evicting a fence.

## Clock checks and conservative bounds

Raw source timestamps remain unchanged. The bridge maps a capture using anchored integer arithmetic:

```text
Pi capture = Pi anchor + floor((raw capture - source anchor) * numerator / denominator)
```

Both rational rate components must be positive and at most `1e9`; the source and Pi validity windows are each bounded to five seconds for bounded arithmetic. Outward-rounded lower/upper capture intervals preserve microsecond distinctions at uint56 epochs, including nonunit rates. The complete raw and mapped capture uncertainty intervals must fit the map's validity and follow its acceptance time. Last-good refresh, rather than map reissuance, bounds the five-second expiry.

Capture must declare a valid sensor-capture or driver-frame boundary, the exact source clock and uncertainty at most 8000 microseconds. The Pi-map error bound must remain at most 2000 microseconds. Declared growth adds Pi-time error per elapsed raw-source time; current map readiness uses an outward inverse bound, including the maximum admissible map error. Capture uncertainty is rate-propagated and combined with the map bound. Eligibility requires `now - mapped_capture + total_uncertainty <= 80000` microseconds. No cross-clock subtraction or half-RTT estimate substitutes for a map.

Native processing, packet-send and status-send times are checked for declared ordering, map applicability and definitely future values at the supplied Pi receipt boundary. Only the declared map error applies to these event checks; this task supplies no measured accuracy for native callback placement. A valid recent capture cannot make an impossible future processing/send boundary valid.

`PoseObservation` retains the full source frame/envelope, immutable map and explicit policy, mapped capture and total bound. Defaults are ineligible. [`BatchCore`](../src/batch_core.cpp) independently revalidates these fields against the installed binding and queried Pi reference after waiting and before submission, including current-source invalidation. It cannot admit a legacy bool/timestamp-only pose, mismatched calibration/session, absent map or understated uncertainty. A removed input rebuilds the candidate from the queried snapshot. Selected native pose capture values are retained in decisions and bounded trace evidence.

## Bounded resources and diagnostics

The bridge uses fixed-capacity storage: four bindings/poses, 17 joints/person, 16 immutable maps, 64 retired source sessions, 64 retired reference epochs and 64 diagnostic records. Full registries reject explicitly. Diagnostic overflow counts loss and never invokes an exporter callback. Configuration operations without a supplied clock have `observed_time_valid=false`; the zero storage value is not a fabricated timestamp.

On the tested MSVC x64 host, `sizeof(TrackingPoseBridge)` is 18144 bytes, `sizeof(PoseObservation)` 704 bytes and `sizeof(BatchCore)` 68968 bytes. These are host-layout observations, not target ABI, stack-headroom or allocation-performance measurements. The bridge/core use fixed storage; test fakes can allocate.

## Validation and handoff

Windows host checks on 2026-11-11 used CMake 4.3.0-rc2, Visual Studio 2026 / MSVC 19.51.36257 and the Windows SDK 10.0.26100.0:

```powershell
cmake -S . -B build-merge -G "Visual Studio 18 2026" -A x64
cmake --build build-merge --config Release
ctest --test-dir build-merge -C Release --output-on-failure
git diff --check
```

Both CTest targets passed: the original 23 batch-core checks and 34 tracking-bridge checks, 57 checks total. The focused suite covers binding/session/reference fences, map absence/expiry/accuracy/acceptance, nonunit rate and uint56 rounding, uncertainty-inclusive age boundaries, handed joint eligibility, calibration, future processing/send times, empty/unavailable/incomplete/contradictory frames, source loss versus gaps, diagnostic capacity and post-query invalidation/rechecks. It also confirms that provenance follows the surviving input after an earlier member is removed.

The adjacent tracking repository's [actual producer/consumer checker](https://github.com/motion-tennis/motion-tennis-camera-tracking/blob/main/scripts/check_consumer_compatibility.py) passed 17 C++ consumer cases from five real producer-emitted synthetic source frames / 15 canonically serialized envelopes. A separate shared-wire check passed six cases / 20 envelopes, maximum 497 encoded bytes. Seven debug scenarios passed their expected integrity outcomes, including visible loss rejection. The [saved compatibility report](https://github.com/motion-tennis/motion-tennis-camera-tracking/blob/main/evidence/s1-consumer-compatibility-results.json) pins source hashes, environment, fixture map and explicit limitations. Its bindings, controller inputs and Pi map are declared synthetic; the contact evaluator is a test fake. The checker does not prove calibrated physical association, metric geometry, live network delivery, clock fitting, C++ wire decoding or target-device behavior.

Remaining bounded work includes the separate controller transaction replay correction (scheduler deduplication still covers queued inputs only), complete physics/game rejected-input and canonical identity agreement, an eventual C++ wire/packet-assembly adapter, actual Pi-issued map acquisition and calibrated 2.5D fusion under the appropriate later scope. No full doubles rules, hardware timing or shared S1 acceptance follows from this handoff.
