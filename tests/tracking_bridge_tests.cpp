// Offline, typed S1 conformance. All maps, confidence policies and observations
// here are explicit synthetic fixtures, not hardware calibration or clock fits.
#include <cstdio>
#include <limits>

#include "fixtures.hpp"
#include "motion_tennis/physics/batch_core.hpp"
#include "motion_tennis/physics/fakes.hpp"
#include "motion_tennis/physics/tracking_pose_bridge.hpp"

using namespace motion_tennis::physics;
using namespace motion_tennis::physics::fixtures;
using namespace motion_tennis::physics::fakes;

namespace {
int failures = 0;
int cases = 0;
int failed_cases = 0;
#define CHECK(value) do { if (!(value)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #value); } } while (false)

template <typename Function> void test(const char* name, Function function) {
    const int before = failures;
    ++cases;
    function();
    failed_cases += before == failures ? 0 : 1;
    std::printf("[%s] %s\n", before == failures ? "PASS" : "FAIL", name);
}

struct Rig {
    MatchSetup setup = build_setup();
    TrackingSourceSession source{301, filled_uuid('T'), filled_uuid('R'), 1};
    PiClockReference reference{filled_uuid('P'), 1};
    TrackingQualityPolicy quality{0.5f};
    TrackingClockMap map = tracked_pose(1, kInitialPiTimeUs, person_for_slot(1)).mapping;
    BoundedVector<PlayerBinding, kPlayers> bindings;
    TrackingPoseBridge bridge;

    Rig(bool with_map = true, bool with_bindings = true) {
        for (const auto& binding : setup.configuration.players) bindings.push_back(binding);
        CHECK(bridge.start_session(source, reference, quality) == BridgeCode::Accepted);
        if (with_bindings) CHECK(bridge.install_confirmed_bindings(bindings) == BridgeCode::Accepted);
        if (with_map) CHECK(bridge.install_clock_map(map) == BridgeCode::Accepted);
    }

    TrackingFrameBatch frame(std::uint64_t id = 1, std::uint64_t capture = kInitialPiTimeUs,
                              std::uint32_t people = 4, bool ready = true) const {
        TrackingFrameBatch batch;
        batch.status.frame_id = id;
        batch.status.camera_ready = ready;
        const auto sequence = (id - 1) * 5 + 1;
        for (std::uint32_t slot = 1; slot <= people; ++slot) {
            auto pose = tracked_pose(id, capture, person_for_slot(slot));
            pose.envelope.sender_session_id = source.session_id;
            pose.envelope.run_id = source.run_id;
            pose.envelope.sender_id = source.sender_id;
            pose.envelope.clock_id = source.clock_id;
            pose.envelope.message_seq = sequence + slot - 1;
            batch.people.push_back({pose.envelope, pose.source_frame});
            batch.status.person_ids.push_back(person_for_slot(slot));
        }
        batch.status_envelope = {0, 3, source.sender_id, source.session_id, source.run_id,
                                  sequence + people, capture + 1000, source.clock_id};
        return batch;
    }
};

void finish(BatchCore& core, FakeStateService& service, std::uint64_t now) {
    const auto update = core.prepare_commit(now);
    CHECK(update.has_value());
    if (!update) return;
    const auto result = service.commit(*update);
    CHECK(result.has_value());
    if (result) core.on_commit_result(*result);
}

void check_core_rejects(PoseObservation pose, Outcome expected) {
    const auto setup = build_setup();
    const auto initial = build_snapshot(setup);
    FakePoseSource poses;
    poses.set(person_for_slot(1), pose);
    ScriptedContactEvaluator evaluator;
    FakeStateService service(setup, initial);
    BatchCore core(setup, initial.simulation_tick, poses, evaluator);
    const auto now = kInitialPiTimeUs + 5000;
    core.admit(input_for(1, 1, now - 2000), now - 1000);
    run_batch(core, service, kInitialTick + 1, now);
    const auto feedback = core.take_feedback();
    CHECK(feedback.size() == 1);
    CHECK(!feedback.empty() && !feedback[0].committed && feedback[0].outcome == expected);
    CHECK(evaluator.observations().empty());
}

}  // namespace

int main() {
    test("four_people_preserve_full_source_and_map_provenance", [] {
        Rig rig;
        const auto batch = rig.frame();
        CHECK(rig.bridge.ingest_frame(batch, kInitialPiTimeUs + 5000) == BridgeCode::Accepted);
        for (std::uint32_t slot = 1; slot <= 4; ++slot) {
            const auto pose = rig.bridge.latest_for(person_for_slot(slot));
            CHECK(pose.has_value());
            CHECK(pose && pose->source_frame == batch.people[slot - 1].frame);
            CHECK(pose && pose->envelope == batch.people[slot - 1].envelope);
            CHECK(pose && pose->mapping == rig.map && pose->total_uncertainty_us == 1500);
            CHECK(pose && validate_tracking_pose(*pose, rig.bindings[slot - 1], rig.reference,
                                                  kInitialPiTimeUs + 5000) == BridgeCode::Accepted);
        }
    });
    test("nonidentity_anchored_map_retains_native_capture", [] {
        Rig rig(false);
        rig.map.pi_anchor_us += 500000;
        rig.map.issued_pi_us += 500000;
        rig.map.last_good_pi_us += 500000;
        rig.map.valid_from_pi_us += 500000;
        rig.map.valid_until_pi_us += 500000;
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(), kInitialPiTimeUs + 505000) == BridgeCode::Accepted);
        const auto pose = rig.bridge.latest_for(person_for_slot(1));
        CHECK(pose && pose->capture_pi_us == kInitialPiTimeUs + 500000);
        CHECK(pose && pose->source_frame.capture.value_us == kInitialPiTimeUs);
    });
    test("missing_map_is_ineligible", [] {
        Rig rig(false);
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::MissingMap);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("missing_confirmed_bindings_are_not_inferred", [] {
        Rig rig(true, false);
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::MissingBindings);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("unbound_people_never_become_slots", [] {
        Rig rig;
        auto batch = rig.frame(1, 500000, 1);
        batch.people[0].frame.person_id = 1;
        batch.status.person_ids[0] = 1;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(1) && !rig.bridge.latest_for(person_for_slot(1)));
        CHECK(rig.bridge.counters().unbound_people == 1);
    });
    test("calibration_mismatch_clears_previous_frame", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        auto bad = rig.frame(2, 510000, 1);
        bad.people[0].frame.calibration_id = 999;
        CHECK(rig.bridge.ingest_frame(bad, 515000) == BridgeCode::CalibrationMismatch);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("required_wrist_unavailable_is_not_ordinary_miss", [] {
        Rig rig;
        auto batch = rig.frame(1, 500000, 1);
        batch.people[0].frame.joints[10].state = TrackingState::Unavailable;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::Accepted);
        const auto pose = rig.bridge.latest_for(person_for_slot(1));
        CHECK(pose && !pose->tracked && pose->source_frame.joints[10].state == TrackingState::Unavailable);
        if (pose) check_core_rejects(*pose, Outcome::TrackingUnavailable);
        CHECK(rig.bridge.counters().unusable_poses == 1);
    });
    test("inferred_required_joint_and_low_confidence_are_ineligible", [] {
        Rig rig;
        auto batch = rig.frame(1, 500000, 1);
        batch.people[0].frame.joints[8].state = TrackingState::Inferred;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1))->tracked);
        batch = rig.frame(2, 510000, 1);
        batch.people[0].frame.joints[11].score = 0.49f;
        CHECK(rig.bridge.ingest_frame(batch, 515000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1))->tracked);
    });
    test("handed_arm_policy_does_not_require_other_wrist_or_ankle", [] {
        Rig rig;
        rig.bindings[0].right_hand = false;
        CHECK(rig.bridge.install_confirmed_bindings(rig.bindings) == BridgeCode::Accepted);
        auto batch = rig.frame(1, 500000, 1);
        batch.people[0].frame.joints[10].state = TrackingState::Unavailable;
        batch.people[0].frame.joints[16].state = TrackingState::Unavailable;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::Accepted);
        CHECK(rig.bridge.latest_for(person_for_slot(1))->tracked);
        batch = rig.frame(2, 510000, 1);
        batch.people[0].frame.joints[9].state = TrackingState::Unavailable;
        CHECK(rig.bridge.ingest_frame(batch, 515000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1))->tracked);
    });
    test("inclusive_80ms_gate_includes_both_uncertainties", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 578500) == BridgeCode::Accepted);
        const auto pose = rig.bridge.latest_for(person_for_slot(1));
        CHECK(pose && validate_tracking_pose(*pose, rig.bindings[0], rig.reference, 578500) == BridgeCode::Accepted);
        CHECK(pose && validate_tracking_pose(*pose, rig.bindings[0], rig.reference, 578501) == BridgeCode::StalePose);
        Rig stale;
        CHECK(stale.bridge.ingest_frame(stale.frame(), 579500) == BridgeCode::StalePose);
    });
    test("capture_uncertainty_8000_inclusive_and_8001_rejected", [] {
        Rig rig;
        auto batch = rig.frame(1, 500000, 1);
        batch.people[0].frame.capture.uncertainty_us = 8000;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::Accepted);
        batch = rig.frame(2, 510000, 1);
        batch.people[0].frame.capture.uncertainty_us = 8001;
        CHECK(rig.bridge.ingest_frame(batch, 515000) == BridgeCode::InvalidTime);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("full_capture_interval_must_fit_map_validity", [] {
        Rig rig(false);
        rig.map.valid_from_source_us = 499001;
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::MapExpired);
    });
    test("map_expiry_is_last_good_not_reissue", [] {
        Rig rig(false);
        rig.map.issued_pi_us = 500000;
        rig.map.last_good_pi_us = 400000;
        rig.map.valid_until_pi_us = 5400001;
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::InvalidMap);
        Rig normal;
        CHECK(normal.bridge.ingest_frame(normal.frame(), 5400001) == BridgeCode::MapExpired);
    });
    test("map_accuracy_and_growth_are_bounded", [] {
        Rig rig(false);
        rig.map.uncertainty_us = 2001;
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::InvalidMap);
        Rig growing(false);
        growing.map.uncertainty_us = 2000;
        growing.map.uncertainty_growth_ppb = 1000000;
        CHECK(growing.bridge.install_clock_map(growing.map) == BridgeCode::Accepted);
        CHECK(growing.bridge.ingest_frame(growing.frame(), 505000) == BridgeCode::MapUncertain);
    });
    test("growth_uses_raw_source_distance_for_nonunit_rate", [] {
        Rig rig;
        auto map = rig.map;
        map.rate_numerator = 1;
        map.rate_denominator = 2;
        map.uncertainty_us = 500;
        map.uncertainty_growth_ppb = 2000;
        SourceTime capture{1500000, 1, TimeKind::DriverFrame, 1000, true};
        MappedCapture mapped;
        CHECK(map_tracking_capture(map, capture, rig.source, rig.reference, 1005000, mapped) == BridgeCode::Accepted);
        // +/-1000 raw us becomes +/-500 Pi us. Source-anchor distance is
        // 1001000 us, so outward-rounded growth is 3 us, not 2 us on Pi time.
        CHECK(mapped.capture_pi_us == 1000000 && mapped.total_uncertainty_us == 1003);
    });
    test("full_mapped_interval_must_follow_map_acceptance", [] {
        Rig rig(false);
        rig.map.issued_pi_us = 500000;
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::MapExpired);
    });
    test("unknown_presentation_clock_mismatch_and_future_time_rejected", [] {
        Rig rig;
        auto batch = rig.frame(1, 500000, 1);
        batch.people[0].frame.capture.kind = TimeKind::Presentation;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::InvalidTime);
        batch = rig.frame(2, 510000, 1);
        batch.people[0].frame.capture.clock_id = 2;
        CHECK(rig.bridge.ingest_frame(batch, 515000) == BridgeCode::InvalidTime);
        batch = rig.frame(3, 530000, 1);
        CHECK(rig.bridge.ingest_frame(batch, 525000) == BridgeCode::InvalidTime);
    });
    test("future_processing_send_and_status_boundaries_are_rejected", [] {
        Rig rig;
        auto batch = rig.frame();
        for (auto& packet : batch.people) {
            packet.frame.processing_complete_us = 2000000;
            packet.envelope.sent_time_us = 2000000;
        }
        batch.status_envelope.sent_time_us = 2000000;
        CHECK(rig.bridge.ingest_frame(batch, 505000) == BridgeCode::InvalidTime);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        auto pose = tracked_pose(1, 500000, person_for_slot(1));
        pose.source_frame.processing_complete_us = pose.envelope.sent_time_us = 2000000;
        check_core_rejects(pose, Outcome::InvalidInput);
    });
    test("uint56_epochs_keep_fractional_rate_outward_bounds", [] {
        Rig rig;
        auto map = rig.map;
        map.source_anchor_us = kMaxWireUint64 - 1000000;
        map.pi_anchor_us = kMaxWireUint64 - 2000000;
        map.rate_numerator = 3;
        map.rate_denominator = 2;
        map.valid_from_source_us = map.source_anchor_us - 10000;
        map.valid_until_source_us = map.source_anchor_us + 10000;
        map.valid_from_pi_us = map.pi_anchor_us - 100000;
        map.last_good_pi_us = map.issued_pi_us = map.valid_from_pi_us;
        map.valid_until_pi_us = map.pi_anchor_us + 100000;
        MappedCapture mapped;
        SourceTime capture{map.source_anchor_us + 1, 1, TimeKind::DriverFrame, 1000, true};
        CHECK(map_tracking_capture(map, capture, rig.source, rig.reference, map.pi_anchor_us + 10000, mapped) == BridgeCode::Accepted);
        CHECK(mapped.capture_pi_us == map.pi_anchor_us + 1);
        CHECK(mapped.total_uncertainty_us == 2001);
        map.rate_denominator = 0;
        CHECK(map_tracking_capture(map, capture, rig.source, rig.reference, map.pi_anchor_us + 10000, mapped) == BridgeCode::InvalidMap);
    });
    test("map_id_is_immutable_and_registry_is_bounded", [] {
        Rig rig;
        auto altered = rig.map;
        altered.uncertainty_us += 1;
        CHECK(rig.bridge.install_clock_map(altered) == BridgeCode::MapConflict);
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::Accepted);
        for (std::uint64_t id = 2; id <= kTrackingMapCapacity; ++id) {
            auto next = rig.map; next.mapping_id = id;
            CHECK(rig.bridge.install_clock_map(next) == BridgeCode::Accepted);
        }
        auto extra = rig.map; extra.mapping_id = kTrackingMapCapacity + 1;
        CHECK(rig.bridge.install_clock_map(extra) == BridgeCode::Capacity);
    });
    test("duplicate_and_reordered_frames_invalidate_instead_of_resurrecting", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(2, 510000), 515000) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(2, 510000), 515001) == BridgeCode::DuplicateFrame);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        CHECK(rig.bridge.ingest_frame(rig.frame(1, 500000), 515002) == BridgeCode::ReorderedFrame);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("incomplete_packet_set_and_duplicate_person_are_rejected", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        auto bad = rig.frame(2, 510000);
        bad.status.person_ids.clear();
        CHECK(rig.bridge.ingest_frame(bad, 515000) == BridgeCode::IncompleteFrame);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        bad = rig.frame(3, 520000);
        bad.status.person_ids[1] = bad.status.person_ids[0];
        CHECK(rig.bridge.ingest_frame(bad, 525000) == BridgeCode::ContradictoryFrame);
    });
    test("contradictory_common_frame_metadata_and_invalid_joints_rejected", [] {
        Rig rig;
        auto bad = rig.frame();
        bad.people[1].frame.capture.value_us += 1;
        CHECK(rig.bridge.ingest_frame(bad, 505000) == BridgeCode::ContradictoryFrame);
        bad = rig.frame(2, 510000);
        bad.people[0].frame.joints[0].x_normalized = std::numeric_limits<float>::quiet_NaN();
        CHECK(rig.bridge.ingest_frame(bad, 515000) == BridgeCode::InvalidPose);
        bad = rig.frame(3, 520000);
        bad.people[0].frame.joints[16].coco_joint_index = 0;
        CHECK(rig.bridge.ingest_frame(bad, 525000) == BridgeCode::InvalidPose);
    });
    test("source_loss_and_id_gaps_are_separate_and_regression_invalidates", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        auto third = rig.frame(3, 520000);
        third.status.frames_dropped_total = 2;
        CHECK(rig.bridge.ingest_frame(third, 525000) == BridgeCode::Accepted);
        CHECK(rig.bridge.counters().source_frames_dropped_total == 2);
        CHECK(rig.bridge.counters().frame_sequence_gaps == 1);
        auto fourth = rig.frame(4, 530000);
        fourth.status.frames_dropped_total = 1;
        CHECK(rig.bridge.ingest_frame(fourth, 535000) == BridgeCode::SourceLossRegression);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("healthy_empty_camera_false_and_missing_person_clear_cache", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(2, 510000, 1), 515000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(2)));
        CHECK(rig.bridge.ingest_frame(rig.frame(3, 520000, 0), 525000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        CHECK(rig.bridge.ingest_frame(rig.frame(4, 530000), 535000) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(5, 540000, 0, false), 545000) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        CHECK(rig.bridge.ingest_frame(rig.frame(6, 550000, 1, false), 555000) == BridgeCode::ContradictoryFrame);
    });
    test("session_restart_requires_rebinding_and_retired_session_is_fenced", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        const auto old = rig.source;
        rig.source.session_id = filled_uuid('N');
        CHECK(rig.bridge.start_session(rig.source, rig.reference, rig.quality) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        CHECK(rig.bridge.ingest_frame(rig.frame(1, 510000), 515000) == BridgeCode::MissingBindings);
        CHECK(rig.bridge.start_session(old, rig.reference, rig.quality) == BridgeCode::RetiredSession);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
    });
    test("reference_epoch_change_clears_maps_and_does_not_allow_frame_replay", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        const auto old = rig.reference;
        rig.reference.session_id = filled_uuid('Q');
        CHECK(rig.bridge.start_session(rig.source, rig.reference, rig.quality) == BridgeCode::Accepted);
        CHECK(!rig.bridge.latest_for(person_for_slot(1)));
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::ReferenceMismatch);
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505001) == BridgeCode::DuplicateFrame);
        CHECK(rig.bridge.start_session(rig.source, old, rig.quality) == BridgeCode::RetiredSession);
    });
    test("same_session_quality_change_preserves_frame_and_message_fences", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        CHECK(rig.bridge.start_session(rig.source, rig.reference, TrackingQualityPolicy{0.6f}) == BridgeCode::Accepted);
        CHECK(rig.bridge.install_confirmed_bindings(rig.bindings) == BridgeCode::Accepted);
        CHECK(rig.bridge.install_clock_map(rig.map) == BridgeCode::Accepted);
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505001) == BridgeCode::DuplicateFrame);
    });
    test("session_history_has_explicit_capacity_failure", [] {
        Rig rig;
        for (std::uint8_t value = 1; value <= kTrackingSessionHistory; ++value) {
            rig.source.session_id = filled_uuid(value);
            CHECK(rig.bridge.start_session(rig.source, rig.reference, rig.quality) == BridgeCode::Accepted);
        }
        rig.source.session_id = filled_uuid(100);
        CHECK(rig.bridge.start_session(rig.source, rig.reference, rig.quality) == BridgeCode::Capacity);
    });
    test("diagnostics_are_bounded_and_configuration_has_no_fabricated_timestamp", [] {
        Rig rig;
        const auto configuration = rig.bridge.take_diagnostics();
        CHECK(!configuration.empty() && !configuration[0].observed_time_valid);
        for (std::uint64_t id = 1; id <= 80; ++id) {
            CHECK(rig.bridge.ingest_frame(rig.frame(id, 500000 + id * 1000, 0), 505000 + id * 1000) == BridgeCode::Accepted);
        }
        const auto diagnostic = rig.bridge.take_diagnostics();
        CHECK(diagnostic.size() == kTrackingDiagnosticCapacity);
        CHECK(rig.bridge.counters().diagnostics_lost == 16);
        CHECK(diagnostic[0].observed_time_valid);
    });
    test("core_rejects_legacy_default_and_lossy_provenance", [] {
        check_core_rejects(PoseObservation{1, 500000, 1, true}, Outcome::InvalidInput);
        auto pose = tracked_pose(1, 500000, person_for_slot(1));
        pose.calibration_id = 999;
        check_core_rejects(pose, Outcome::InvalidInput);
        pose = tracked_pose(1, 500000, person_for_slot(1));
        pose.envelope.sender_session_id = filled_uuid('N');
        check_core_rejects(pose, Outcome::InvalidInput);
        pose = tracked_pose(1, 500000, person_for_slot(1));
        pose.mapping.mapping_id = 0;
        check_core_rejects(pose, Outcome::TrackingUnavailable);
        pose = tracked_pose(1, 500000, person_for_slot(1));
        pose.total_uncertainty_us = 0;
        check_core_rejects(pose, Outcome::InvalidInput);
    });
    test("core_rechecks_empty_status_after_query_before_commit", [] {
        Rig rig;
        CHECK(rig.bridge.ingest_frame(rig.frame(), 505000) == BridgeCode::Accepted);
        const auto initial = build_snapshot(rig.setup);
        FakeStateService service(rig.setup, initial);
        ScriptedContactEvaluator evaluator;
        BatchCore core(rig.setup, initial.simulation_tick, rig.bridge, evaluator);
        core.admit(input_for(1, 1, 503000), 504000);
        const auto request = core.release(kInitialTick + 1);
        core.on_query_response(service.query(*request), 505000);
        CHECK(rig.bridge.ingest_frame(rig.frame(2, 506000, 0), 507000) == BridgeCode::Accepted);
        finish(core, service, 507000);
        const auto feedback = core.take_feedback();
        CHECK(feedback.size() == 1 && feedback[0].outcome == Outcome::TrackingUnavailable && !feedback[0].committed);
        CHECK(core.counters().rebuilds == 1);
    });
    test("core_rechecks_age_and_map_expiry_after_wait", [] {
        const auto setup = build_setup();
        const auto initial = build_snapshot(setup);
        auto pose = tracked_pose(1, 500000, person_for_slot(1));
        pose.mapping.valid_until_pi_us = 505001;
        FakePoseSource poses; poses.set(person_for_slot(1), pose);
        FakeStateService service(setup, initial);
        ScriptedContactEvaluator evaluator;
        BatchCore core(setup, initial.simulation_tick, poses, evaluator);
        core.admit(input_for(1, 1, 503000), 504000);
        const auto request = core.release(kInitialTick + 1);
        core.on_query_response(service.query(*request), 505000);
        finish(core, service, 505002);
        const auto feedback = core.take_feedback();
        CHECK(feedback.size() == 1 && feedback[0].outcome == Outcome::TrackingUnavailable && !feedback[0].committed);
    });
    test("core_diagnostic_pose_matches_surviving_original_input", [] {
        const auto setup = build_setup();
        const auto initial = build_snapshot(setup);
        FakePoseSource poses;
        poses.set(person_for_slot(1), tracked_pose(10, 500000, person_for_slot(1)));
        poses.set(person_for_slot(2), tracked_pose(20, 500000, person_for_slot(2)));
        FakeStateService service(setup, initial);
        ScriptedContactEvaluator evaluator;
        BatchCore core(setup, initial.simulation_tick, poses, evaluator);
        core.admit(input_for(1, 1, 490000), 504000);
        core.admit(input_for(2, 1, 504000), 504500);
        const auto request = core.release(kInitialTick + 1);
        core.on_query_response(service.query(*request), 515000);
        finish(core, service, 521000);
        bool staged = false;
        for (const auto& trace : core.take_trace()) {
            if (trace.kind != TraceKind::DecisionStaged) continue;
            staged = true;
            CHECK(trace.transaction.controller_id == controller_for_slot(2));
            CHECK(trace.person_id == person_for_slot(2));
            CHECK(trace.pose_raw_capture.value_us == 500000 && trace.pose_mapping_id == 1);
            CHECK(trace.pose_total_uncertainty_us == 1500 && trace.observed_pi_us == 521000);
            CHECK(trace.tracking_session_id == filled_uuid('T'));
        }
        CHECK(staged);
    });
    std::printf("%d/%d tracking bridge checks passed\n", cases - failed_cases, cases);
    return failed_cases == 0 ? 0 : 1;
}
