// S1 acceptance fixtures for the deterministic batch core. Each case maps to an
// item of the "First bounded S1 task" in docs/s0-contract-review.md.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <vector>

#include "fixtures.hpp"
#include "motion_tennis/physics/batch_core.hpp"
#include "motion_tennis/physics/fakes.hpp"
#include "motion_tennis/physics/simulation.hpp"

using namespace motion_tennis::physics;
using namespace motion_tennis::physics::fixtures;
using motion_tennis::physics::fakes::FakePoseSource;
using motion_tennis::physics::fakes::FakeStateService;
using motion_tennis::physics::fakes::ScriptedContactEvaluator;

namespace {

int g_failures = 0;

void report_failure(const char* file, int line, const char* expression) {
    ++g_failures;
    std::printf("    FAILED %s:%d: %s\n", file, line, expression);
}

#define CHECK(condition)                                     \
    do {                                                     \
        if (!(condition)) {                                  \
            report_failure(__FILE__, __LINE__, #condition);  \
        }                                                    \
    } while (false)

bool near(float actual, double expected, double tolerance) {
    return std::fabs(static_cast<double>(actual) - expected) <= tolerance;
}

// Complete fake environment around one writer.
struct Rig {
    MatchSetup setup = build_setup();
    Snapshot initial = build_snapshot(setup);
    FakeStateService service{setup, initial};
    FakePoseSource poses;
    ScriptedContactEvaluator evaluator;
    BatchCore core{setup, initial.simulation_tick, poses, evaluator};

    Rig() { track_all(kInitialPiTimeUs); }

    void track_all(std::uint64_t capture_pi_us, std::uint64_t frame_id = 10) {
        for (std::uint32_t slot = 1; slot <= 4; ++slot) {
            poses.set(person_for_slot(slot), tracked_pose(frame_id, capture_pi_us, person_for_slot(slot)));
        }
    }
};

std::vector<FeedbackResult> drain(BatchCore& core) {
    const auto feedback = core.take_feedback();
    return std::vector<FeedbackResult>(feedback.begin(), feedback.end());
}

// ---------------------------------------------------------------------------
// Known-answer integration

void test_free_fall_known_answer() {
    const SimulationParameters parameters;
    Ball ball{Vector3{0.0f, 2.0f, 0.0f}, Vector3{3.0f, 0.0f, 0.0f}, 0.033f, 1};
    integrate_ball_tick(ball, parameters);
    // One semi-implicit step: v = -g dt, y = y0 + v dt.
    CHECK(near(ball.velocity_m_s.y, -0.04905, 1e-7));
    CHECK(near(ball.position_m.y, 2.0 - 0.04905 * 0.005, 1e-6));
    CHECK(near(ball.position_m.x, 0.015, 1e-7));

    for (int tick = 1; tick < 40; ++tick) {
        integrate_ball_tick(ball, parameters);
    }
    // Closed form after n steps: v = -g dt n, y = y0 - g dt^2 n(n+1)/2.
    const double n = 40.0;
    const double dt = 0.005;
    CHECK(near(ball.velocity_m_s.y, -9.81 * dt * n, 1e-5));
    CHECK(near(ball.position_m.y, 2.0 - 9.81 * dt * dt * n * (n + 1.0) / 2.0, 1e-5));
    CHECK(near(ball.position_m.x, 3.0 * dt * n, 1e-5));
    CHECK(ball.velocity_m_s.x == 3.0f);
}

void test_ground_bounce_restitution() {
    const SimulationParameters parameters;
    Ball ball{Vector3{0.0f, 0.04f, 0.0f}, Vector3{0.0f, -2.0f, 0.0f}, 0.033f, 1};
    integrate_ball_tick(ball, parameters);
    // Pre-contact speed 2.04905 m/s reflects with restitution 0.75; the
    // 3.2 mm penetration is mirrored above the contact height.
    CHECK(near(ball.velocity_m_s.y, 2.04905 * 0.75, 1e-5));
    CHECK(near(ball.position_m.y, 0.033 + (0.033 - (0.04 - 2.04905 * 0.005)) * 0.75, 1e-6));
    CHECK(ball.position_m.y >= ball.radius_m);

    // A resting ball never tunnels below the court over many ticks.
    for (int tick = 0; tick < 2000; ++tick) {
        integrate_ball_tick(ball, parameters);
        CHECK(ball.position_m.y >= ball.radius_m * 0.999f);
    }
}

void test_avatar_bounded_step() {
    const SimulationParameters parameters;
    AvatarState avatar;
    avatar.player_slot = 1;
    avatar.position_m = Vector3{0.0f, 0.0f, 0.0f};
    avatar.target_m = Vector3{0.0f, 0.0f, 1.0f};
    avatar.movement = MovementMode::Covering;
    integrate_avatar_tick(avatar, parameters);
    CHECK(near(avatar.position_m.z, 0.025, 1e-7));
    CHECK(near(avatar.velocity_m_s.z, 5.0, 1e-6));
    // 1 m at 25 mm/tick needs about 40 ticks; float rounding may add one.
    // The root never overshoots and snaps exactly onto the target.
    for (int tick = 1; tick < 42; ++tick) {
        integrate_avatar_tick(avatar, parameters);
        CHECK(avatar.position_m.z <= 1.0f);
    }
    CHECK(avatar.position_m.z == 1.0f);
    CHECK(avatar.velocity_m_s == Vector3{});
    CHECK(avatar.velocity_m_s == Vector3{});
}

// ---------------------------------------------------------------------------
// Replay determinism

struct ScenarioResult {
    Snapshot snapshot;
    std::vector<FeedbackResult> feedback;
    SideState side;
};

ScenarioResult run_reference_scenario() {
    Rig rig;
    rig.evaluator.script_hit(controller_for_slot(1), 1, Vector3{0.5f, 6.0f, -12.0f});
    rig.evaluator.script_hit(controller_for_slot(3), 2, Vector3{-0.5f, 4.0f, 11.0f});
    ScenarioResult result;
    std::uint64_t now = kInitialPiTimeUs;
    for (std::uint64_t tick = kInitialTick + 1; tick <= kInitialTick + 40; ++tick) {
        now += 5000;
        rig.track_all(now - 3000, tick);
        if (tick == kInitialTick + 5) {
            rig.core.admit(input_for(1, 1, now - 2000), now - 1000);
            rig.core.admit(input_for(2, 1, now - 1500), now - 900);
        }
        if (tick == kInitialTick + 20) {
            rig.core.admit(input_for(3, 2, now - 2000), now - 1000);
            rig.core.admit(input_for(4, 1, now - 1000), now - 800);
        }
        run_batch(rig.core, rig.service, tick, now);
        const auto feedback = drain(rig.core);
        result.feedback.insert(result.feedback.end(), feedback.begin(), feedback.end());
    }
    result.snapshot = rig.service.snapshot();
    result.side = rig.core.side_state();
    return result;
}

void test_replay_is_bit_identical() {
    const ScenarioResult first = run_reference_scenario();
    const ScenarioResult second = run_reference_scenario();
    CHECK(first.snapshot == second.snapshot);
    CHECK(first.feedback == second.feedback);
    CHECK(first.side == second.side);
    CHECK(first.snapshot.state_version == 41);
    CHECK(first.snapshot.simulation_tick == kInitialTick + 40);
    CHECK(first.feedback.size() == 4);
}

// ---------------------------------------------------------------------------
// Periodic-only work

void test_periodic_only_batch() {
    Rig rig;
    const auto request = rig.core.release(kInitialTick + 1);
    CHECK(request.has_value());
    CHECK(request->queries.empty());
    rig.core.on_query_response(rig.service.query(*request), kInitialPiTimeUs + 5000);
    const auto update = rig.core.prepare_commit(kInitialPiTimeUs + 5000);
    CHECK(update.has_value());
    CHECK(update->decisions.empty());
    CHECK(update->rule_events.empty());
    const auto result = rig.service.commit(*update);
    CHECK(result.has_value() && result->status == CommitStatus::Accepted);
    rig.core.on_commit_result(*result);

    const Snapshot& committed = rig.service.snapshot();
    CHECK(committed.state_version == 2);
    CHECK(committed.simulation_tick == kInitialTick + 1);
    CHECK(committed.match.ball.position_m.y < 1.0f);
    CHECK(committed.match.state_time_pi_us == kInitialPiTimeUs + 5000);
    CHECK(committed.match.score == rig.initial.match.score);
    CHECK(drain(rig.core).empty());  // No invented controller transactions or cues.
    CHECK(rig.core.counters().periodic_batches == 1);
    CHECK(rig.core.phase() == BatchPhase::Idle);
}

void test_catch_up_up_to_four_ticks() {
    Rig rig;
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 4, kInitialPiTimeUs + 20000);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);
    CHECK(rig.service.snapshot().simulation_tick == kInitialTick + 4);
    CHECK(rig.service.snapshot().match.state_time_pi_us == kInitialPiTimeUs + 20000);
}

void test_lag_beyond_catch_up_pauses() {
    Rig rig;
    CHECK(!rig.core.release(kInitialTick + 5).has_value());
    CHECK(rig.core.phase() == BatchPhase::Paused);
    CHECK(rig.core.pause_reason() == PauseReason::SimulationLag);
    CHECK(!rig.core.release(kInitialTick + 1).has_value());
    CHECK(rig.core.counters().paused_releases == 1);
    CHECK(rig.service.snapshot() == rig.initial);
    // Explicit recovery rebases on the committed tick instead of integrating the gap.
    CHECK(rig.core.resume(kInitialTick));
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 1, kInitialPiTimeUs + 5000);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);
}

// ---------------------------------------------------------------------------
// Ordering, batching and bounded queues

void test_eight_inputs_one_atomic_commit() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    std::uint64_t arrival = now - 4000;
    for (std::uint64_t seq = 1; seq <= 2; ++seq) {
        for (std::uint32_t slot = 1; slot <= 4; ++slot) {
            CHECK(rig.core.admit(input_for(slot, seq, now - 2000), arrival++) == AdmitResult::Queued);
        }
    }
    const auto request = rig.core.release(kInitialTick + 1);
    CHECK(request.has_value());
    CHECK(request->queries.size() == 8);
    // Order follows Pi arrival, not source time or slot rotation.
    std::size_t index = 0;
    for (std::uint64_t seq = 1; seq <= 2; ++seq) {
        for (std::uint32_t slot = 1; slot <= 4; ++slot) {
            const QueryEntry& entry = request->queries[index];
            CHECK(entry.transaction.controller_id == controller_for_slot(slot));
            CHECK(entry.transaction.request_seq == seq);
            CHECK(entry.query_id == index + 1);
            ++index;
        }
    }
    rig.core.on_query_response(rig.service.query(*request), now);
    const auto update = rig.core.prepare_commit(now);
    CHECK(update.has_value() && update->decisions.size() == 8);
    const auto result = rig.service.commit(*update);
    CHECK(result.has_value() && result->status == CommitStatus::Accepted);
    rig.core.on_commit_result(*result);

    CHECK(rig.service.accepted_commits() == 1);
    CHECK(rig.service.snapshot().state_version == 2);
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 8);
    for (std::size_t i = 0; i < feedback.size(); ++i) {
        CHECK(feedback[i].committed);
        CHECK(feedback[i].outcome == Outcome::Miss);
        CHECK(feedback[i].event_index == i);
        CHECK(feedback[i].state_version == 2);
        CHECK(feedback[i].transaction == request->queries[i].transaction);
    }
}

void test_per_controller_limit_and_queue_overflow() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    for (std::uint64_t seq = 1; seq <= 4; ++seq) {
        CHECK(rig.core.admit(input_for(1, seq, now - 2000), now - 3000 + seq) == AdmitResult::Queued);
    }
    CHECK(rig.core.admit(input_for(1, 5, now - 2000), now - 2000) == AdmitResult::QueueFull);
    CHECK(rig.core.admit(input_for(1, 4, now - 2000), now - 2000) == AdmitResult::DuplicateTransaction);
    ControllerInput stranger = input_for(2, 1, now - 2000);
    stranger.transaction.controller_id = 999;
    CHECK(rig.core.admit(stranger, now - 2000) == AdmitResult::UnknownController);

    auto feedback = drain(rig.core);
    CHECK(feedback.size() == 3);
    CHECK(feedback[0].outcome == Outcome::Overloaded && !feedback[0].committed);
    CHECK(feedback[1].outcome == Outcome::InvalidInput);
    CHECK(feedback[2].outcome == Outcome::InvalidInput);

    const auto request = rig.core.release(kInitialTick + 1);
    CHECK(request.has_value() && request->queries.size() == 2);  // At most two per controller.
    CHECK(rig.core.pending_inputs() == 2);
    rig.core.on_query_response(rig.service.query(*request), now);
    rig.core.on_commit_result(*rig.service.commit(*rig.core.prepare_commit(now)));

    const auto next = rig.core.release(kInitialTick + 2);
    CHECK(next.has_value() && next->queries.size() == 2);
    CHECK(next->queries[0].transaction.request_seq == 3);
    CHECK(rig.core.scheduler_counters().rejected_overflow == 1);
}

void test_later_event_sees_earlier_candidate() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    const Vector3 hit{0.0f, 5.0f, -10.0f};
    rig.evaluator.script_hit(controller_for_slot(1), 1, hit);
    rig.core.admit(input_for(1, 1, now - 2000), now - 1500);
    rig.core.admit(input_for(2, 1, now - 2000), now - 1000);
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 1, now);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);

    const auto& seen = rig.evaluator.observations();
    CHECK(seen.size() == 2);
    CHECK(near(seen[0].incoming_velocity_m_s.y, -0.04905, 1e-6));  // After one integrated tick.
    CHECK(seen[1].incoming_velocity_m_s == hit);                     // Sees the first hit.
    const MatchState& committed = rig.service.snapshot().match;
    CHECK(committed.ball.velocity_m_s == hit);
    CHECK(committed.rally.last_hitter_slot == 1);
    CHECK(committed.rally.contact_slot_mask == 1u);
    CHECK(committed.rally.contact_id == 1);
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 2);
    CHECK(feedback[0].outcome == Outcome::Hit && feedback[0].cue.kind == CueKind::Hit);
    CHECK(feedback[1].outcome == Outcome::Miss && feedback[1].cue.kind == CueKind::Miss);
}

void test_outgoing_speed_cap() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.evaluator.script_hit(controller_for_slot(1), 1, Vector3{0.0f, 0.0f, -40.0f});
    rig.core.admit(input_for(1, 1, now - 2000), now - 1500);
    run_batch(rig.core, rig.service, kInitialTick + 1, now);
    CHECK(near(rig.service.snapshot().match.ball.velocity_m_s.z, -20.0, 1e-6));
}

// ---------------------------------------------------------------------------
// Rejection and rollback

void test_input_invalidated_before_submission_rebuilds() {
    // Reference: the same batch where only slot 2 ever existed.
    Rig reference;
    reference.evaluator.script_hit(controller_for_slot(2), 1, Vector3{1.0f, 1.0f, 1.0f});
    reference.core.admit(input_for(2, 1, kInitialPiTimeUs + 4000), kInitialPiTimeUs + 4500);
    const auto reference_request = reference.core.release(kInitialTick + 1);
    reference.core.on_query_response(reference.service.query(*reference_request), kInitialPiTimeUs + 15000);
    const auto reference_update = reference.core.prepare_commit(kInitialPiTimeUs + 21000);

    Rig rig;
    rig.evaluator.script_hit(controller_for_slot(1), 1, Vector3{0.0f, 5.0f, -10.0f});
    rig.evaluator.script_hit(controller_for_slot(2), 1, Vector3{1.0f, 1.0f, 1.0f});
    rig.core.admit(input_for(1, 1, kInitialPiTimeUs - 10000), kInitialPiTimeUs + 4000);
    rig.core.admit(input_for(2, 1, kInitialPiTimeUs + 4000), kInitialPiTimeUs + 4500);
    const auto request = rig.core.release(kInitialTick + 1);
    // At query time both inputs are within the 30 ms IMU age gate.
    rig.core.on_query_response(rig.service.query(*request), kInitialPiTimeUs + 15000);
    CHECK(drain(rig.core).empty());
    // The wait pushes slot 1 to 31 ms; the candidate must be rebuilt without it.
    const auto update = rig.core.prepare_commit(kInitialPiTimeUs + 21000);
    CHECK(update.has_value());
    CHECK(rig.core.counters().rebuilds == 1);
    CHECK(update->decisions.size() == 1);
    CHECK(update->decisions[0].transaction.controller_id == controller_for_slot(2));
    CHECK(update->decisions[0].event_index == 0);
    CHECK(update->match == reference_update->match);
    CHECK(update->decisions[0].swing_id == reference_update->decisions[0].swing_id);

    const auto failures = drain(rig.core);
    CHECK(failures.size() == 1);
    CHECK(failures[0].outcome == Outcome::StaleInput && !failures[0].committed);
    CHECK(failures[0].transaction.controller_id == controller_for_slot(1));

    const auto result = rig.service.commit(*update);
    CHECK(result.has_value() && result->status == CommitStatus::Accepted);
    rig.core.on_commit_result(*result);
    const auto confirmed = drain(rig.core);
    CHECK(confirmed.size() == 1 && confirmed[0].outcome == Outcome::Hit && confirmed[0].committed);
}

void test_tracking_unavailable_is_not_a_miss() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    PoseObservation lost = tracked_pose(10, now - 3000, person_for_slot(3));
    lost.tracked = false;
    rig.poses.set(person_for_slot(3), lost);
    rig.core.admit(input_for(3, 1, now - 2000), now - 1000);
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 1, now);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 1);
    CHECK(feedback[0].outcome == Outcome::TrackingUnavailable && !feedback[0].committed);
    CHECK(feedback[0].cue.kind == CueKind::None);
}

void test_stale_base_version_rolls_back_everything() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.evaluator.script_hit(controller_for_slot(1), 1, Vector3{0.0f, 5.0f, -10.0f});
    rig.core.admit(input_for(1, 1, now - 2000), now - 1500);
    rig.core.admit(input_for(3, 1, now - 2000), now - 1000);
    const auto request = rig.core.release(kInitialTick + 1);
    rig.core.on_query_response(rig.service.query(*request), now);
    rig.service.bump_version_for_test();  // The queried version is now stale.
    const Snapshot before = rig.service.snapshot();
    const SideState side_before = rig.core.side_state();

    const auto result = rig.service.commit(*rig.core.prepare_commit(now));
    CHECK(result.has_value() && result->status == CommitStatus::Conflict);
    rig.core.on_commit_result(*result);

    CHECK(rig.service.snapshot() == before);       // Ball, avatars, rally, score unchanged.
    CHECK(rig.core.side_state() == side_before);   // No staged swing/cue identity installed.
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 2);
    for (const FeedbackResult& item : feedback) {
        CHECK(item.outcome == Outcome::StateConflict && !item.committed && item.cue.kind == CueKind::None);
    }
    CHECK(rig.core.phase() == BatchPhase::Idle);

    // A fresh query recovers on the current version.
    rig.core.admit(input_for(1, 2, now + 3000), now + 3500);
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 2, now + 5000);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);
    CHECK(rig.service.snapshot().state_version == before.state_version + 1);
}

void test_wrong_writer_epoch_is_fenced() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.core.admit(input_for(1, 1, now - 2000), now - 1500);
    const auto request = rig.core.release(kInitialTick + 1);
    rig.core.on_query_response(rig.service.query(*request), now);
    const auto update = rig.core.prepare_commit(now);
    rig.service.restart_writer(2);  // A replacement physics process took over.

    const auto result = rig.service.commit(*update);
    CHECK(result.has_value() && result->status == CommitStatus::WrongWriter);
    rig.core.on_commit_result(*result);
    CHECK(rig.service.snapshot() == rig.initial);
    CHECK(rig.core.phase() == BatchPhase::Paused);
    CHECK(rig.core.pause_reason() == PauseReason::WrongWriter);
    CHECK(!rig.core.resume(kInitialTick));  // An old epoch can never write again.
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 1 && feedback[0].outcome == Outcome::StateConflict);

    // The replacement writer resumes after reconciliation with a new epoch.
    rig.service.resume_writer();
    BatchCore replacement{build_setup('G', 2), rig.service.snapshot().simulation_tick, rig.poses, rig.evaluator};
    const BatchRun run = run_batch(replacement, rig.service, kInitialTick + 1, now);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);
    CHECK(run.result->writer_epoch == 2);
}

void test_wrong_game_session_is_rejected() {
    Rig rig;
    BatchCore stranger{build_setup('H'), kInitialTick, rig.poses, rig.evaluator};
    const auto request = stranger.release(kInitialTick + 1);
    CHECK(request.has_value());
    stranger.on_query_response(rig.service.query(*request), kInitialPiTimeUs + 5000);
    CHECK(stranger.phase() == BatchPhase::Paused);
    CHECK(stranger.pause_reason() == PauseReason::QueryRejected);
    CHECK(!stranger.prepare_commit(kInitialPiTimeUs + 5000).has_value());

    // A hand-built update under the wrong session is rejected without effects.
    StateUpdate forged;
    forged.game_session_id = filled_uuid('H');
    forged.writer_epoch = 1;
    forged.batch_id = 1;
    const auto result = rig.service.commit(forged);
    CHECK(result.has_value() && result->status == CommitStatus::WrongSession);
    CHECK(rig.service.snapshot() == rig.initial);
}

// ---------------------------------------------------------------------------
// Retries and ambiguity

void test_identical_retry_returns_retained_outcome() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.evaluator.script_hit(controller_for_slot(1), 1, Vector3{0.0f, 5.0f, -10.0f});
    rig.core.admit(input_for(1, 1, now - 2000), now - 1500);
    rig.service.drop_next_commit_replies(1);  // Commit applies, reply is lost.
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 1, now);
    CHECK(run.result.has_value() && run.result->status == CommitStatus::Accepted);
    CHECK(rig.core.counters().commit_resends == 1);
    CHECK(rig.service.accepted_commits() == 1);  // The resend did not advance twice.
    CHECK(rig.service.snapshot().state_version == 2);
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 1 && feedback[0].committed && feedback[0].outcome == Outcome::Hit);

    // A late duplicate reply is ignored instead of confirming the hit again.
    rig.core.on_commit_result(*run.result);
    CHECK(drain(rig.core).empty());
    CHECK(rig.core.counters().unmatched_replies == 1);
}

void test_changed_payload_under_same_identity_is_rejected() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    const auto request = rig.core.release(kInitialTick + 1);
    rig.core.on_query_response(rig.service.query(*request), now);
    const auto update = rig.core.prepare_commit(now);
    const auto accepted = rig.service.commit(*update);
    CHECK(accepted.has_value() && accepted->status == CommitStatus::Accepted);
    const Snapshot after_accept = rig.service.snapshot();

    StateUpdate altered = *update;
    altered.match.ball.position_m.x += 1.0f;
    const auto rejected = rig.service.commit(altered);
    CHECK(rejected.has_value() && rejected->status == CommitStatus::Invalid);
    CHECK(rig.service.snapshot() == after_accept);
    // The identical retry still returns the retained original outcome.
    const auto retry = rig.service.commit(*update);
    CHECK(retry.has_value() && *retry == *accepted);

    // The query side behaves the same way.
    BallStateRequest altered_query = *request;
    altered_query.batch_id = request->batch_id;
    QueryEntry extra;
    extra.query_id = 99;
    extra.transaction = input_for(1, 7, now).transaction;
    altered_query.queries.push_back(extra);
    CHECK(rig.service.query(altered_query).status == QueryStatus::Rejected);
}

void test_ambiguous_commit_pauses_until_resolved() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.evaluator.script_hit(controller_for_slot(2), 1, Vector3{0.0f, 4.0f, 9.0f});
    rig.core.admit(input_for(2, 1, now - 2000), now - 1500);
    rig.service.drop_next_commit_replies(3);  // Original and both resends are lost.
    const BatchRun run = run_batch(rig.core, rig.service, kInitialTick + 1, now);
    CHECK(run.released && !run.result.has_value());
    CHECK(rig.core.phase() == BatchPhase::Paused);
    CHECK(rig.core.pause_reason() == PauseReason::CommitAmbiguous);
    CHECK(rig.core.counters().commit_resends == 2);
    CHECK(drain(rig.core).empty());              // No haptic confirmation while unresolved.
    CHECK(rig.core.side_state() == SideState{});  // Staged state is retained, not installed.
    CHECK(!rig.core.release(kInitialTick + 2).has_value());
    CHECK(!rig.core.resume(kInitialTick));

    // Control-plane lookup resolves the original identity.
    const StateCommitResult status = rig.service.lookup(1, 1);
    CHECK(status.status == CommitStatus::Accepted);
    rig.core.on_commit_result(status);
    CHECK(rig.core.phase() == BatchPhase::Idle);
    CHECK(rig.core.side_state().next_swing_id == 2);
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 1 && feedback[0].committed && feedback[0].outcome == Outcome::Hit);
    CHECK(rig.service.accepted_commits() == 1);
}

void test_unknown_outcome_after_retention_pauses() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.core.admit(input_for(4, 1, now - 2000), now - 1500);
    const auto request = rig.core.release(kInitialTick + 1);
    rig.core.on_query_response(rig.service.query(*request), now);
    rig.core.prepare_commit(now);
    // No reply ever arrives: two identical resends, then the writer pauses.
    CHECK(rig.core.on_commit_timeout().has_value());
    CHECK(rig.core.on_commit_timeout().has_value());
    CHECK(!rig.core.on_commit_timeout().has_value());
    // The lookup cannot find the batch (it never reached the service).
    const StateCommitResult status = rig.service.lookup(1, request->batch_id);
    CHECK(status.status == CommitStatus::UnknownBatch);
    rig.core.on_commit_result(status);
    CHECK(rig.core.pause_reason() == PauseReason::CommitUnknown);
    const auto feedback = drain(rig.core);
    CHECK(feedback.size() == 1 && feedback[0].outcome == Outcome::Timeout && !feedback[0].committed);
    CHECK(rig.service.snapshot() == rig.initial);  // The snapshot alone proves nothing happened.
}

// ---------------------------------------------------------------------------
// Scheduling and trace identities

void test_one_batch_in_flight() {
    Rig rig;
    const auto request = rig.core.release(kInitialTick + 1);
    CHECK(request.has_value());
    CHECK(!rig.core.release(kInitialTick + 1).has_value());
    CHECK(!rig.core.release(kInitialTick + 2).has_value());
    CHECK(rig.core.counters().skipped_releases == 2);
    CHECK(!rig.core.release(kInitialTick).has_value());
    // Stray replies for another batch are not applied.
    BallStateResponse stray = rig.service.query(*request);
    stray.batch_id = 77;
    rig.core.on_query_response(stray, kInitialPiTimeUs + 5000);
    CHECK(rig.core.phase() == BatchPhase::QueryPending);
    CHECK(rig.core.counters().unmatched_replies == 1);
}

void test_already_current_tick_does_not_release() {
    Rig rig;
    CHECK(!rig.core.release(kInitialTick).has_value());
    CHECK(rig.core.counters().idle_releases == 1);
    CHECK(rig.core.phase() == BatchPhase::Idle);
}

void test_trace_preserves_identities() {
    Rig rig;
    const std::uint64_t now = kInitialPiTimeUs + 5000;
    rig.core.admit(input_for(1, 1, now - 2000), now - 1500);
    rig.core.admit(input_for(2, 1, now - 2000), now - 1000);
    run_batch(rig.core, rig.service, kInitialTick + 1, now);
    const auto trace = rig.core.take_trace();
    std::vector<TraceRecord> released;
    std::vector<TraceRecord> staged;
    std::vector<TraceRecord> resolved;
    for (const TraceRecord& record : trace) {
        CHECK(record.writer_epoch == 1 && record.batch_id == 1);
        if (record.kind == TraceKind::QueryReleased) released.push_back(record);
        if (record.kind == TraceKind::DecisionStaged) staged.push_back(record);
        if (record.kind == TraceKind::CommitResolved) resolved.push_back(record);
    }
    CHECK(released.size() == 2 && staged.size() == 2 && resolved.size() == 1);
    for (std::size_t i = 0; i < staged.size() && i < released.size(); ++i) {
        CHECK(staged[i].query_id == released[i].query_id);
        CHECK(staged[i].transaction == released[i].transaction);
        CHECK(staged[i].event_index == i);
    }
    CHECK(resolved[0].commit_status == CommitStatus::Accepted);
    CHECK(resolved[0].state_version == 2 && resolved[0].simulation_tick == kInitialTick + 1);
    CHECK(rig.core.counters().trace_lost == 0);
}

struct TestCase {
    const char* name;
    void (*run)();
};

const TestCase kTests[] = {
    {"free_fall_known_answer", test_free_fall_known_answer},
    {"ground_bounce_restitution", test_ground_bounce_restitution},
    {"avatar_bounded_step", test_avatar_bounded_step},
    {"replay_is_bit_identical", test_replay_is_bit_identical},
    {"periodic_only_batch", test_periodic_only_batch},
    {"catch_up_up_to_four_ticks", test_catch_up_up_to_four_ticks},
    {"lag_beyond_catch_up_pauses", test_lag_beyond_catch_up_pauses},
    {"eight_inputs_one_atomic_commit", test_eight_inputs_one_atomic_commit},
    {"per_controller_limit_and_queue_overflow", test_per_controller_limit_and_queue_overflow},
    {"later_event_sees_earlier_candidate", test_later_event_sees_earlier_candidate},
    {"outgoing_speed_cap", test_outgoing_speed_cap},
    {"input_invalidated_before_submission_rebuilds", test_input_invalidated_before_submission_rebuilds},
    {"tracking_unavailable_is_not_a_miss", test_tracking_unavailable_is_not_a_miss},
    {"stale_base_version_rolls_back_everything", test_stale_base_version_rolls_back_everything},
    {"wrong_writer_epoch_is_fenced", test_wrong_writer_epoch_is_fenced},
    {"wrong_game_session_is_rejected", test_wrong_game_session_is_rejected},
    {"identical_retry_returns_retained_outcome", test_identical_retry_returns_retained_outcome},
    {"changed_payload_under_same_identity_is_rejected", test_changed_payload_under_same_identity_is_rejected},
    {"ambiguous_commit_pauses_until_resolved", test_ambiguous_commit_pauses_until_resolved},
    {"unknown_outcome_after_retention_pauses", test_unknown_outcome_after_retention_pauses},
    {"one_batch_in_flight", test_one_batch_in_flight},
    {"already_current_tick_does_not_release", test_already_current_tick_does_not_release},
    {"trace_preserves_identities", test_trace_preserves_identities},
};

}  // namespace

int main() {
    int failed_tests = 0;
    for (const TestCase& test : kTests) {
        const int before = g_failures;
        test.run();
        const bool passed = g_failures == before;
        failed_tests += passed ? 0 : 1;
        std::printf("[%s] %s\n", passed ? "PASS" : "FAIL", test.name);
    }
    const int total = static_cast<int>(sizeof(kTests) / sizeof(kTests[0]));
    std::printf("%d/%d S1 batch-core checks passed\n", total - failed_tests, total);
    return failed_tests == 0 ? 0 : 1;
}
