#include "motion_tennis/physics/batch_core.hpp"
#include "motion_tennis/physics/tracking_pose_bridge.hpp"

namespace motion_tennis::physics {
namespace {

Outcome pose_failure(BridgeCode code) {
    switch (code) {
        case BridgeCode::StalePose:
            return Outcome::StaleInput;
        case BridgeCode::RequiredJointsUnavailable:
        case BridgeCode::MissingMap:
        case BridgeCode::InvalidMap:
        case BridgeCode::MapExpired:
        case BridgeCode::MapUncertain:
            return Outcome::TrackingUnavailable;
        default:
            return Outcome::InvalidInput;
    }
}

void retain_pose_trace(TraceRecord& record, const PoseObservation& pose, std::uint64_t now_pi_us) {
    record.observed_pi_us = now_pi_us;
    record.tracking_session_id = pose.envelope.sender_session_id;
    record.pi_reference_session_id = pose.mapping.reference.session_id;
    record.pi_reference_clock_id = pose.mapping.reference.clock_id;
    record.person_id = pose.source_frame.person_id;
    record.calibration_id = pose.calibration_id;
    record.pose_raw_capture = pose.source_frame.capture;
    record.pose_capture_pi_us = pose.capture_pi_us;
    record.pose_mapping_id = pose.mapping.mapping_id;
    record.pose_mapping_version = pose.mapping.version;
    record.pose_total_uncertainty_us = pose.total_uncertainty_us;
}

}  // namespace

BatchCore::BatchCore(const MatchSetup& setup, std::uint64_t committed_tick, const PoseSource& poses,
                     const ContactEvaluator& evaluator, SimulationParameters parameters)
    : setup_(setup),
      poses_(poses),
      evaluator_(evaluator),
      parameters_(parameters),
      scheduler_(setup.configuration),
      known_committed_tick_(committed_tick) {}

const PlayerBinding& BatchCore::binding_for_slot(std::uint32_t slot) const {
    return setup_.configuration.players[slot - 1];
}

void BatchCore::emit_feedback(const FeedbackResult& feedback) {
    if (!feedback_.push_back(feedback)) {
        ++counters_.feedback_lost;
    }
}

void BatchCore::emit_trace(const TraceRecord& record) {
    if (!trace_.push_back(record)) {
        ++counters_.trace_lost;
    }
}

BoundedVector<FeedbackResult, kFeedbackCapacity> BatchCore::take_feedback() {
    BoundedVector<FeedbackResult, kFeedbackCapacity> drained = feedback_;
    feedback_.clear();
    return drained;
}

BoundedVector<TraceRecord, kTraceCapacity> BatchCore::take_trace() {
    BoundedVector<TraceRecord, kTraceCapacity> drained = trace_;
    trace_.clear();
    return drained;
}

FeedbackResult BatchCore::feedback_for(const Transaction& transaction) const {
    FeedbackResult feedback;
    feedback.transaction = transaction;
    feedback.game_session_id = setup_.game_session_id;
    feedback.writer_epoch = setup_.writer_epoch;
    feedback.batch_id = in_flight_.request.batch_id;
    return feedback;
}

void BatchCore::pause(PauseReason reason) {
    phase_ = BatchPhase::Paused;
    pause_reason_ = reason;
}

AdmitResult BatchCore::admit(const ControllerInput& input, std::uint64_t arrival_pi_us) {
    const AdmitResult result = scheduler_.admit(input, arrival_pi_us);
    if (result != AdmitResult::Queued) {
        FeedbackResult feedback;
        feedback.transaction = input.transaction;
        feedback.game_session_id = setup_.game_session_id;
        feedback.writer_epoch = setup_.writer_epoch;
        feedback.outcome = result == AdmitResult::QueueFull ? Outcome::Overloaded : Outcome::InvalidInput;
        feedback.reason = result == AdmitResult::QueueFull ? "controller queue full"
                          : result == AdmitResult::DuplicateTransaction ? "duplicate pending transaction"
                                                                        : "controller absent from roster";
        ++counters_.input_failures;
        emit_feedback(feedback);
    }
    return result;
}

std::optional<BallStateRequest> BatchCore::release(std::uint64_t desired_tick) {
    if (phase_ == BatchPhase::Paused) {
        ++counters_.paused_releases;
        return std::nullopt;
    }
    if (phase_ != BatchPhase::Idle) {
        // Only one batch is in flight; a release while busy never creates a second writer.
        ++counters_.skipped_releases;
        return std::nullopt;
    }
    if (desired_tick <= known_committed_tick_) {
        ++counters_.idle_releases;
        return std::nullopt;
    }
    if (desired_tick - known_committed_tick_ > parameters_.max_catchup_ticks) {
        // Do not integrate an unbounded jump to hide missed work.
        pause(PauseReason::SimulationLag);
        return std::nullopt;
    }

    in_flight_ = InFlight{};
    in_flight_.inputs = scheduler_.select_batch();
    in_flight_.target_tick = desired_tick;
    BallStateRequest& request = in_flight_.request;
    request.game_session_id = setup_.game_session_id;
    request.writer_epoch = setup_.writer_epoch;
    request.batch_id = next_batch_id_++;
    for (const PendingInput& item : in_flight_.inputs) {
        QueryEntry entry;
        entry.query_id = next_query_id_++;
        entry.transaction = item.input.transaction;
        request.queries.push_back(entry);

        TraceRecord record;
        record.kind = TraceKind::QueryReleased;
        record.writer_epoch = request.writer_epoch;
        record.batch_id = request.batch_id;
        record.query_id = entry.query_id;
        record.transaction = entry.transaction;
        emit_trace(record);
    }
    if (request.queries.empty()) {
        TraceRecord record;
        record.kind = TraceKind::QueryReleased;
        record.writer_epoch = request.writer_epoch;
        record.batch_id = request.batch_id;
        emit_trace(record);
        ++counters_.periodic_batches;
    }
    ++counters_.batches_released;
    phase_ = BatchPhase::QueryPending;
    return request;
}

void BatchCore::fail_batch(Outcome outcome, PauseReason reason, std::string_view why) {
    for (std::size_t index = 0; index < in_flight_.inputs.size(); ++index) {
        if (in_flight_.failures[index] != Outcome::Unspecified) {
            continue;  // Already reported with its own failure.
        }
        FeedbackResult feedback = feedback_for(in_flight_.inputs[index].input.transaction);
        feedback.outcome = outcome;
        feedback.reason = why;
        emit_feedback(feedback);
    }
    ++counters_.batches_failed;
    in_flight_ = InFlight{};
    pause(reason);
}

void BatchCore::on_query_response(const BallStateResponse& response, std::uint64_t now_pi_us) {
    const BallStateRequest& request = in_flight_.request;
    if (phase_ != BatchPhase::QueryPending || response.batch_id != request.batch_id ||
        response.writer_epoch != request.writer_epoch ||
        response.game_session_id != request.game_session_id) {
        ++counters_.unmatched_replies;
        return;
    }
    if (!(response.queries == request.queries)) {
        fail_batch(Outcome::StateConflict, PauseReason::QueryRejected, "response does not echo queries");
        return;
    }
    if (response.status != QueryStatus::Ok || !response.has_snapshot) {
        fail_batch(Outcome::StateConflict, PauseReason::QueryRejected, "ball state query rejected");
        return;
    }
    const Snapshot& snapshot = response.snapshot;
    if (snapshot.game_session_id != setup_.game_session_id ||
        snapshot.tick_duration_us != parameters_.tick_duration_us ||
        snapshot.match.configuration_sha256 != setup_.configuration_sha256 ||
        snapshot.match.roster_revision != setup_.configuration.roster_revision ||
        snapshot.match.rules_revision != setup_.configuration.rules_revision) {
        fail_batch(Outcome::StateConflict, PauseReason::QueryRejected, "snapshot identity mismatch");
        return;
    }
    if (snapshot.simulation_tick >= in_flight_.target_tick ||
        in_flight_.target_tick - snapshot.simulation_tick > parameters_.max_catchup_ticks) {
        fail_batch(Outcome::StateConflict, PauseReason::SimulationLag,
                   "snapshot tick outside catch-up window");
        return;
    }

    in_flight_.snapshot = snapshot;
    known_committed_tick_ = snapshot.simulation_tick;
    phase_ = BatchPhase::Compute;
    recheck_inputs(now_pi_us, true);
    build_candidate();
}

Outcome BatchCore::check_input(std::size_t index, std::uint64_t now_pi_us, bool capture_pose) {
    const PendingInput& item = in_flight_.inputs[index];
    const ControllerInput& input = item.input;
    const PlayerBinding& binding = binding_for_slot(item.player_slot);
    if (input.clock_mapping_version == 0 || input.association_version != binding.association_version ||
        input.calibration_id != binding.calibration_id) {
        return Outcome::InvalidInput;
    }
    if (input.capture_pi_us > now_pi_us) {
        return Outcome::InvalidInput;
    }
    if (now_pi_us - input.capture_pi_us > parameters_.max_imu_age_us) {
        return Outcome::StaleInput;
    }
    const std::optional<PoseObservation> current = poses_.latest_for(binding.person_id);
    if (!current) {
        return Outcome::TrackingUnavailable;
    }
    if (capture_pose) {
        // Each decision uses one immutable pose snapshot; rebuilds reuse it.
        in_flight_.poses[index] = *current;
    }
    const PiClockReference reference{in_flight_.snapshot.match.pi_reference_session_id,
                                     in_flight_.snapshot.match.pi_reference_clock_id};
    const BridgeCode current_code = validate_tracking_pose(*current, binding, reference, now_pi_us,
                                                           parameters_.max_pose_age_us);
    if (current_code != BridgeCode::Accepted) {
        return pose_failure(current_code);
    }
    const PoseObservation& pose = in_flight_.poses[index];
    if (pose.envelope.sender_id != current->envelope.sender_id ||
        pose.envelope.sender_session_id != current->envelope.sender_session_id ||
        pose.envelope.run_id != current->envelope.run_id || pose.envelope.clock_id != current->envelope.clock_id ||
        pose.mapping.reference != current->mapping.reference) {
        return Outcome::InvalidInput;
    }
    const BridgeCode code = validate_tracking_pose(pose, binding, reference, now_pi_us,
                                                   parameters_.max_pose_age_us);
    if (code != BridgeCode::Accepted) {
        return pose_failure(code);
    }
    return Outcome::Unspecified;
}

bool BatchCore::recheck_inputs(std::uint64_t now_pi_us, bool capture_pose) {
    bool newly_failed = false;
    for (std::size_t index = 0; index < in_flight_.inputs.size(); ++index) {
        if (in_flight_.failures[index] != Outcome::Unspecified) {
            continue;
        }
        const Outcome failure = check_input(index, now_pi_us, capture_pose);
        if (failure == Outcome::Unspecified) {
            continue;
        }
        in_flight_.failures[index] = failure;
        newly_failed = true;
        ++counters_.input_failures;

        FeedbackResult feedback = feedback_for(in_flight_.inputs[index].input.transaction);
        feedback.outcome = failure;
        feedback.reason = "input failed eligibility check";
        emit_feedback(feedback);

        TraceRecord record;
        record.kind = TraceKind::InputFailed;
        record.writer_epoch = setup_.writer_epoch;
        record.batch_id = in_flight_.request.batch_id;
        record.query_id = in_flight_.request.queries[index].query_id;
        record.transaction = in_flight_.inputs[index].input.transaction;
        record.outcome = failure;
        retain_pose_trace(record, in_flight_.poses[index], now_pi_us);
        emit_trace(record);
    }
    return newly_failed;
}

void BatchCore::build_candidate() {
    InFlight& flight = in_flight_;
    // Always restart from the queried snapshot and the installed side-state so a
    // rebuild never inherits effects of a removed input.
    MatchState match = flight.snapshot.match;
    SideState side = side_state_;
    advance_match(match, static_cast<std::uint32_t>(flight.target_tick - flight.snapshot.simulation_tick),
                  parameters_);

    StateUpdate update;
    update.game_session_id = setup_.game_session_id;
    update.writer_epoch = setup_.writer_epoch;
    update.batch_id = flight.request.batch_id;
    update.base_state_version = flight.snapshot.state_version;
    update.target_simulation_tick = flight.target_tick;
    update.tracking_session_id = setup_.configuration.players[0].tracking_session_id;

    for (std::size_t index = 0; index < flight.inputs.size(); ++index) {
        if (flight.failures[index] != Outcome::Unspecified) {
            continue;
        }
        const PendingInput& item = flight.inputs[index];
        const PlayerBinding& binding = binding_for_slot(item.player_slot);
        const PoseObservation& pose = flight.poses[index];
        // Later events see the candidate produced by earlier ones.
        const ContactResult contact = evaluator_.evaluate(match, item, pose, binding);

        Decision decision;
        decision.transaction = item.input.transaction;
        decision.current_sample_id = item.input.sample_id;
        decision.pose_frame_id = pose.frame_id;
        decision.person_id = binding.person_id;
        decision.association_version = item.input.association_version;
        decision.imu_capture_us = item.input.capture_pi_us;
        decision.pose_capture_us = pose.source_frame.capture.value_us;
        decision.event_index = static_cast<std::uint32_t>(update.decisions.size());
        decision.swing_id = side.next_swing_id++;
        if (contact.outcome == Outcome::Hit) {
            decision.outcome = Outcome::Hit;
            match.ball.velocity_m_s = cap_speed(contact.outgoing_velocity_m_s,
                                                parameters_.max_outgoing_speed_m_s);
            match.rally.last_hitter_slot = item.player_slot;
            match.rally.contact_slot_mask |= 1u << (item.player_slot - 1);
            ++match.rally.contact_id;
            decision.cue = Cue{CueKind::Hit, side.next_cue_id++, 1.0f, 40, true};
        } else {
            decision.outcome = Outcome::Miss;
            decision.cue = Cue{CueKind::Miss, side.next_cue_id++, 0.5f, 20, true};
        }
        update.decisions.push_back(decision);
    }
    update.match = match;
    flight.update = update;
    flight.staged_side = side;
}

std::optional<StateUpdate> BatchCore::prepare_commit(std::uint64_t now_pi_us) {
    if (phase_ != BatchPhase::Compute) {
        return std::nullopt;
    }
    if (recheck_inputs(now_pi_us, false)) {
        ++counters_.rebuilds;
        build_candidate();
    }

    const StateUpdate& update = in_flight_.update;
    std::size_t decision_index = 0;
    for (std::size_t index = 0; index < in_flight_.inputs.size(); ++index) {
        if (in_flight_.failures[index] != Outcome::Unspecified) {
            continue;
        }
        const Decision& decision = update.decisions[decision_index++];
        TraceRecord record;
        record.kind = TraceKind::DecisionStaged;
        record.writer_epoch = update.writer_epoch;
        record.batch_id = update.batch_id;
        record.query_id = in_flight_.request.queries[index].query_id;
        record.transaction = decision.transaction;
        record.event_index = decision.event_index;
        record.outcome = decision.outcome;
        record.state_version = update.base_state_version;
        record.simulation_tick = update.target_simulation_tick;
        retain_pose_trace(record, in_flight_.poses[index], now_pi_us);
        emit_trace(record);
    }
    TraceRecord submitted;
    submitted.kind = TraceKind::CommitSubmitted;
    submitted.writer_epoch = update.writer_epoch;
    submitted.batch_id = update.batch_id;
    submitted.state_version = update.base_state_version;
    submitted.simulation_tick = update.target_simulation_tick;
    emit_trace(submitted);

    phase_ = BatchPhase::CommitPending;
    return update;
}

std::optional<StateUpdate> BatchCore::on_commit_timeout() {
    if (phase_ != BatchPhase::CommitPending) {
        return std::nullopt;
    }
    if (in_flight_.resends < kMaxCommitResends) {
        ++in_flight_.resends;
        ++counters_.commit_resends;
        TraceRecord record;
        record.kind = TraceKind::CommitResent;
        record.writer_epoch = in_flight_.update.writer_epoch;
        record.batch_id = in_flight_.update.batch_id;
        emit_trace(record);
        return in_flight_.update;  // Identical bytes: a retry can never advance twice.
    }
    // Keep the staged side-state until the original identity is resolved.
    pause(PauseReason::CommitAmbiguous);
    return std::nullopt;
}

void BatchCore::on_commit_result(const StateCommitResult& result) {
    const StateUpdate& update = in_flight_.update;
    const bool awaiting = phase_ == BatchPhase::CommitPending ||
                          (phase_ == BatchPhase::Paused && pause_reason_ == PauseReason::CommitAmbiguous);
    if (!awaiting || result.batch_id != update.batch_id || result.writer_epoch != update.writer_epoch ||
        result.game_session_id != update.game_session_id) {
        ++counters_.unmatched_replies;
        return;
    }

    TraceRecord resolved;
    resolved.kind = TraceKind::CommitResolved;
    resolved.writer_epoch = result.writer_epoch;
    resolved.batch_id = result.batch_id;
    resolved.commit_status = result.status;
    resolved.state_version = result.state_version;
    resolved.simulation_tick = result.simulation_tick;
    emit_trace(resolved);

    const bool accepted = result.status == CommitStatus::Accepted;
    for (const Decision& decision : update.decisions) {
        FeedbackResult feedback = feedback_for(decision.transaction);
        feedback.event_index = decision.event_index;
        if (accepted) {
            // Gameplay feedback is confirmed only after the accepted commit.
            feedback.outcome = decision.outcome;
            feedback.committed = true;
            feedback.state_version = result.state_version;
            feedback.simulation_tick = result.simulation_tick;
            feedback.cue = decision.cue;
        } else {
            feedback.outcome = result.status == CommitStatus::UnknownBatch ? Outcome::Timeout
                                                                           : Outcome::StateConflict;
            feedback.reason = result.reason;
        }
        emit_feedback(feedback);
    }

    switch (result.status) {
        case CommitStatus::Accepted:
            side_state_ = in_flight_.staged_side;
            known_committed_tick_ = result.simulation_tick;
            ++counters_.batches_accepted;
            phase_ = BatchPhase::Idle;
            pause_reason_ = PauseReason::None;
            break;
        case CommitStatus::Conflict:
            // Nothing staged is installed; a later batch queries anew.
            ++counters_.batches_conflicted;
            phase_ = BatchPhase::Idle;
            pause_reason_ = PauseReason::None;
            break;
        case CommitStatus::UnknownBatch:
            ++counters_.batches_failed;
            pause(PauseReason::CommitUnknown);
            break;
        case CommitStatus::WrongWriter:
            ++counters_.batches_failed;
            pause(PauseReason::WrongWriter);
            break;
        default:
            ++counters_.batches_failed;
            pause(PauseReason::CommitRejected);
            break;
    }
    in_flight_ = InFlight{};
}

bool BatchCore::resume(std::uint64_t committed_tick) {
    if (phase_ != BatchPhase::Paused || pause_reason_ == PauseReason::WrongWriter ||
        pause_reason_ == PauseReason::CommitAmbiguous) {
        return false;
    }
    in_flight_ = InFlight{};
    known_committed_tick_ = committed_tick;
    phase_ = BatchPhase::Idle;
    pause_reason_ = PauseReason::None;
    return true;
}

BatchRun run_batch(BatchCore& core, StateService& service, std::uint64_t desired_tick,
                   std::uint64_t now_pi_us) {
    BatchRun run;
    const std::optional<BallStateRequest> request = core.release(desired_tick);
    if (!request) {
        return run;
    }
    run.released = true;
    core.on_query_response(service.query(*request), now_pi_us);
    std::optional<StateUpdate> update = core.prepare_commit(now_pi_us);
    if (!update) {
        return run;
    }
    std::optional<StateCommitResult> result = service.commit(*update);
    while (!result) {
        update = core.on_commit_timeout();
        if (!update) {
            return run;
        }
        result = service.commit(*update);
    }
    core.on_commit_result(*result);
    run.result = result;
    return run;
}

}  // namespace motion_tennis::physics
