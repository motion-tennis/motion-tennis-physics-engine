#include "motion_tennis/physics/fakes.hpp"

#include <cmath>

namespace motion_tennis::physics::fakes {
namespace {

bool finite(const Vector3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool is_award(RuleEventKind kind) {
    return kind == RuleEventKind::PointAwarded || kind == RuleEventKind::GameAwarded ||
           kind == RuleEventKind::SetAwarded || kind == RuleEventKind::MatchAwarded;
}

}  // namespace

FakeStateService::FakeStateService(const MatchSetup& setup, const Snapshot& initial)
    : setup_(setup), snapshot_(initial), writer_epoch_(setup.writer_epoch) {}

StateCommitResult FakeStateService::current(const StateUpdate& update, CommitStatus status,
                                             std::string_view reason) const {
    // Echo the batch identity so the writer can correlate every outcome.
    StateCommitResult result;
    result.game_session_id = update.game_session_id;
    result.writer_epoch = update.writer_epoch;
    result.batch_id = update.batch_id;
    result.status = status;
    result.state_version = snapshot_.state_version;
    result.simulation_tick = snapshot_.simulation_tick;
    result.reason = reason;
    return result;
}

BallStateResponse FakeStateService::query(const BallStateRequest& request) {
    BallStateResponse response;
    response.batch_id = request.batch_id;
    response.queries = request.queries;
    response.writer_epoch = request.writer_epoch;
    response.game_session_id = request.game_session_id;
    const auto reject = [&response](std::string_view reason) {
        response.status = QueryStatus::Rejected;
        response.reason = reason;
        return response;
    };

    if (request.game_session_id != setup_.game_session_id) {
        return reject("wrong game session");
    }
    if (paused_ || request.writer_epoch != writer_epoch_) {
        return reject("writer is paused or fenced");
    }
    const Key key{request.writer_epoch, request.batch_id};
    if (const auto found = queries_.find(key); found != queries_.end()) {
        if (found->second.request == request) {
            return found->second.response;
        }
        return reject("query identity reused with changed payload");
    }
    if (queries_.size() >= kRetainedBatches) {
        return reject("query retention is full");
    }
    std::map<std::uint32_t, std::uint32_t> per_controller;
    for (std::size_t index = 0; index < request.queries.size(); ++index) {
        const QueryEntry& entry = request.queries[index];
        bool rostered = false;
        for (const PlayerBinding& binding : setup_.configuration.players) {
            rostered = rostered || (binding.controller_id == entry.transaction.controller_id &&
                                    binding.controller_session_id == entry.transaction.session_id);
        }
        if (!rostered || entry.query_id == 0 || entry.transaction.request_seq == 0 ||
            ++per_controller[entry.transaction.controller_id] > kMaxQueriesPerController) {
            return reject("invalid query entries");
        }
        for (std::size_t other = 0; other < index; ++other) {
            if (request.queries[other].query_id == entry.query_id ||
                request.queries[other].transaction == entry.transaction) {
                return reject("duplicate query identity");
            }
        }
    }

    response.status = QueryStatus::Ok;
    response.has_snapshot = true;
    response.snapshot = snapshot_;
    queries_[key] = RetainedQuery{request, response};
    return response;
}

std::string_view FakeStateService::validate(const StateUpdate& update, const RetainedQuery& query) const {
    if (update.base_state_version != query.response.snapshot.state_version) {
        return "candidate does not use its returned query version";
    }
    const MatchState& match = update.match;
    if (match.configuration_sha256 != setup_.configuration_sha256 ||
        match.roster_revision != setup_.configuration.roster_revision ||
        match.rules_revision != setup_.configuration.rules_revision) {
        return "state configuration/revision mismatch";
    }
    if (!finite(match.ball.position_m) || !finite(match.ball.velocity_m_s) || !(match.ball.radius_m > 0.0f)) {
        return "invalid ball state";
    }
    for (std::size_t index = 0; index < match.avatars.size(); ++index) {
        const AvatarState& avatar = match.avatars[index];
        if (avatar.player_slot != index + 1 || !finite(avatar.position_m) || !finite(avatar.velocity_m_s) ||
            !finite(avatar.target_m) || avatar.movement == MovementMode::Unspecified) {
            return "invalid avatar state";
        }
    }
    if (update.tracking_session_id != setup_.configuration.players[0].tracking_session_id) {
        return "tracking session mismatch";
    }

    // Decisions must preserve the surviving source query order exactly.
    std::size_t source = 0;
    for (std::size_t index = 0; index < update.decisions.size(); ++index) {
        const Decision& decision = update.decisions[index];
        while (source < query.request.queries.size() &&
               !(query.request.queries[source].transaction == decision.transaction)) {
            ++source;
        }
        if (source == query.request.queries.size()) {
            return "decisions must preserve surviving source query order";
        }
        ++source;
        if (decision.event_index != index) {
            return "decision indices must be contiguous from zero";
        }
        if (decision.outcome != Outcome::Hit && decision.outcome != Outcome::Miss) {
            return "only hit/miss may commit";
        }
        const PlayerBinding* binding = nullptr;
        for (const PlayerBinding& candidate : setup_.configuration.players) {
            if (candidate.controller_id == decision.transaction.controller_id) {
                binding = &candidate;
            }
        }
        if (binding == nullptr || decision.person_id != binding->person_id ||
            decision.association_version != binding->association_version || decision.current_sample_id == 0 ||
            decision.pose_frame_id == 0) {
            return "decision tracking/association provenance mismatch";
        }
        const Cue& cue = decision.cue;
        if (!(cue.intensity >= 0.0f && cue.intensity <= 1.0f)) {
            return "cue intensity out of range";
        }
        if (cue.electrical_eligible) {
            const CueKind expected = decision.outcome == Outcome::Hit ? CueKind::Hit : CueKind::Miss;
            if (cue.cue_id == 0 || cue.duration_ms == 0 || cue.kind != expected) {
                return "cue disagrees with outcome";
            }
        } else if (cue.kind != CueKind::None) {
            return "ineligible cue must be none";
        }
    }

    std::uint64_t previous_tick = snapshot_.simulation_tick;
    for (std::size_t index = 0; index < update.rule_events.size(); ++index) {
        const RuleEvent& event = update.rule_events[index];
        if (event.event_index != kRuleEventIndexBase + index || event.kind == RuleEventKind::Unspecified ||
            event.simulation_tick < previous_tick || event.simulation_tick > update.target_simulation_tick) {
            return "invalid rule event order";
        }
        previous_tick = event.simulation_tick;
    }
    return {};
}

std::optional<StateCommitResult> FakeStateService::commit(const StateUpdate& update) {
    if (update.game_session_id != setup_.game_session_id) {
        return current(update, CommitStatus::WrongSession, "wrong game session");
    }
    if (update.writer_epoch != writer_epoch_) {
        return current(update, CommitStatus::WrongWriter, "writer epoch is fenced");
    }
    const Key key{update.writer_epoch, update.batch_id};
    const auto deliver = [this](const StateCommitResult& result) -> std::optional<StateCommitResult> {
        if (drop_replies_ > 0) {
            --drop_replies_;
            return std::nullopt;
        }
        return result;
    };
    if (const auto found = commits_.find(key); found != commits_.end()) {
        if (found->second.update == update) {
            return deliver(found->second.result);
        }
        return current(update, CommitStatus::Invalid, "changed duplicate payload");
    }
    if (paused_) {
        return current(update, CommitStatus::Invalid, "writer needs explicit reconciliation/resume");
    }
    const auto query = queries_.find(key);
    if (query == queries_.end()) {
        return current(update, CommitStatus::Invalid, "missing required query");
    }
    if (commits_.size() >= kRetainedBatches) {
        return current(update, CommitStatus::Invalid, "commit retention is full");
    }
    const auto retain = [&](CommitStatus status, std::string_view reason) {
        const StateCommitResult result = current(update, status, reason);
        commits_[key] = RetainedCommit{update, result};
        return deliver(result);
    };

    if (const std::string_view reason = validate(update, query->second); !reason.empty()) {
        return retain(CommitStatus::Invalid, reason);
    }
    if (update.base_state_version != snapshot_.state_version) {
        return retain(CommitStatus::Conflict, "stale base version");
    }
    const MatchState& next = update.match;
    const MatchState& previous = snapshot_.match;
    if (update.target_simulation_tick <= snapshot_.simulation_tick ||
        update.target_simulation_tick > snapshot_.simulation_tick + 4) {
        return retain(CommitStatus::Invalid, "target tick outside catch-up window");
    }
    if (next.pi_reference_session_id != previous.pi_reference_session_id ||
        next.pi_reference_clock_id != previous.pi_reference_clock_id) {
        return retain(CommitStatus::Invalid, "Pi reference cannot change inside a live writer epoch");
    }
    if (next.state_time_pi_us < previous.state_time_pi_us || next.point_id < previous.point_id) {
        return retain(CommitStatus::Invalid, "Pi state time or point identity moves backwards");
    }
    bool award = false;
    for (const RuleEvent& event : update.rule_events) {
        award = award || is_award(event.kind);
    }
    if (!(next.score == previous.score) && !award) {
        return retain(CommitStatus::Invalid, "score change lacks an explicit sporting award event");
    }

    snapshot_.state_version += 1;
    snapshot_.simulation_tick = update.target_simulation_tick;
    snapshot_.match = update.match;
    ++accepted_commits_;
    return retain(CommitStatus::Accepted, {});
}

StateCommitResult FakeStateService::lookup(std::uint64_t writer_epoch, std::uint64_t batch_id) const {
    if (const auto found = commits_.find(Key{writer_epoch, batch_id}); found != commits_.end()) {
        return found->second.result;
    }
    StateUpdate identity;
    identity.game_session_id = setup_.game_session_id;
    identity.writer_epoch = writer_epoch;
    identity.batch_id = batch_id;
    return current(identity, CommitStatus::UnknownBatch, "unknown batch");
}

void FakeStateService::restart_writer(std::uint64_t new_epoch) {
    if (new_epoch > writer_epoch_) {
        writer_epoch_ = new_epoch;
        paused_ = true;
    }
}

std::optional<PoseObservation> FakePoseSource::latest_for(std::uint32_t person_id) const {
    if (const auto found = poses_.find(person_id); found != poses_.end()) {
        return found->second;
    }
    return std::nullopt;
}

ContactResult ScriptedContactEvaluator::evaluate(const MatchState& candidate, const PendingInput& input,
                                                 const PoseObservation&, const PlayerBinding&) const {
    observations_.push_back(Observation{input.input.transaction, candidate.ball.velocity_m_s});
    const auto found = hits_.find({input.input.transaction.controller_id, input.input.transaction.request_seq});
    if (found == hits_.end()) {
        return ContactResult{Outcome::Miss, {}};
    }
    return ContactResult{Outcome::Hit, found->second};
}

}  // namespace motion_tennis::physics::fakes
