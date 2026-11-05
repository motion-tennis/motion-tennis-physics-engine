#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "motion_tennis/physics/adapters.hpp"
#include "motion_tennis/physics/contract.hpp"

// Test-only adapters for the S1 slice. They may allocate; the core may not.
namespace motion_tennis::physics::fakes {

// In-memory committed-state service following the offline v0.3 oracle
// (documentation/scripts/match_contract.py): single fenced writer, retained
// query/commit outcomes, exact-version acceptance and whole-batch rejection.
// It is a test double, not the C# game state service.
class FakeStateService final : public StateService {
public:
    static constexpr std::size_t kRetainedBatches = 512;

    FakeStateService(const MatchSetup& setup, const Snapshot& initial);

    BallStateResponse query(const BallStateRequest& request) override;
    std::optional<StateCommitResult> commit(const StateUpdate& update) override;

    // Control-plane commit lookup for an uncertain batch.
    StateCommitResult lookup(std::uint64_t writer_epoch, std::uint64_t batch_id) const;

    void restart_writer(std::uint64_t new_epoch);
    void resume_writer() { paused_ = false; }

    // Fault injection.
    void drop_next_commit_replies(std::uint32_t count) { drop_replies_ = count; }
    void bump_version_for_test() { ++snapshot_.state_version; }

    const Snapshot& snapshot() const { return snapshot_; }
    std::uint64_t writer_epoch() const { return writer_epoch_; }
    std::uint64_t accepted_commits() const { return accepted_commits_; }

private:
    using Key = std::pair<std::uint64_t, std::uint64_t>;
    struct RetainedQuery {
        BallStateRequest request;
        BallStateResponse response;
    };
    struct RetainedCommit {
        StateUpdate update;
        StateCommitResult result;
    };

    StateCommitResult current(const StateUpdate& update, CommitStatus status, std::string_view reason) const;
    std::string_view validate(const StateUpdate& update, const RetainedQuery& query) const;

    MatchSetup setup_;
    Snapshot snapshot_;
    std::uint64_t writer_epoch_;
    bool paused_ = false;
    std::uint32_t drop_replies_ = 0;
    std::uint64_t accepted_commits_ = 0;
    std::map<Key, RetainedQuery> queries_;
    std::map<Key, RetainedCommit> commits_;
};

class FakePoseSource final : public PoseSource {
public:
    void set(std::uint32_t person_id, const PoseObservation& pose) { poses_[person_id] = pose; }
    void remove(std::uint32_t person_id) { poses_.erase(person_id); }
    std::optional<PoseObservation> latest_for(std::uint32_t person_id) const override;

private:
    std::map<std::uint32_t, PoseObservation> poses_;
};

// Scripted contact model keyed by (controller_id, request_seq). Unscripted
// inputs miss. It records the candidate ball velocity it was shown, which lets
// fixtures prove that later events see earlier candidate results.
class ScriptedContactEvaluator final : public ContactEvaluator {
public:
    struct Observation {
        Transaction transaction;
        Vector3 incoming_velocity_m_s;
    };

    void script_hit(std::uint32_t controller_id, std::uint64_t request_seq, const Vector3& outgoing) {
        hits_[{controller_id, request_seq}] = outgoing;
    }
    ContactResult evaluate(const MatchState& candidate, const PendingInput& input,
                           const PoseObservation& pose, const PlayerBinding& binding) const override;

    const std::vector<Observation>& observations() const { return observations_; }
    void clear_observations() { observations_.clear(); }

private:
    std::map<std::pair<std::uint32_t, std::uint64_t>, Vector3> hits_;
    mutable std::vector<Observation> observations_;  // Diagnostic only; never affects results.
};

}  // namespace motion_tennis::physics::fakes
