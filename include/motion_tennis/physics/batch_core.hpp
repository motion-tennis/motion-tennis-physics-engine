#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "motion_tennis/physics/adapters.hpp"
#include "motion_tennis/physics/bounded_vector.hpp"
#include "motion_tennis/physics/contract.hpp"
#include "motion_tennis/physics/input_scheduler.hpp"
#include "motion_tennis/physics/simulation.hpp"

namespace motion_tennis::physics {

inline constexpr std::size_t kFeedbackCapacity = 32;
inline constexpr std::size_t kTraceCapacity = 256;
inline constexpr std::uint32_t kMaxCommitResends = 2;

enum class BatchPhase { Idle, QueryPending, Compute, CommitPending, Paused };

enum class PauseReason {
    None,
    SimulationLag,
    QueryRejected,
    CommitRejected,
    WrongWriter,
    CommitAmbiguous,
    CommitUnknown,
};

// Pi-local gameplay side-state that is staged with each candidate and
// installed only after an accepted commit.
struct SideState {
    std::uint64_t next_swing_id = 1;
    std::uint64_t next_cue_id = 1;
    friend bool operator==(const SideState&, const SideState&) = default;
};

enum class TraceKind {
    QueryReleased,
    InputFailed,
    DecisionStaged,
    CommitSubmitted,
    CommitResent,
    CommitResolved,
};

// Asynchronous diagnostic record; never consulted by gameplay decisions.
struct TraceRecord {
    TraceKind kind = TraceKind::QueryReleased;
    std::uint64_t writer_epoch = 0;
    std::uint64_t batch_id = 0;
    std::uint64_t query_id = 0;
    Transaction transaction;
    std::uint32_t event_index = 0;
    Outcome outcome = Outcome::Unspecified;
    CommitStatus commit_status = CommitStatus::Unspecified;
    std::uint64_t state_version = 0;
    std::uint64_t simulation_tick = 0;
    std::uint64_t observed_pi_us = 0;
    Uuid tracking_session_id{};
    Uuid pi_reference_session_id{};
    std::uint32_t pi_reference_clock_id = 0;
    std::uint32_t person_id = 0;
    std::uint32_t calibration_id = 0;
    SourceTime pose_raw_capture;
    std::uint64_t pose_capture_pi_us = 0;
    std::uint64_t pose_mapping_id = 0;
    std::uint32_t pose_mapping_version = 0;
    std::uint32_t pose_total_uncertainty_us = 0;
    friend bool operator==(const TraceRecord&, const TraceRecord&) = default;
};

struct CoreCounters {
    std::uint64_t batches_released = 0;
    std::uint64_t periodic_batches = 0;
    std::uint64_t batches_accepted = 0;
    std::uint64_t batches_conflicted = 0;
    std::uint64_t batches_failed = 0;
    std::uint64_t skipped_releases = 0;
    std::uint64_t idle_releases = 0;
    std::uint64_t paused_releases = 0;
    std::uint64_t rebuilds = 0;
    std::uint64_t input_failures = 0;
    std::uint64_t commit_resends = 0;
    std::uint64_t unmatched_replies = 0;
    std::uint64_t feedback_lost = 0;
    std::uint64_t trace_lost = 0;
};

// Single-writer batch state machine:
// Idle -> QueryPending -> Compute -> CommitPending -> Idle, errors -> Paused.
// It never writes committed state itself; the state service decides.
class BatchCore {
public:
    BatchCore(const MatchSetup& setup, std::uint64_t committed_tick, const PoseSource& poses,
              const ContactEvaluator& evaluator, SimulationParameters parameters = {});

    // Queues one controller input. Rejections produce explicit feedback.
    AdmitResult admit(const ControllerInput& input, std::uint64_t arrival_pi_us);

    // Starts one batch toward the desired logical tick, or returns nullopt when
    // busy, paused, already current or lagging beyond the catch-up limit.
    std::optional<BallStateRequest> release(std::uint64_t desired_tick);

    // Validates the echoed query and atomic snapshot, then computes the candidate.
    void on_query_response(const BallStateResponse& response, std::uint64_t now_pi_us);

    // Rechecks every input immediately before submission and rebuilds the full
    // candidate from the queried snapshot if one became invalid.
    std::optional<StateUpdate> prepare_commit(std::uint64_t now_pi_us);

    // Returns the identical update for a resend, or nullopt after the last
    // resend, when the writer pauses with an ambiguous commit.
    std::optional<StateUpdate> on_commit_timeout();

    // Applies an accepted, rejected or looked-up commit outcome exactly once.
    void on_commit_result(const StateCommitResult& result);

    // Explicit operator recovery after a resolvable pause.
    bool resume(std::uint64_t committed_tick);

    BoundedVector<FeedbackResult, kFeedbackCapacity> take_feedback();
    BoundedVector<TraceRecord, kTraceCapacity> take_trace();

    BatchPhase phase() const { return phase_; }
    PauseReason pause_reason() const { return pause_reason_; }
    const CoreCounters& counters() const { return counters_; }
    const SchedulerCounters& scheduler_counters() const { return scheduler_.counters(); }
    const SideState& side_state() const { return side_state_; }
    std::uint64_t writer_epoch() const { return setup_.writer_epoch; }
    std::uint64_t known_committed_tick() const { return known_committed_tick_; }
    std::size_t pending_inputs() const { return scheduler_.total_pending(); }

private:
    struct InFlight {
        BallStateRequest request;
        BoundedVector<PendingInput, kMaxBatchQueries> inputs;
        std::array<PoseObservation, kMaxBatchQueries> poses{};
        std::array<Outcome, kMaxBatchQueries> failures{};
        std::uint64_t target_tick = 0;
        Snapshot snapshot;
        StateUpdate update;
        SideState staged_side;
        std::uint32_t resends = 0;
    };

    Outcome check_input(std::size_t index, std::uint64_t now_pi_us, bool capture_pose);
    bool recheck_inputs(std::uint64_t now_pi_us, bool capture_pose);
    void build_candidate();
    void fail_batch(Outcome outcome, PauseReason reason, std::string_view why);
    void pause(PauseReason reason);
    void emit_feedback(const FeedbackResult& feedback);
    void emit_trace(const TraceRecord& record);
    FeedbackResult feedback_for(const Transaction& transaction) const;
    const PlayerBinding& binding_for_slot(std::uint32_t slot) const;

    MatchSetup setup_;
    const PoseSource& poses_;
    const ContactEvaluator& evaluator_;
    SimulationParameters parameters_;
    InputScheduler scheduler_;

    BatchPhase phase_ = BatchPhase::Idle;
    PauseReason pause_reason_ = PauseReason::None;
    std::uint64_t known_committed_tick_ = 0;
    std::uint64_t next_batch_id_ = 1;
    std::uint64_t next_query_id_ = 1;
    SideState side_state_;
    InFlight in_flight_;

    BoundedVector<FeedbackResult, kFeedbackCapacity> feedback_;
    BoundedVector<TraceRecord, kTraceCapacity> trace_;
    CoreCounters counters_;
};

// Drives one complete synchronous batch against a state service. Used by the
// S1 fixtures; the real runtime will drive the same steps from I/O completions.
struct BatchRun {
    bool released = false;
    std::optional<StateCommitResult> result;
};
BatchRun run_batch(BatchCore& core, StateService& service, std::uint64_t desired_tick,
                   std::uint64_t now_pi_us);

}  // namespace motion_tennis::physics
