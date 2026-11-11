#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "motion_tennis/physics/bounded_vector.hpp"

// Typed in-memory mirror of the accepted motion.v03 messages used by the S1
// batch core. Field names and enum numbers follow spec/motion.proto in the
// documentation repository. This is not a wire codec: generated Protobuf/nanopb
// encoding, canonical bytes and the configuration digest remain later work.
namespace motion_tennis::physics {

using Uuid = std::array<std::uint8_t, 16>;
using Sha256 = std::array<std::uint8_t, 32>;

inline constexpr std::uint64_t kMaxWireUint64 = (std::uint64_t{1} << 56) - 1;
inline constexpr std::size_t kMaxBatchQueries = 8;
inline constexpr std::size_t kMaxQueriesPerController = 2;
inline constexpr std::size_t kMaxRuleEvents = 16;
inline constexpr std::size_t kMaxCompletedSets = 5;
inline constexpr std::size_t kPlayers = 4;
inline constexpr std::uint32_t kRuleEventIndexBase = 8;

enum class QueryStatus : std::uint32_t { Unspecified = 0, Ok = 1, Rejected = 2 };

enum class Outcome : std::uint32_t {
    Unspecified = 0,
    Hit = 1,
    Miss = 2,
    InvalidInput = 3,
    TrackingUnavailable = 4,
    StaleInput = 5,
    StateConflict = 6,
    Timeout = 7,
    Overloaded = 8,
    InternalError = 9,
};

enum class CueKind : std::uint32_t { None = 0, Hit = 1, Miss = 2, Fault = 3 };

enum class CommitStatus : std::uint32_t {
    Unspecified = 0,
    Accepted = 1,
    Conflict = 2,
    Invalid = 3,
    WrongSession = 4,
    WrongWriter = 5,
    UnknownBatch = 6,
};

enum class Team : std::uint32_t { None = 0, A = 1, B = 2 };
enum class MatchPhase : std::uint32_t { Unspecified = 0, Ready = 1, Playing = 2, Paused = 3, Finished = 4 };
enum class MovementMode : std::uint32_t {
    Unspecified = 0,
    Holding = 1,
    Serving = 2,
    Receiving = 3,
    Covering = 4,
    Intercepting = 5,
};
enum class ServePhase : std::uint32_t {
    Unspecified = 0,
    Ready = 1,
    Lift = 2,
    Toss = 3,
    Contact = 4,
    InFlight = 5,
    Rally = 6,
    Fault = 7,
    Let = 8,
    Interrupted = 9,
};
enum class ServiceSide : std::uint32_t { Unspecified = 0, Deuce = 1, Ad = 2 };
enum class PointScore : std::uint32_t { Love = 0, Fifteen = 1, Thirty = 2, Forty = 3, Advantage = 4 };
enum class RuleEventKind : std::uint32_t {
    Unspecified = 0,
    TossStarted = 1,
    TossCancelled = 2,
    ServeAttempt = 3,
    LegalContact = 4,
    BallBounce = 5,
    BallOut = 6,
    ServeFaultEvent = 7,
    ServeLetEvent = 8,
    PointAwarded = 9,
    GameAwarded = 10,
    SetAwarded = 11,
    MatchAwarded = 12,
    EndsChanged = 13,
    TechnicalPause = 14,
    PlayResumed = 15,
};

struct Vector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    friend bool operator==(const Vector3&, const Vector3&) = default;
};

struct Transaction {
    std::uint32_t controller_id = 0;
    Uuid session_id{};
    std::uint64_t request_seq = 0;
    friend bool operator==(const Transaction&, const Transaction&) = default;
};

struct QueryEntry {
    std::uint64_t query_id = 0;
    Transaction transaction;
    friend bool operator==(const QueryEntry&, const QueryEntry&) = default;
};

struct BallStateRequest {
    Uuid game_session_id{};
    std::uint64_t writer_epoch = 0;
    std::uint64_t batch_id = 0;
    // Empty queries means periodic-only work, never an unidentified controller input.
    BoundedVector<QueryEntry, kMaxBatchQueries> queries;
    friend bool operator==(const BallStateRequest&, const BallStateRequest&) = default;
};

struct Ball {
    Vector3 position_m;
    Vector3 velocity_m_s;
    float radius_m = 0.0f;
    std::uint32_t calibration_id = 0;
    friend bool operator==(const Ball&, const Ball&) = default;
};

struct AvatarState {
    std::uint32_t player_slot = 0;
    Vector3 position_m;
    Vector3 velocity_m_s;
    float heading_rad = 0.0f;
    Vector3 target_m;
    MovementMode movement = MovementMode::Unspecified;
    std::uint32_t coverage_owner_slot = 0;
    friend bool operator==(const AvatarState&, const AvatarState&) = default;
};

struct ServeState {
    ServePhase phase = ServePhase::Unspecified;
    std::uint32_t server_slot = 0;
    std::uint32_t receiver_slot = 0;
    ServiceSide side = ServiceSide::Unspecified;
    std::uint32_t attempt = 0;
    bool stroke_attempted = false;
    bool net_touched = false;
    std::uint64_t toss_id = 0;
    std::array<std::uint32_t, kPlayers> service_order{};
    std::array<std::uint32_t, kPlayers> receiving_order{};
    std::uint32_t service_turn_index = 0;
    std::uint32_t tiebreak_first_server_slot = 0;
    friend bool operator==(const ServeState&, const ServeState&) = default;
};

struct RallyState {
    std::uint64_t rally_id = 0;
    std::uint32_t last_hitter_slot = 0;
    Team last_bounce_team = Team::None;
    std::uint32_t bounce_count = 0;
    bool net_touched = false;
    bool service_return_pending = false;
    std::uint32_t contact_slot_mask = 0;
    std::uint64_t contact_id = 0;
    std::uint64_t bounce_id = 0;
    Team point_winner = Team::None;
    friend bool operator==(const RallyState&, const RallyState&) = default;
};

struct CompletedSet {
    std::uint32_t games_a = 0;
    std::uint32_t games_b = 0;
    bool had_tiebreak = false;
    std::uint64_t tiebreak_points_a = 0;
    std::uint64_t tiebreak_points_b = 0;
    friend bool operator==(const CompletedSet&, const CompletedSet&) = default;
};

struct ScoreState {
    PointScore points_a = PointScore::Love;
    PointScore points_b = PointScore::Love;
    std::uint32_t games_a = 0;
    std::uint32_t games_b = 0;
    std::uint32_t sets_a = 0;
    std::uint32_t sets_b = 0;
    bool tiebreak = false;
    std::uint64_t tiebreak_points_a = 0;
    std::uint64_t tiebreak_points_b = 0;
    BoundedVector<CompletedSet, kMaxCompletedSets> completed_sets;
    friend bool operator==(const ScoreState&, const ScoreState&) = default;
};

struct MatchState {
    Ball ball;
    std::uint32_t roster_revision = 0;
    std::uint32_t rules_revision = 0;
    Sha256 configuration_sha256{};
    std::array<AvatarState, kPlayers> avatars{};
    ServeState serve;
    RallyState rally;
    ScoreState score;
    bool team_a_near_end = false;
    MatchPhase phase = MatchPhase::Unspecified;
    Team winner = Team::None;
    std::uint64_t point_id = 0;
    std::uint64_t state_time_pi_us = 0;
    Uuid pi_reference_session_id{};
    std::uint32_t pi_reference_clock_id = 0;
    friend bool operator==(const MatchState&, const MatchState&) = default;
};

struct Snapshot {
    Uuid game_session_id{};
    std::uint64_t state_version = 0;
    std::uint64_t simulation_tick = 0;
    std::uint32_t tick_duration_us = 0;
    MatchState match;
    friend bool operator==(const Snapshot&, const Snapshot&) = default;
};

struct BallStateResponse {
    std::uint64_t batch_id = 0;
    BoundedVector<QueryEntry, kMaxBatchQueries> queries;
    QueryStatus status = QueryStatus::Unspecified;
    bool has_snapshot = false;
    Snapshot snapshot;
    std::string_view reason;
    std::uint64_t writer_epoch = 0;
    Uuid game_session_id{};
    friend bool operator==(const BallStateResponse&, const BallStateResponse&) = default;
};

struct Cue {
    CueKind kind = CueKind::None;
    std::uint64_t cue_id = 0;
    float intensity = 0.0f;
    std::uint32_t duration_ms = 0;
    bool electrical_eligible = false;
    friend bool operator==(const Cue&, const Cue&) = default;
};

struct Decision {
    Transaction transaction;
    std::uint64_t current_sample_id = 0;
    std::uint64_t pose_frame_id = 0;
    std::uint32_t person_id = 0;
    std::uint32_t association_version = 0;
    std::uint64_t imu_capture_us = 0;
    std::uint64_t pose_capture_us = 0;
    Outcome outcome = Outcome::Unspecified;
    std::uint32_t event_index = 0;
    std::uint64_t swing_id = 0;
    Cue cue;
    friend bool operator==(const Decision&, const Decision&) = default;
};

struct RuleEvent {
    std::uint32_t event_index = 0;
    RuleEventKind kind = RuleEventKind::Unspecified;
    std::uint64_t point_id = 0;
    std::uint32_t player_slot = 0;
    Team team = Team::None;
    bool has_cause = false;
    std::uint32_t cause_decision_index = 0;
    std::uint64_t simulation_tick = 0;
    std::uint32_t serve_attempt = 0;
    friend bool operator==(const RuleEvent&, const RuleEvent&) = default;
};

struct StateUpdate {
    Uuid game_session_id{};
    std::uint64_t writer_epoch = 0;
    std::uint64_t batch_id = 0;
    std::uint64_t base_state_version = 0;
    std::uint64_t target_simulation_tick = 0;
    Uuid tracking_session_id{};
    BoundedVector<Decision, kMaxBatchQueries> decisions;
    MatchState match;
    BoundedVector<RuleEvent, kMaxRuleEvents> rule_events;
    friend bool operator==(const StateUpdate&, const StateUpdate&) = default;
};

struct StateCommitResult {
    Uuid game_session_id{};
    std::uint64_t writer_epoch = 0;
    std::uint64_t batch_id = 0;
    CommitStatus status = CommitStatus::Unspecified;
    std::uint64_t state_version = 0;
    std::uint64_t simulation_tick = 0;
    std::string_view reason;
    friend bool operator==(const StateCommitResult&, const StateCommitResult&) = default;
};

struct FeedbackResult {
    Transaction transaction;
    Outcome outcome = Outcome::Unspecified;
    bool committed = false;
    Uuid game_session_id{};
    std::uint64_t writer_epoch = 0;
    std::uint64_t batch_id = 0;
    std::uint32_t event_index = 0;
    std::uint64_t state_version = 0;
    std::uint64_t simulation_tick = 0;
    Cue cue;
    std::string_view reason;
    friend bool operator==(const FeedbackResult&, const FeedbackResult&) = default;
};

struct PlayerBinding {
    std::uint32_t player_slot = 0;
    std::uint32_t controller_id = 0;
    Uuid controller_session_id{};
    Uuid tracking_session_id{};
    std::uint32_t person_id = 0;
    std::uint32_t avatar_id = 0;
    std::uint32_t association_version = 0;
    std::uint32_t calibration_id = 0;
    bool right_hand = false;
    std::uint32_t physical_station_id = 0;
    friend bool operator==(const PlayerBinding&, const PlayerBinding&) = default;
};

struct MatchRules {
    std::uint32_t sets_to_win = 0;
    std::uint32_t games_to_win_set = 0;
    std::uint32_t minimum_game_lead = 0;
    std::uint32_t tiebreak_at_games = 0;
    std::uint32_t tiebreak_points_to_win = 0;
    std::uint32_t tiebreak_minimum_lead = 0;
    bool final_set_tiebreak = false;
    Sha256 tuning_sha256{};
    friend bool operator==(const MatchRules&, const MatchRules&) = default;
};

struct MatchConfiguration {
    std::uint32_t roster_revision = 0;
    std::uint32_t rules_revision = 0;
    std::array<PlayerBinding, kPlayers> players{};
    MatchRules rules;
    friend bool operator==(const MatchConfiguration&, const MatchConfiguration&) = default;
};

struct MatchSetup {
    std::uint64_t command_id = 0;
    Uuid game_session_id{};
    MatchConfiguration configuration;
    // Opaque in S1: the digest over deterministic Protobuf encoding needs the
    // generated codec, which is not part of this slice.
    Sha256 configuration_sha256{};
    std::uint64_t writer_epoch = 0;
    friend bool operator==(const MatchSetup&, const MatchSetup&) = default;
};

// Controller input after Pi receipt. Clock mapping is out of S1 scope, so the
// fake adapter supplies the capture already expressed in Pi reference time
// together with the immutable mapping version it would have used.
struct ControllerInput {
    Transaction transaction;
    std::uint64_t t0_us = 0;
    std::uint64_t sample_id = 0;
    std::uint64_t capture_pi_us = 0;
    std::uint32_t clock_mapping_version = 0;
    std::uint32_t association_version = 0;
    std::uint32_t calibration_id = 0;
    Vector3 specific_force_m_s2;
    Vector3 angular_velocity_rad_s;
    bool sensor_saturated = false;
    friend bool operator==(const ControllerInput&, const ControllerInput&) = default;
};

enum class TimeKind : std::uint32_t {
    Unknown = 0, IrqMonotonic = 1, SensorCapture = 2, DriverFrame = 3, Presentation = 4,
};
enum class TrackingState : std::uint32_t { Unspecified = 0, Tracked = 1, Inferred = 2, Unavailable = 3 };

struct SourceTime {
    std::uint64_t value_us = 0;
    std::uint32_t clock_id = 0;
    TimeKind kind = TimeKind::Unknown;
    std::uint32_t uncertainty_us = 0;
    bool timing_valid = false;
    friend bool operator==(const SourceTime&, const SourceTime&) = default;
};

struct Joint2D {
    std::uint32_t coco_joint_index = 0;
    float x_normalized = 0.0f;
    float y_normalized = 0.0f;
    float score = 0.0f;
    TrackingState state = TrackingState::Unspecified;
    friend bool operator==(const Joint2D&, const Joint2D&) = default;
};

// Typed mirrors of v0.3 JSON/Protobuf fields. Decoding byte identities and wire
// validation belong to the caller; this bridge validates their typed meanings.
struct TrackingEnvelope {
    std::uint32_t major = 0;
    std::uint32_t minor = 0;
    std::uint32_t sender_id = 0;
    Uuid sender_session_id{};
    Uuid run_id{};
    std::uint64_t message_seq = 0;
    std::uint64_t sent_time_us = 0;
    std::uint32_t clock_id = 0;
    friend bool operator==(const TrackingEnvelope&, const TrackingEnvelope&) = default;
};

struct SkeletonFrame {
    std::uint64_t frame_id = 0;
    std::uint32_t person_id = 0;
    SourceTime capture;
    std::uint64_t processing_complete_us = 0;
    std::uint32_t processing_clock_id = 0;
    std::uint32_t image_width = 0;
    std::uint32_t image_height = 0;
    std::uint32_t calibration_id = 0;
    TrackingState state = TrackingState::Unspecified;
    BoundedVector<Joint2D, 17> joints;
    friend bool operator==(const SkeletonFrame&, const SkeletonFrame&) = default;
};

struct FrameStatus {
    std::uint64_t frame_id = 0;
    BoundedVector<std::uint32_t, kPlayers> person_ids;
    std::uint64_t frames_dropped_total = 0;
    bool camera_ready = false;
    friend bool operator==(const FrameStatus&, const FrameStatus&) = default;
};

struct TrackingPersonPacket {
    TrackingEnvelope envelope;
    SkeletonFrame frame;
    friend bool operator==(const TrackingPersonPacket&, const TrackingPersonPacket&) = default;
};

// Atomic offline handoff: exactly the people named by this status, from one
// source frame. UDP assembly, partial-packet deadlines and buffering are later work.
struct TrackingFrameBatch {
    TrackingEnvelope status_envelope;
    FrameStatus status;
    BoundedVector<TrackingPersonPacket, kPlayers> people;
    friend bool operator==(const TrackingFrameBatch&, const TrackingFrameBatch&) = default;
};

struct TrackingSourceSession {
    std::uint32_t sender_id = 0;
    Uuid session_id{};
    Uuid run_id{};
    std::uint32_t clock_id = 0;
    friend bool operator==(const TrackingSourceSession&, const TrackingSourceSession&) = default;
};

struct PiClockReference {
    Uuid session_id{};
    std::uint32_t clock_id = 0;
    friend bool operator==(const PiClockReference&, const PiClockReference&) = default;
};

// Caller-installed, immutable Pi-owned evidence. Anchored rational rates avoid
// losing microseconds at uint56 epochs. No clock fitter or measured map is implied.
struct TrackingClockMap {
    std::uint64_t mapping_id = 0;
    std::uint32_t version = 0;
    TrackingSourceSession source;
    PiClockReference reference;
    std::uint64_t source_anchor_us = 0;
    std::uint64_t pi_anchor_us = 0;
    std::uint32_t rate_numerator = 0;
    std::uint32_t rate_denominator = 0;
    std::uint64_t valid_from_source_us = 0;
    std::uint64_t valid_until_source_us = 0;
    std::uint64_t issued_pi_us = 0;
    std::uint64_t last_good_pi_us = 0;
    std::uint64_t valid_from_pi_us = 0;
    std::uint64_t valid_until_pi_us = 0;
    std::uint32_t uncertainty_us = 0;
    // Pi-time error growth per raw source microsecond (ppb), matching the
    // offline clock-map profile's source-anchor basis.
    std::uint32_t uncertainty_growth_ppb = 0;
    bool valid = false;
    friend bool operator==(const TrackingClockMap&, const TrackingClockMap&) = default;
};

// Explicit fixture/deployment policy, not a measured model-confidence threshold.
// The zero default is invalid and cannot silently make observations eligible.
struct TrackingQualityPolicy {
    float minimum_joint_score = 0.0f;
    friend bool operator==(const TrackingQualityPolicy&, const TrackingQualityPolicy&) = default;
};

// Full source observation and map evidence accompany the mapped capture. The
// default has no provenance and is ineligible; no legacy permissive fallback.
struct PoseObservation {
    std::uint64_t frame_id = 0;
    std::uint64_t capture_pi_us = 0;
    std::uint32_t calibration_id = 0;
    bool tracked = false;
    std::uint32_t association_version = 0;
    TrackingEnvelope envelope;
    SkeletonFrame source_frame;
    TrackingClockMap mapping;
    TrackingQualityPolicy quality_policy;
    std::uint32_t total_uncertainty_us = 0;
    std::uint64_t received_pi_us = 0;
    friend bool operator==(const PoseObservation&, const PoseObservation&) = default;
};

inline Team team_of_slot(std::uint32_t slot) {
    if (slot == 1 || slot == 2) {
        return Team::A;
    }
    if (slot == 3 || slot == 4) {
        return Team::B;
    }
    return Team::None;
}

}  // namespace motion_tennis::physics
