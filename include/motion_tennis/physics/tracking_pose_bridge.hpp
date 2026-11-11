#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "motion_tennis/physics/adapters.hpp"

namespace motion_tennis::physics {

inline constexpr std::size_t kTrackingMapCapacity = 16;
inline constexpr std::size_t kTrackingSessionHistory = 64;
inline constexpr std::size_t kTrackingDiagnosticCapacity = 64;
inline constexpr std::uint32_t kMaximumCaptureUncertaintyUs = 8000;
inline constexpr std::uint32_t kMaximumMapUncertaintyUs = 2000;
inline constexpr std::uint64_t kMaximumMapAgeUs = 5000000;
inline constexpr std::uint64_t kMaximumPoseAgeUs = 80000;

enum class BridgeCode : std::uint32_t {
    Accepted, NotConfigured, InvalidSession, RetiredSession, Capacity,
    InvalidBinding, MissingBindings, InvalidMap, MapConflict, MissingMap,
    MapExpired, MapUncertain, SourceMismatch, ReferenceMismatch,
    InvalidEnvelope, DuplicateFrame, ReorderedFrame, IncompleteFrame,
    ContradictoryFrame, SourceLossRegression, InvalidPose, CalibrationMismatch,
    InvalidTime, StalePose, RequiredJointsUnavailable, ObservationMismatch, Count,
};

struct MappedCapture {
    std::uint64_t capture_pi_us = 0;
    std::uint32_t total_uncertainty_us = 0;
};

// Also used by BatchCore to recheck caller-supplied PoseSource implementations.
BridgeCode map_tracking_capture(const TrackingClockMap& map, const SourceTime& capture,
                                const TrackingSourceSession& source,
                                const PiClockReference& reference, std::uint64_t now_pi_us,
                                MappedCapture& result);
BridgeCode validate_tracking_pose(const PoseObservation& pose, const PlayerBinding& binding,
                                  const PiClockReference& reference, std::uint64_t now_pi_us,
                                  std::uint64_t maximum_age_us = kMaximumPoseAgeUs);

struct BridgeDiagnostic {
    BridgeCode code = BridgeCode::NotConfigured;
    TrackingSourceSession source;
    PiClockReference reference;
    std::uint64_t observed_pi_us = 0;
    bool observed_time_valid = false;
    std::uint64_t frame_id = 0;
    std::uint32_t person_id = 0;
    SourceTime raw_capture;
    std::uint64_t mapping_id = 0;
    std::uint32_t mapping_version = 0;
    std::uint64_t capture_pi_us = 0;
    std::uint32_t total_uncertainty_us = 0;
};

struct TrackingBridgeCounters {
    std::uint64_t frames_received = 0;
    std::uint64_t frames_accepted = 0;
    std::uint64_t frames_rejected = 0;
    std::uint64_t source_frames_dropped_total = 0;
    std::uint64_t frame_sequence_gaps = 0;
    std::uint64_t invalidated_poses = 0;
    std::uint64_t unusable_poses = 0;
    std::uint64_t unbound_people = 0;
    std::uint64_t session_restarts = 0;
    std::uint64_t reference_restarts = 0;
    std::uint64_t diagnostics_lost = 0;
    std::array<std::uint64_t, static_cast<std::size_t>(BridgeCode::Count)> rejections{};
};

class TrackingPoseBridge final : public PoseSource {
public:
    BridgeCode start_session(const TrackingSourceSession& source, const PiClockReference& reference,
                             const TrackingQualityPolicy& quality);
    BridgeCode install_confirmed_bindings(const BoundedVector<PlayerBinding, kPlayers>& bindings);
    BridgeCode install_clock_map(const TrackingClockMap& map);
    BridgeCode ingest_frame(const TrackingFrameBatch& batch, std::uint64_t now_pi_us);

    std::optional<PoseObservation> latest_for(std::uint32_t person_id) const override;
    BoundedVector<BridgeDiagnostic, kTrackingDiagnosticCapacity> take_diagnostics();
    const TrackingBridgeCounters& counters() const { return counters_; }
    BridgeCode last_result() const { return last_result_; }

private:
    void invalidate();
    BridgeCode reject(BridgeCode code, std::uint64_t now_pi_us = 0,
                      std::uint64_t frame_id = 0, bool rejected_frame = false);
    void diagnose(BridgeCode code, std::uint64_t now_pi_us, std::uint64_t frame_id,
                  const PoseObservation* pose = nullptr, bool observed_time_valid = false);
    const PlayerBinding* binding_for(std::uint32_t person_id) const;

    bool configured_ = false;
    TrackingSourceSession source_;
    PiClockReference reference_;
    TrackingQualityPolicy quality_;
    BoundedVector<Uuid, kTrackingSessionHistory> retired_sessions_;
    BoundedVector<PiClockReference, kTrackingSessionHistory> retired_references_;
    BoundedVector<PlayerBinding, kPlayers> bindings_;
    BoundedVector<TrackingClockMap, kTrackingMapCapacity> maps_;
    std::size_t active_map_ = 0;
    BoundedVector<PoseObservation, kPlayers> poses_;
    std::uint64_t last_frame_id_ = 0;
    std::uint64_t last_accepted_frame_id_ = 0;
    std::uint64_t last_message_seq_ = 0;
    std::uint64_t last_source_drops_ = 0;
    std::uint64_t last_now_pi_us_ = 0;
    BridgeCode last_result_ = BridgeCode::NotConfigured;
    TrackingBridgeCounters counters_;
    BoundedVector<BridgeDiagnostic, kTrackingDiagnosticCapacity> diagnostics_;
};

}  // namespace motion_tennis::physics
