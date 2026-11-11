#include "motion_tennis/physics/tracking_pose_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_tennis::physics {
namespace {

bool uuid_valid(const Uuid& id) {
    return std::any_of(id.begin(), id.end(), [](std::uint8_t value) { return value != 0; });
}
bool wire(std::uint64_t value) { return value <= kMaxWireUint64; }
bool source_valid(const TrackingSourceSession& source) {
    return source.sender_id != 0 && source.clock_id != 0 && uuid_valid(source.session_id) &&
           uuid_valid(source.run_id);
}
bool reference_valid(const PiClockReference& reference) {
    return uuid_valid(reference.session_id) && reference.clock_id != 0;
}
bool quality_valid(const TrackingQualityPolicy& quality) {
    return std::isfinite(quality.minimum_joint_score) && quality.minimum_joint_score > 0.0f &&
           quality.minimum_joint_score <= 1.0f;
}
TrackingSourceSession envelope_source(const TrackingEnvelope& envelope) {
    return {envelope.sender_id, envelope.sender_session_id, envelope.run_id, envelope.clock_id};
}
bool envelope_valid(const TrackingEnvelope& envelope) {
    return envelope.major == 0 && envelope.minor == 3 && source_valid(envelope_source(envelope)) &&
           envelope.message_seq != 0 && wire(envelope.message_seq) && wire(envelope.sent_time_us);
}
bool tracking_state_valid(TrackingState state) {
    return state == TrackingState::Tracked || state == TrackingState::Inferred ||
           state == TrackingState::Unavailable;
}
bool frame_shape_valid(const SkeletonFrame& frame, const TrackingEnvelope& envelope) {
    if (frame.frame_id == 0 || !wire(frame.frame_id) || frame.person_id == 0 || frame.calibration_id == 0 ||
        frame.image_width == 0 || frame.image_height == 0 || frame.image_width > 16384 ||
        frame.image_height > 16384 || !tracking_state_valid(frame.state) || frame.joints.size() != 17 ||
        frame.processing_clock_id != envelope.clock_id || !wire(frame.processing_complete_us) ||
        frame.capture.value_us > frame.processing_complete_us ||
        frame.processing_complete_us > envelope.sent_time_us) {
        return false;
    }
    std::array<bool, 17> seen{};
    for (const Joint2D& joint : frame.joints) {
        if (joint.coco_joint_index >= seen.size() || seen[joint.coco_joint_index] ||
            !std::isfinite(joint.x_normalized) || !std::isfinite(joint.y_normalized) ||
            !std::isfinite(joint.score) || joint.x_normalized < 0.0f || joint.x_normalized > 1.0f ||
            joint.y_normalized < 0.0f || joint.y_normalized > 1.0f || joint.score < 0.0f || joint.score > 1.0f ||
            !tracking_state_valid(joint.state)) {
            return false;
        }
        seen[joint.coco_joint_index] = true;
    }
    return true;
}
bool required_joints_usable(const SkeletonFrame& frame, const PlayerBinding& binding,
                            const TrackingQualityPolicy& policy) {
    if (!quality_valid(policy) || frame.state != TrackingState::Tracked) {
        return false;
    }
    // Both shoulder/hip anchors plus the playing arm. This is a conservative
    // fixture eligibility policy, not a calibrated pose or racket estimator.
    const std::array<std::uint32_t, 6> required{5, 6, 11, 12, binding.right_hand ? 8u : 7u,
                                               binding.right_hand ? 10u : 9u};
    for (const std::uint32_t index : required) {
        const auto joint = std::find_if(frame.joints.begin(), frame.joints.end(),
                                        [index](const Joint2D& value) { return value.coco_joint_index == index; });
        if (joint == frame.joints.end() || joint->state != TrackingState::Tracked ||
            joint->score < policy.minimum_joint_score) {
            return false;
        }
    }
    return true;
}
bool map_structure_valid(const TrackingClockMap& map) {
    return map.valid && map.mapping_id != 0 && wire(map.mapping_id) && map.version != 0 &&
           source_valid(map.source) && reference_valid(map.reference) &&
           map.rate_numerator != 0 && map.rate_numerator <= 1000000000 &&
           map.rate_denominator != 0 && map.rate_denominator <= 1000000000 &&
           wire(map.source_anchor_us) && wire(map.pi_anchor_us) &&
           wire(map.valid_from_source_us) && wire(map.valid_until_source_us) &&
           map.valid_from_source_us <= map.source_anchor_us &&
           map.source_anchor_us <= map.valid_until_source_us &&
           map.valid_until_source_us - map.valid_from_source_us <= kMaximumMapAgeUs &&
           wire(map.issued_pi_us) && wire(map.last_good_pi_us) &&
           wire(map.valid_from_pi_us) && wire(map.valid_until_pi_us) &&
           map.valid_from_pi_us <= map.pi_anchor_us && map.pi_anchor_us <= map.valid_until_pi_us &&
           map.valid_from_pi_us <= map.last_good_pi_us && map.last_good_pi_us <= map.issued_pi_us &&
           map.issued_pi_us <= map.valid_until_pi_us &&
           map.valid_until_pi_us - map.valid_from_pi_us <= kMaximumMapAgeUs &&
           map.valid_until_pi_us - map.last_good_pi_us <= kMaximumMapAgeUs &&
           map.uncertainty_us <= kMaximumMapUncertaintyUs;
}

// Outward-rounded bounds using integer arithmetic. The source validity window
// is at most five seconds and the rate numerator at most 1e9, so multiplication
// cannot overflow uint64 even when the native epoch is near the uint56 ceiling.
bool map_bound(const TrackingClockMap& map, std::uint64_t raw, bool upper, std::uint64_t& result) {
    const bool negative = raw < map.source_anchor_us;
    const std::uint64_t distance = negative ? map.source_anchor_us - raw : raw - map.source_anchor_us;
    if (distance > kMaximumMapAgeUs) {
        return false;
    }
    const std::uint64_t product = distance * map.rate_numerator;
    std::uint64_t step = product / map.rate_denominator;
    const bool remainder = product % map.rate_denominator != 0;
    if (remainder && (negative != upper)) {
        ++step;
    }
    if (negative) {
        if (step > map.pi_anchor_us) {
            return false;
        }
        result = map.pi_anchor_us - step;
    } else {
        if (step > kMaxWireUint64 - map.pi_anchor_us) {
            return false;
        }
        result = map.pi_anchor_us + step;
    }
    return true;
}

std::uint64_t distance_from(std::uint64_t left, std::uint64_t right) {
    return left > right ? left - right : right - left;
}
std::uint64_t grown_uncertainty(const TrackingClockMap& map, std::uint64_t distance) {
    if (distance > kMaximumMapAgeUs) {
        return kMaximumMapUncertaintyUs + 1;
    }
    const std::uint64_t product = distance * map.uncertainty_growth_ppb;
    return map.uncertainty_us + product / 1000000000 + (product % 1000000000 != 0 ? 1 : 0);
}

// Native processing/send times are event boundaries, not camera captures. Only
// the declared clock-map bound is applied here; no exposure uncertainty or
// unmeasured event-boundary accuracy is invented. Reject definitely future
// events and retain the original native boundary in the source records.
BridgeCode validate_native_event(const TrackingClockMap& map, std::uint64_t raw,
                                 std::uint64_t observed_pi_us) {
    if (!wire(raw) || raw < map.valid_from_source_us || raw > map.valid_until_source_us) {
        return BridgeCode::InvalidTime;
    }
    std::uint64_t low = 0, high = 0;
    if (!map_bound(map, raw, false, low) || !map_bound(map, raw, true, high)) {
        return BridgeCode::InvalidMap;
    }
    const std::uint64_t uncertainty = grown_uncertainty(map, distance_from(raw, map.source_anchor_us));
    if (uncertainty > kMaximumMapUncertaintyUs) {
        return BridgeCode::MapUncertain;
    }
    if (low < uncertainty || low - uncertainty < map.valid_from_pi_us ||
        low - uncertainty < map.issued_pi_us || uncertainty > kMaxWireUint64 - high ||
        high + uncertainty > map.valid_until_pi_us) {
        return BridgeCode::MapExpired;
    }
    return low - uncertainty > observed_pi_us ? BridgeCode::InvalidTime : BridgeCode::Accepted;
}

}  // namespace

BridgeCode map_tracking_capture(const TrackingClockMap& map, const SourceTime& capture,
                                const TrackingSourceSession& source, const PiClockReference& reference,
                                std::uint64_t now_pi_us, MappedCapture& result) {
    result = {};
    if (!map_structure_valid(map)) {
        return BridgeCode::InvalidMap;
    }
    if (map.source != source) {
        return BridgeCode::SourceMismatch;
    }
    if (map.reference != reference) {
        return BridgeCode::ReferenceMismatch;
    }
    if (!wire(now_pi_us) || !wire(capture.value_us) || !capture.timing_valid ||
        capture.clock_id != source.clock_id || capture.uncertainty_us > kMaximumCaptureUncertaintyUs ||
        (capture.kind != TimeKind::DriverFrame && capture.kind != TimeKind::SensorCapture)) {
        return BridgeCode::InvalidTime;
    }
    if (now_pi_us < map.issued_pi_us || now_pi_us < map.valid_from_pi_us ||
        now_pi_us > map.valid_until_pi_us || now_pi_us - map.last_good_pi_us > kMaximumMapAgeUs) {
        return BridgeCode::MapExpired;
    }
    if (capture.value_us < capture.uncertainty_us ||
        capture.uncertainty_us > kMaxWireUint64 - capture.value_us) {
        return BridgeCode::InvalidTime;
    }
    const std::uint64_t raw_low = capture.value_us - capture.uncertainty_us;
    const std::uint64_t raw_high = capture.value_us + capture.uncertainty_us;
    if (raw_low < map.valid_from_source_us || raw_high > map.valid_until_source_us) {
        return BridgeCode::MapExpired;
    }
    std::uint64_t low = 0, high = 0, nominal = 0;
    if (!map_bound(map, raw_low, false, low) || !map_bound(map, raw_high, true, high) ||
        !map_bound(map, capture.value_us, false, nominal)) {
        return BridgeCode::InvalidMap;
    }
    const std::uint64_t capture_distance = std::max(distance_from(raw_low, map.source_anchor_us),
                                                     distance_from(raw_high, map.source_anchor_us));
    const std::uint64_t map_uncertainty = grown_uncertainty(map, capture_distance);
    // Invert a conservative Pi-time interval to the source-time basis used by
    // the declared growth rate. The extra 2 ms bounds every admissible map.
    const std::uint64_t current_product = (distance_from(now_pi_us, map.pi_anchor_us) +
                                            kMaximumMapUncertaintyUs) * map.rate_denominator;
    const std::uint64_t current_source_distance = current_product / map.rate_numerator +
                                                  (current_product % map.rate_numerator != 0 ? 1 : 0);
    if (map_uncertainty > kMaximumMapUncertaintyUs ||
        grown_uncertainty(map, current_source_distance) > kMaximumMapUncertaintyUs) {
        return BridgeCode::MapUncertain;
    }
    if (low < map_uncertainty || low - map_uncertainty < map.valid_from_pi_us ||
        low - map_uncertainty < map.issued_pi_us ||
        map_uncertainty > kMaxWireUint64 - high || high + map_uncertainty > map.valid_until_pi_us) {
        return BridgeCode::MapExpired;
    }
    if (nominal > now_pi_us) {
        return BridgeCode::InvalidTime;
    }
    const std::uint64_t uncertainty = std::max(nominal - low, high - nominal) + map_uncertainty;
    if (uncertainty > std::numeric_limits<std::uint32_t>::max()) {
        return BridgeCode::InvalidMap;
    }
    result.capture_pi_us = nominal;
    result.total_uncertainty_us = static_cast<std::uint32_t>(uncertainty);
    return BridgeCode::Accepted;
}

BridgeCode validate_tracking_pose(const PoseObservation& pose, const PlayerBinding& binding,
                                  const PiClockReference& reference, std::uint64_t now_pi_us,
                                  std::uint64_t maximum_age_us) {
    if (!envelope_valid(pose.envelope) || !frame_shape_valid(pose.source_frame, pose.envelope) ||
        !quality_valid(pose.quality_policy)) {
        return BridgeCode::InvalidPose;
    }
    if (pose.envelope.sender_session_id != binding.tracking_session_id ||
        pose.source_frame.person_id != binding.person_id || pose.association_version != binding.association_version) {
        return BridgeCode::SourceMismatch;
    }
    if (pose.calibration_id != binding.calibration_id || pose.source_frame.calibration_id != binding.calibration_id) {
        return BridgeCode::CalibrationMismatch;
    }
    if (pose.frame_id != pose.source_frame.frame_id || pose.received_pi_us > now_pi_us ||
        pose.received_pi_us < pose.capture_pi_us) {
        return BridgeCode::ObservationMismatch;
    }
    MappedCapture mapped;
    const BridgeCode code = map_tracking_capture(pose.mapping, pose.source_frame.capture,
                                                envelope_source(pose.envelope), reference, now_pi_us, mapped);
    if (code != BridgeCode::Accepted) {
        return code;
    }
    if (mapped.capture_pi_us != pose.capture_pi_us || mapped.total_uncertainty_us != pose.total_uncertainty_us) {
        return BridgeCode::ObservationMismatch;
    }
    for (const std::uint64_t boundary : {pose.source_frame.processing_complete_us, pose.envelope.sent_time_us}) {
        const BridgeCode event_code = validate_native_event(pose.mapping, boundary, pose.received_pi_us);
        if (event_code != BridgeCode::Accepted) {
            return event_code;
        }
    }
    const std::uint64_t age = now_pi_us - pose.capture_pi_us;
    if (age > maximum_age_us || pose.total_uncertainty_us > maximum_age_us - age) {
        return BridgeCode::StalePose;
    }
    if (!pose.tracked || !required_joints_usable(pose.source_frame, binding, pose.quality_policy)) {
        return BridgeCode::RequiredJointsUnavailable;
    }
    return BridgeCode::Accepted;
}

void TrackingPoseBridge::invalidate() {
    counters_.invalidated_poses += poses_.size();
    poses_.clear();
}
void TrackingPoseBridge::diagnose(BridgeCode code, std::uint64_t now_pi_us, std::uint64_t frame_id,
                                 const PoseObservation* pose, bool observed_time_valid) {
    BridgeDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.source = source_;
    diagnostic.reference = reference_;
    diagnostic.observed_pi_us = now_pi_us;
    diagnostic.observed_time_valid = observed_time_valid;
    diagnostic.frame_id = frame_id;
    if (pose != nullptr) {
        diagnostic.person_id = pose->source_frame.person_id;
        diagnostic.raw_capture = pose->source_frame.capture;
        diagnostic.mapping_id = pose->mapping.mapping_id;
        diagnostic.mapping_version = pose->mapping.version;
        diagnostic.capture_pi_us = pose->capture_pi_us;
        diagnostic.total_uncertainty_us = pose->total_uncertainty_us;
    }
    if (!diagnostics_.push_back(diagnostic)) {
        ++counters_.diagnostics_lost;
    }
}
BridgeCode TrackingPoseBridge::reject(BridgeCode code, std::uint64_t now_pi_us,
                                     std::uint64_t frame_id, bool rejected_frame) {
    invalidate();
    ++counters_.rejections[static_cast<std::size_t>(code)];
    if (rejected_frame) {
        ++counters_.frames_rejected;
    }
    last_result_ = code;
    diagnose(code, now_pi_us, frame_id, nullptr, rejected_frame && code != BridgeCode::InvalidTime);
    return code;
}

BridgeCode TrackingPoseBridge::start_session(const TrackingSourceSession& source,
                                            const PiClockReference& reference,
                                            const TrackingQualityPolicy& quality) {
    if (!source_valid(source) || !reference_valid(reference) || !quality_valid(quality)) {
        return reject(BridgeCode::InvalidSession);
    }
    if (configured_ && source == source_ && reference == reference_ && quality == quality_) {
        return BridgeCode::Accepted;
    }
    if (configured_ && source.session_id == source_.session_id && source != source_) {
        return reject(BridgeCode::InvalidSession);
    }
    if (std::find(retired_sessions_.begin(), retired_sessions_.end(), source.session_id) != retired_sessions_.end() ||
        std::find(retired_references_.begin(), retired_references_.end(), reference) != retired_references_.end()) {
        return reject(BridgeCode::RetiredSession);
    }
    const bool new_source = configured_ && source != source_;
    const bool new_reference = configured_ && reference != reference_;
    if ((new_source && retired_sessions_.full()) || (new_reference && retired_references_.full())) {
        return reject(BridgeCode::Capacity);
    }
    invalidate();
    if (new_source) {
        retired_sessions_.push_back(source_.session_id);
        ++counters_.session_restarts;
    }
    if (new_reference) {
        retired_references_.push_back(reference_);
        ++counters_.reference_restarts;
    }
    if (!configured_ || new_reference) {
        last_now_pi_us_ = 0;
    }
    source_ = source;
    reference_ = reference;
    quality_ = quality;
    bindings_.clear();
    maps_.clear();
    if (!configured_ || new_source) {
        last_frame_id_ = last_accepted_frame_id_ = last_message_seq_ = last_source_drops_ = 0;
    }
    configured_ = true;
    last_result_ = BridgeCode::Accepted;
    diagnose(BridgeCode::Accepted, 0, 0);
    return BridgeCode::Accepted;
}

BridgeCode TrackingPoseBridge::install_confirmed_bindings(const BoundedVector<PlayerBinding, kPlayers>& bindings) {
    if (!configured_) {
        return reject(BridgeCode::NotConfigured);
    }
    if (bindings.empty()) {
        return reject(BridgeCode::InvalidBinding);
    }
    for (std::size_t index = 0; index < bindings.size(); ++index) {
        const PlayerBinding& binding = bindings[index];
        if (binding.player_slot < 1 || binding.player_slot > kPlayers || binding.controller_id == 0 ||
            !uuid_valid(binding.controller_session_id) || binding.tracking_session_id != source_.session_id ||
            binding.person_id == 0 || binding.avatar_id == 0 || binding.association_version == 0 || binding.calibration_id == 0) {
            return reject(BridgeCode::InvalidBinding);
        }
        for (std::size_t previous = 0; previous < index; ++previous) {
            const PlayerBinding& other = bindings[previous];
            if (other.player_slot == binding.player_slot || other.person_id == binding.person_id ||
                other.controller_id == binding.controller_id || other.avatar_id == binding.avatar_id) {
                return reject(BridgeCode::InvalidBinding);
            }
        }
    }
    invalidate();
    bindings_ = bindings;
    last_result_ = BridgeCode::Accepted;
    return BridgeCode::Accepted;
}

BridgeCode TrackingPoseBridge::install_clock_map(const TrackingClockMap& map) {
    if (!configured_) {
        return reject(BridgeCode::NotConfigured);
    }
    if (!map_structure_valid(map)) {
        return reject(BridgeCode::InvalidMap);
    }
    if (map.source != source_) {
        return reject(BridgeCode::SourceMismatch);
    }
    if (map.reference != reference_) {
        return reject(BridgeCode::ReferenceMismatch);
    }
    for (std::size_t index = 0; index < maps_.size(); ++index) {
        if (maps_[index].mapping_id == map.mapping_id) {
            if (maps_[index] != map) {
                return reject(BridgeCode::MapConflict);
            }
            active_map_ = index;
            return BridgeCode::Accepted;
        }
    }
    if (!maps_.push_back(map)) {
        return reject(BridgeCode::Capacity);
    }
    active_map_ = maps_.size() - 1;
    last_result_ = BridgeCode::Accepted;
    return BridgeCode::Accepted;
}

const PlayerBinding* TrackingPoseBridge::binding_for(std::uint32_t person_id) const {
    const auto found = std::find_if(bindings_.begin(), bindings_.end(),
                                     [person_id](const PlayerBinding& value) { return value.person_id == person_id; });
    return found == bindings_.end() ? nullptr : found;
}

BridgeCode TrackingPoseBridge::ingest_frame(const TrackingFrameBatch& batch, std::uint64_t now_pi_us) {
    ++counters_.frames_received;
    const std::uint64_t frame_id = batch.status.frame_id;
    if (!configured_) {
        return reject(BridgeCode::NotConfigured, now_pi_us, frame_id, true);
    }
    if (!wire(now_pi_us) || now_pi_us < last_now_pi_us_) {
        return reject(BridgeCode::InvalidTime, now_pi_us, frame_id, true);
    }
    last_now_pi_us_ = now_pi_us;
    if (!envelope_valid(batch.status_envelope)) {
        return reject(BridgeCode::InvalidEnvelope, now_pi_us, frame_id, true);
    }
    if (envelope_source(batch.status_envelope) != source_) {
        return reject(BridgeCode::SourceMismatch, now_pi_us, frame_id, true);
    }
    if (frame_id == 0 || !wire(frame_id) || !wire(batch.status.frames_dropped_total)) {
        return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
    }
    if (frame_id == last_frame_id_) {
        return reject(BridgeCode::DuplicateFrame, now_pi_us, frame_id, true);
    }
    if (frame_id < last_frame_id_) {
        return reject(BridgeCode::ReorderedFrame, now_pi_us, frame_id, true);
    }
    // Even a rejected newer frame fences its identity; an old cached pose or a
    // later contradictory reconstruction of that frame cannot become eligible.
    last_frame_id_ = frame_id;
    if (batch.status_envelope.message_seq <= last_message_seq_) {
        return reject(BridgeCode::InvalidEnvelope, now_pi_us, frame_id, true);
    }
    const std::uint64_t previous_message_seq = last_message_seq_;
    last_message_seq_ = batch.status_envelope.message_seq;
    if (batch.status.person_ids.size() != batch.people.size()) {
        return reject(BridgeCode::IncompleteFrame, now_pi_us, frame_id, true);
    }
    if (!batch.status.camera_ready && !batch.people.empty()) {
        return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
    }
    if (batch.status.frames_dropped_total < last_source_drops_) {
        return reject(BridgeCode::SourceLossRegression, now_pi_us, frame_id, true);
    }
    std::array<bool, kPlayers> matched{};
    for (std::size_t index = 0; index < batch.status.person_ids.size(); ++index) {
        if (batch.status.person_ids[index] == 0) {
            return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
        }
        for (std::size_t other = 0; other < index; ++other) {
            if (batch.status.person_ids[index] == batch.status.person_ids[other]) {
                return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
            }
        }
    }
    for (std::size_t index = 0; index < batch.people.size(); ++index) {
        const TrackingPersonPacket& packet = batch.people[index];
        if (!envelope_valid(packet.envelope) || envelope_source(packet.envelope) != source_ ||
            packet.envelope.message_seq <= previous_message_seq ||
            packet.envelope.message_seq >= batch.status_envelope.message_seq ||
            packet.envelope.sent_time_us > batch.status_envelope.sent_time_us) {
            return reject(BridgeCode::InvalidEnvelope, now_pi_us, frame_id, true);
        }
        for (std::size_t other = 0; other < index; ++other) {
            if (packet.envelope.message_seq == batch.people[other].envelope.message_seq ||
                packet.frame.person_id == batch.people[other].frame.person_id) {
                return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
            }
        }
        if (packet.frame.frame_id != frame_id || !frame_shape_valid(packet.frame, packet.envelope)) {
            return reject(BridgeCode::InvalidPose, now_pi_us, frame_id, true);
        }
        const auto named = std::find(batch.status.person_ids.begin(), batch.status.person_ids.end(), packet.frame.person_id);
        if (named == batch.status.person_ids.end()) {
            return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
        }
        matched[static_cast<std::size_t>(named - batch.status.person_ids.begin())] = true;
        if (index > 0) {
            const SkeletonFrame& first = batch.people[0].frame;
            if (packet.frame.capture != first.capture || packet.frame.calibration_id != first.calibration_id ||
                packet.frame.image_width != first.image_width || packet.frame.image_height != first.image_height ||
                packet.frame.processing_complete_us != first.processing_complete_us) {
                return reject(BridgeCode::ContradictoryFrame, now_pi_us, frame_id, true);
            }
        }
    }
    for (std::size_t index = 0; index < batch.status.person_ids.size(); ++index) {
        if (!matched[index]) {
            return reject(BridgeCode::IncompleteFrame, now_pi_us, frame_id, true);
        }
    }
    BoundedVector<PoseObservation, kPlayers> staged;
    if (!batch.people.empty() && bindings_.empty()) {
        return reject(BridgeCode::MissingBindings, now_pi_us, frame_id, true);
    }
    if (!batch.people.empty() && maps_.empty()) {
        return reject(BridgeCode::MissingMap, now_pi_us, frame_id, true);
    }
    if (!maps_.empty()) {
        const BridgeCode status_time = validate_native_event(maps_[active_map_], batch.status_envelope.sent_time_us,
                                                             now_pi_us);
        if (status_time != BridgeCode::Accepted) {
            return reject(status_time, now_pi_us, frame_id, true);
        }
    }
    for (const TrackingPersonPacket& packet : batch.people) {
        MappedCapture mapped;
        BridgeCode code = map_tracking_capture(maps_[active_map_], packet.frame.capture, source_, reference_, now_pi_us, mapped);
        if (code != BridgeCode::Accepted) {
            return reject(code, now_pi_us, frame_id, true);
        }
        const PlayerBinding* binding = binding_for(packet.frame.person_id);
        if (binding == nullptr) {
            ++counters_.unbound_people;
            continue;
        }
        PoseObservation pose;
        pose.frame_id = packet.frame.frame_id;
        pose.calibration_id = packet.frame.calibration_id;
        pose.tracked = required_joints_usable(packet.frame, *binding, quality_);
        pose.association_version = binding->association_version;
        pose.envelope = packet.envelope;
        pose.source_frame = packet.frame;
        pose.mapping = maps_[active_map_];
        pose.quality_policy = quality_;
        pose.received_pi_us = now_pi_us;
        pose.capture_pi_us = mapped.capture_pi_us;
        pose.total_uncertainty_us = mapped.total_uncertainty_us;
        code = validate_tracking_pose(pose, *binding, reference_, now_pi_us);
        if (code == BridgeCode::RequiredJointsUnavailable) {
            ++counters_.unusable_poses;
        } else if (code != BridgeCode::Accepted) {
            return reject(code, now_pi_us, frame_id, true);
        }
        staged.push_back(pose);
        diagnose(code, now_pi_us, frame_id, &pose, true);
    }
    invalidate();
    poses_ = staged;
    counters_.source_frames_dropped_total += batch.status.frames_dropped_total - last_source_drops_;
    last_source_drops_ = batch.status.frames_dropped_total;
    if (last_accepted_frame_id_ != 0) {
        counters_.frame_sequence_gaps += frame_id - last_accepted_frame_id_ - 1;
    }
    last_accepted_frame_id_ = frame_id;
    ++counters_.frames_accepted;
    last_result_ = BridgeCode::Accepted;
    diagnose(BridgeCode::Accepted, now_pi_us, frame_id, nullptr, true);
    return BridgeCode::Accepted;
}

std::optional<PoseObservation> TrackingPoseBridge::latest_for(std::uint32_t person_id) const {
    const auto found = std::find_if(poses_.begin(), poses_.end(),
                                     [person_id](const PoseObservation& value) { return value.source_frame.person_id == person_id; });
    return found == poses_.end() ? std::nullopt : std::optional<PoseObservation>(*found);
}
BoundedVector<BridgeDiagnostic, kTrackingDiagnosticCapacity> TrackingPoseBridge::take_diagnostics() {
    const auto result = diagnostics_;
    diagnostics_.clear();
    return result;
}

}  // namespace motion_tennis::physics
