#pragma once

#include <array>
#include <charconv>
#include <concepts>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "motion_tennis/physics/contract.hpp"

// Test-only JSON projection of the real core's typed output. The shared
// harness parses these field names and values with the generated v0.3 codec.
namespace motion_tennis::physics::fixtures::proto_json {

inline std::string object(std::initializer_list<std::pair<std::string_view, std::string>> fields) {
    std::string result = "{";
    for (const auto& [name, value] : fields) {
        if (result.size() > 1) result += ',';
        result += '"';
        result += name;
        result += "\":";
        result += value;
    }
    return result + '}';
}

template<std::integral T>
std::string json(T value) { return std::to_string(value); }

template<class T> requires std::is_enum_v<T>
std::string json(T value) { return json(static_cast<std::underlying_type_t<T>>(value)); }

inline std::string json(bool value) { return value ? "true" : "false"; }

inline std::string json(float value) {
    std::array<char, 48> buffer{};
    auto [end, error] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                                      std::chars_format::general, std::numeric_limits<float>::max_digits10);
    if (error != std::errc{}) throw std::runtime_error("Could not format fixture float");
    return std::string(buffer.data(), end);
}

template<std::size_t N>
std::string json(const std::array<std::uint8_t, N>& bytes) {
    constexpr std::string_view alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded = "\"";
    for (std::size_t i = 0; i < bytes.size(); i += 3) {
        const std::uint32_t a = bytes[i];
        const std::uint32_t b = i + 1 < bytes.size() ? bytes[i + 1] : 0u;
        const std::uint32_t c = i + 2 < bytes.size() ? bytes[i + 2] : 0u;
        encoded += alphabet[a >> 2];
        encoded += alphabet[((a & 3u) << 4) | (b >> 4)];
        encoded += i + 1 < bytes.size() ? alphabet[((b & 15u) << 2) | (c >> 6)] : '=';
        encoded += i + 2 < bytes.size() ? alphabet[c & 63u] : '=';
    }
    return encoded + '"';
}

template<class Range>
std::string array(const Range& values);

// A field has the same spelling in the typed fixture and the accepted schema.
#define FIXTURE_FIELD(member) {#member, json(v.member)}
#define FIXTURE_LIST(member) {#member, array(v.member)}

inline std::string json(const PlayerBinding& v) {
    return object({FIXTURE_FIELD(player_slot), FIXTURE_FIELD(controller_id), FIXTURE_FIELD(controller_session_id),
                   FIXTURE_FIELD(tracking_session_id), FIXTURE_FIELD(person_id), FIXTURE_FIELD(avatar_id),
                   FIXTURE_FIELD(association_version), FIXTURE_FIELD(calibration_id), FIXTURE_FIELD(right_hand),
                   FIXTURE_FIELD(physical_station_id)});
}

inline std::string json(const MatchRules& v) {
    return object({FIXTURE_FIELD(sets_to_win), FIXTURE_FIELD(games_to_win_set), FIXTURE_FIELD(minimum_game_lead),
                   FIXTURE_FIELD(tiebreak_at_games), FIXTURE_FIELD(tiebreak_points_to_win),
                   FIXTURE_FIELD(tiebreak_minimum_lead), FIXTURE_FIELD(final_set_tiebreak), FIXTURE_FIELD(tuning_sha256)});
}

inline std::string json(const MatchConfiguration& v) {
    return object({FIXTURE_FIELD(roster_revision), FIXTURE_FIELD(rules_revision), FIXTURE_LIST(players), FIXTURE_FIELD(rules)});
}

inline std::string json(const Transaction& v) {
    return object({FIXTURE_FIELD(controller_id), FIXTURE_FIELD(session_id), FIXTURE_FIELD(request_seq)});
}

inline std::string json(const QueryEntry& v) {
    return object({FIXTURE_FIELD(query_id), FIXTURE_FIELD(transaction)});
}

inline std::string json(const BallStateRequest& v) {
    return object({FIXTURE_FIELD(game_session_id), FIXTURE_FIELD(writer_epoch), FIXTURE_FIELD(batch_id), FIXTURE_LIST(queries)});
}

inline std::string json(const Vector3& v) {
    return object({FIXTURE_FIELD(x), FIXTURE_FIELD(y), FIXTURE_FIELD(z)});
}

inline std::string json(const Ball& v) {
    return object({FIXTURE_FIELD(position_m), FIXTURE_FIELD(velocity_m_s), FIXTURE_FIELD(radius_m), FIXTURE_FIELD(calibration_id)});
}

inline std::string json(const AvatarState& v) {
    return object({FIXTURE_FIELD(player_slot), FIXTURE_FIELD(position_m), FIXTURE_FIELD(velocity_m_s),
                   FIXTURE_FIELD(heading_rad), FIXTURE_FIELD(target_m), FIXTURE_FIELD(movement), FIXTURE_FIELD(coverage_owner_slot)});
}

inline std::string json(const ServeState& v) {
    return object({FIXTURE_FIELD(phase), FIXTURE_FIELD(server_slot), FIXTURE_FIELD(receiver_slot), FIXTURE_FIELD(side),
                   FIXTURE_FIELD(attempt), FIXTURE_FIELD(stroke_attempted), FIXTURE_FIELD(net_touched), FIXTURE_FIELD(toss_id),
                   FIXTURE_LIST(service_order), FIXTURE_LIST(receiving_order), FIXTURE_FIELD(service_turn_index),
                   FIXTURE_FIELD(tiebreak_first_server_slot)});
}

inline std::string json(const RallyState& v) {
    return object({FIXTURE_FIELD(rally_id), FIXTURE_FIELD(last_hitter_slot), FIXTURE_FIELD(last_bounce_team),
                   FIXTURE_FIELD(bounce_count), FIXTURE_FIELD(net_touched), FIXTURE_FIELD(service_return_pending),
                   FIXTURE_FIELD(contact_slot_mask), FIXTURE_FIELD(contact_id), FIXTURE_FIELD(bounce_id), FIXTURE_FIELD(point_winner)});
}

inline std::string json(const CompletedSet& v) {
    return object({FIXTURE_FIELD(games_a), FIXTURE_FIELD(games_b), FIXTURE_FIELD(had_tiebreak),
                   FIXTURE_FIELD(tiebreak_points_a), FIXTURE_FIELD(tiebreak_points_b)});
}

inline std::string json(const ScoreState& v) {
    return object({FIXTURE_FIELD(points_a), FIXTURE_FIELD(points_b), FIXTURE_FIELD(games_a), FIXTURE_FIELD(games_b),
                   FIXTURE_FIELD(sets_a), FIXTURE_FIELD(sets_b), FIXTURE_FIELD(tiebreak), FIXTURE_FIELD(tiebreak_points_a),
                   FIXTURE_FIELD(tiebreak_points_b), FIXTURE_LIST(completed_sets)});
}

inline std::string json(const MatchState& v) {
    return object({FIXTURE_FIELD(ball), FIXTURE_FIELD(roster_revision), FIXTURE_FIELD(rules_revision),
                   FIXTURE_FIELD(configuration_sha256), FIXTURE_LIST(avatars), FIXTURE_FIELD(serve), FIXTURE_FIELD(rally),
                   FIXTURE_FIELD(score), FIXTURE_FIELD(team_a_near_end), FIXTURE_FIELD(phase), FIXTURE_FIELD(winner),
                   FIXTURE_FIELD(point_id), FIXTURE_FIELD(state_time_pi_us), FIXTURE_FIELD(pi_reference_session_id),
                   FIXTURE_FIELD(pi_reference_clock_id)});
}

inline std::string json(const Snapshot& v) {
    return object({FIXTURE_FIELD(game_session_id), FIXTURE_FIELD(state_version), FIXTURE_FIELD(simulation_tick),
                   FIXTURE_FIELD(tick_duration_us), FIXTURE_FIELD(match)});
}

inline std::string json(const Cue& v) {
    return object({FIXTURE_FIELD(kind), FIXTURE_FIELD(cue_id), FIXTURE_FIELD(intensity), FIXTURE_FIELD(duration_ms),
                   FIXTURE_FIELD(electrical_eligible)});
}

inline std::string json(const Decision& v) {
    return object({FIXTURE_FIELD(transaction), FIXTURE_FIELD(current_sample_id), FIXTURE_FIELD(pose_frame_id),
                   FIXTURE_FIELD(person_id), FIXTURE_FIELD(association_version), FIXTURE_FIELD(imu_capture_us),
                   FIXTURE_FIELD(pose_capture_us), FIXTURE_FIELD(outcome), FIXTURE_FIELD(event_index),
                   FIXTURE_FIELD(swing_id), FIXTURE_FIELD(cue)});
}

inline std::string json(const RuleEvent& v) {
    return object({FIXTURE_FIELD(event_index), FIXTURE_FIELD(kind), FIXTURE_FIELD(point_id), FIXTURE_FIELD(player_slot),
                   FIXTURE_FIELD(team), FIXTURE_FIELD(has_cause), FIXTURE_FIELD(cause_decision_index),
                   FIXTURE_FIELD(simulation_tick), FIXTURE_FIELD(serve_attempt)});
}

inline std::string json(const StateUpdate& v) {
    return object({FIXTURE_FIELD(game_session_id), FIXTURE_FIELD(writer_epoch), FIXTURE_FIELD(batch_id),
                   FIXTURE_FIELD(base_state_version), FIXTURE_FIELD(target_simulation_tick), FIXTURE_FIELD(tracking_session_id),
                   FIXTURE_LIST(decisions), FIXTURE_FIELD(match), FIXTURE_LIST(rule_events)});
}

#undef FIXTURE_FIELD
#undef FIXTURE_LIST

template<class Range>
std::string array(const Range& values) {
    std::string result = "[";
    for (const auto& value : values) {
        if (result.size() > 1) result += ',';
        result += json(value);
    }
    return result + ']';
}

}  // namespace motion_tennis::physics::fixtures::proto_json
