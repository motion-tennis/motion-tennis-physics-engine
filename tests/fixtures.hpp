#pragma once

#include <cstdint>

#include "motion_tennis/physics/contract.hpp"

// Synthetic S1 fixtures mirroring documentation/tests/match/fixtures.py
// (build_setup/build_snapshot). They are inspectable test data, not computed
// gameplay or hardware results.
namespace motion_tennis::physics::fixtures {

inline constexpr std::uint64_t kInitialTick = 100;
inline constexpr std::uint64_t kInitialPiTimeUs = 500000;

inline Uuid filled_uuid(std::uint8_t value) {
    Uuid id{};
    id.fill(value);
    return id;
}

inline Sha256 filled_sha(std::uint8_t value) {
    Sha256 hash{};
    hash.fill(value);
    return hash;
}

inline std::uint32_t controller_for_slot(std::uint32_t slot) { return 100 + slot; }
inline std::uint32_t person_for_slot(std::uint32_t slot) { return 200 + slot; }

inline MatchSetup build_setup(std::uint8_t session_byte = 'G', std::uint64_t writer_epoch = 1) {
    MatchSetup setup;
    setup.command_id = 1;
    setup.game_session_id = filled_uuid(session_byte);
    setup.writer_epoch = writer_epoch;
    MatchConfiguration& configuration = setup.configuration;
    configuration.roster_revision = 1;
    configuration.rules_revision = 1;
    for (std::uint32_t slot = 1; slot <= 4; ++slot) {
        PlayerBinding& player = configuration.players[slot - 1];
        player.player_slot = slot;
        player.controller_id = controller_for_slot(slot);
        player.controller_session_id = filled_uuid(static_cast<std::uint8_t>(slot));
        player.tracking_session_id = filled_uuid('T');
        player.person_id = person_for_slot(slot);
        player.avatar_id = slot;
        player.association_version = 1;
        player.calibration_id = 1;
        player.right_hand = true;
        player.physical_station_id = slot;
    }
    configuration.rules = MatchRules{2, 6, 2, 6, 7, 2, true, filled_sha('U')};
    // Placeholder: the real digest is SHA-256 over the deterministic Protobuf
    // encoding and needs the generated codec, which is outside this slice.
    setup.configuration_sha256 = filled_sha('C');
    return setup;
}

inline Snapshot build_snapshot(const MatchSetup& setup) {
    Snapshot snapshot;
    snapshot.game_session_id = setup.game_session_id;
    snapshot.state_version = 1;
    snapshot.simulation_tick = kInitialTick;
    snapshot.tick_duration_us = 5000;
    MatchState& state = snapshot.match;
    state.ball = Ball{Vector3{1.0f, 1.0f, 1.0f}, Vector3{}, 0.033f, 1};
    state.roster_revision = setup.configuration.roster_revision;
    state.rules_revision = setup.configuration.rules_revision;
    state.configuration_sha256 = setup.configuration_sha256;
    for (std::uint32_t slot = 1; slot <= 4; ++slot) {
        AvatarState& avatar = state.avatars[slot - 1];
        avatar.player_slot = slot;
        avatar.position_m = Vector3{static_cast<float>(slot), 0.0f, 2.0f};
        avatar.target_m = avatar.position_m;
        avatar.movement = MovementMode::Holding;
        avatar.coverage_owner_slot = slot;
    }
    state.serve.phase = ServePhase::Ready;
    state.serve.server_slot = 1;
    state.serve.receiver_slot = 3;
    state.serve.side = ServiceSide::Deuce;
    state.serve.attempt = 1;
    state.serve.service_order = {1, 3, 2, 4};
    state.serve.receiving_order = {1, 2, 3, 4};
    state.rally.rally_id = 1;
    state.team_a_near_end = true;
    state.phase = MatchPhase::Ready;
    state.point_id = 1;
    state.state_time_pi_us = kInitialPiTimeUs;
    state.pi_reference_session_id = filled_uuid('P');
    state.pi_reference_clock_id = 1;
    return snapshot;
}

inline ControllerInput input_for(std::uint32_t slot, std::uint64_t request_seq, std::uint64_t capture_pi_us) {
    ControllerInput input;
    input.transaction.controller_id = controller_for_slot(slot);
    input.transaction.session_id = filled_uuid(static_cast<std::uint8_t>(slot));
    input.transaction.request_seq = request_seq;
    input.t0_us = 1000 + request_seq;
    input.sample_id = slot * 1000 + request_seq;
    input.capture_pi_us = capture_pi_us;
    input.clock_mapping_version = 1;
    input.association_version = 1;
    input.calibration_id = 1;
    return input;
}

inline PoseObservation tracked_pose(std::uint64_t frame_id, std::uint64_t capture_pi_us) {
    return PoseObservation{frame_id, capture_pi_us, 1, true};
}

}  // namespace motion_tennis::physics::fixtures
