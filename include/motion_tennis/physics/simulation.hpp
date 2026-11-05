#pragma once

#include <cstdint>

#include "motion_tennis/physics/contract.hpp"

namespace motion_tennis::physics {

// Proposed demonstrator parameters from the shared physics design and the
// webcam-pi3-doubles-v3 profile. They are versioned assumptions, not measured
// equipment or tuning results.
struct SimulationParameters {
    std::uint32_t tick_duration_us = 5000;
    float gravity_m_s2 = 9.81f;
    float ground_restitution = 0.75f;
    float max_outgoing_speed_m_s = 20.0f;
    // Placeholder bound for the minimal avatar step; the real planner and its
    // tuning hash are S4 work.
    float avatar_max_speed_m_s = 5.0f;
    std::uint32_t max_catchup_ticks = 4;
    std::uint64_t max_imu_age_us = 30000;
    std::uint64_t max_pose_age_us = 80000;
};

// One fixed 5 ms semi-implicit Euler step with gravity and a flat ground plane
// at y = 0 (court frame: +Y up). No drag, spin, net or racket geometry yet.
void integrate_ball_tick(Ball& ball, const SimulationParameters& parameters);

// Moves an avatar root toward its committed target in the court plane with a
// bounded speed. Deterministic and teleport-free; not the doubles planner.
void integrate_avatar_tick(AvatarState& avatar, const SimulationParameters& parameters);

// Advances the complete candidate match by whole ticks: ball, avatars and Pi
// state time. Rules, serve and score are left untouched in S1.
void advance_match(MatchState& match, std::uint32_t ticks, const SimulationParameters& parameters);

// Caps a speed vector at the configured outgoing racket speed.
Vector3 cap_speed(const Vector3& velocity, float max_speed);

}  // namespace motion_tennis::physics
