#include "motion_tennis/physics/simulation.hpp"

#include <cmath>

namespace motion_tennis::physics {
namespace {

float tick_seconds(const SimulationParameters& parameters) {
    return static_cast<float>(parameters.tick_duration_us) * 1.0e-6f;
}

}  // namespace

void integrate_ball_tick(Ball& ball, const SimulationParameters& parameters) {
    const float dt = tick_seconds(parameters);
    Vector3& velocity = ball.velocity_m_s;
    Vector3& position = ball.position_m;

    // Semi-implicit Euler: update velocity first, then position.
    velocity.y -= parameters.gravity_m_s2 * dt;
    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    // Swept ground contact: a step that ends below the contact height reflects
    // the penetration instead of letting the ball tunnel through the court.
    const float contact_height = ball.radius_m;
    if (position.y < contact_height && velocity.y < 0.0f) {
        const float penetration = contact_height - position.y;
        position.y = contact_height + penetration * parameters.ground_restitution;
        velocity.y = -velocity.y * parameters.ground_restitution;
    }
}

void integrate_avatar_tick(AvatarState& avatar, const SimulationParameters& parameters) {
    const float dt = tick_seconds(parameters);
    const float dx = avatar.target_m.x - avatar.position_m.x;
    const float dz = avatar.target_m.z - avatar.position_m.z;
    const float distance = std::sqrt(dx * dx + dz * dz);
    const float max_step = parameters.avatar_max_speed_m_s * dt;

    if (distance == 0.0f) {
        avatar.velocity_m_s = Vector3{};
        return;
    }
    if (distance <= max_step) {
        avatar.position_m.x = avatar.target_m.x;
        avatar.position_m.z = avatar.target_m.z;
        avatar.velocity_m_s = Vector3{dx / dt, 0.0f, dz / dt};
        return;
    }
    const float ux = dx / distance;
    const float uz = dz / distance;
    avatar.position_m.x += ux * max_step;
    avatar.position_m.z += uz * max_step;
    avatar.velocity_m_s = Vector3{ux * parameters.avatar_max_speed_m_s, 0.0f,
                                  uz * parameters.avatar_max_speed_m_s};
}

void advance_match(MatchState& match, std::uint32_t ticks, const SimulationParameters& parameters) {
    for (std::uint32_t tick = 0; tick < ticks; ++tick) {
        integrate_ball_tick(match.ball, parameters);
        for (AvatarState& avatar : match.avatars) {
            integrate_avatar_tick(avatar, parameters);
        }
        match.state_time_pi_us += parameters.tick_duration_us;
    }
}

Vector3 cap_speed(const Vector3& velocity, float max_speed) {
    const float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                                  velocity.z * velocity.z);
    if (speed <= max_speed || speed == 0.0f) {
        return velocity;
    }
    const float scale = max_speed / speed;
    return Vector3{velocity.x * scale, velocity.y * scale, velocity.z * scale};
}

}  // namespace motion_tennis::physics
