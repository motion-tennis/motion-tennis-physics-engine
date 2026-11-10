#pragma once

#include <cstdint>
#include <optional>

#include "motion_tennis/physics/contract.hpp"
#include "motion_tennis/physics/input_scheduler.hpp"

// Adapter boundaries of the batch core. S1 supplies fakes only; real UDP pose
// input, sensor fusion and the TCP state stream implement these later.
namespace motion_tennis::physics {

// Latest body-local pose evidence per tracked person, already in Pi time.
class PoseSource {
public:
    virtual ~PoseSource() = default;
    virtual std::optional<PoseObservation> latest_for(std::uint32_t person_id) const = 0;
};

struct ContactResult {
    Outcome outcome = Outcome::Miss;  // Only Hit or Miss; failures are decided by the core.
    Vector3 outgoing_velocity_m_s;
};

// Evaluates one eligible input against the current candidate state. It must be
// a pure function of its arguments: a rebuild after a late input rejection
// replays the same calls and must reproduce the same results.
class ContactEvaluator {
public:
    virtual ~ContactEvaluator() = default;
    virtual ContactResult evaluate(const MatchState& candidate, const PendingInput& input,
                                   const PoseObservation& pose, const PlayerBinding& binding) const = 0;
};

// Required Pi/PC exchange. A missing commit reply is represented by nullopt so
// the core's resend and ambiguity handling can be exercised.
class StateService {
public:
    virtual ~StateService() = default;
    virtual BallStateResponse query(const BallStateRequest& request) = 0;
    virtual std::optional<StateCommitResult> commit(const StateUpdate& update) = 0;
};

}  // namespace motion_tennis::physics
