#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "motion_tennis/physics/bounded_vector.hpp"
#include "motion_tennis/physics/contract.hpp"

namespace motion_tennis::physics {

inline constexpr std::size_t kQueuePerController = 4;

struct PendingInput {
    ControllerInput input;
    // Pi-local receipt order; source timestamps never decide physical order.
    std::uint64_t arrival_seq = 0;
    std::uint64_t arrival_pi_us = 0;
    std::uint32_t player_slot = 0;
    friend bool operator==(const PendingInput&, const PendingInput&) = default;
};

enum class AdmitResult {
    Queued,
    UnknownController,
    DuplicateTransaction,
    QueueFull,
};

struct SchedulerCounters {
    std::uint64_t admitted = 0;
    std::uint64_t rejected_unknown = 0;
    std::uint64_t rejected_duplicate = 0;
    std::uint64_t rejected_overflow = 0;
    std::uint64_t releases = 0;
    std::uint64_t selected = 0;
};

// Bounded per-controller queues (four each) and the deterministic batch
// selection rule: rotate the starting slot every release, take at most two
// entries per controller and eight in total, then order by Pi arrival.
class InputScheduler {
public:
    explicit InputScheduler(const MatchConfiguration& configuration);

    AdmitResult admit(const ControllerInput& input, std::uint64_t arrival_pi_us);
    BoundedVector<PendingInput, kMaxBatchQueries> select_batch();

    std::size_t pending_for_slot(std::uint32_t slot) const;
    std::size_t total_pending() const;
    const SchedulerCounters& counters() const { return counters_; }

private:
    struct Ring {
        std::array<PendingInput, kQueuePerController> items{};
        std::size_t head = 0;
        std::size_t count = 0;
    };

    int slot_index_for(const Transaction& transaction) const;
    bool is_queued(const Transaction& transaction) const;

    std::array<PlayerBinding, kPlayers> bindings_;
    std::array<Ring, kPlayers> queues_{};
    std::uint64_t next_arrival_seq_ = 1;
    std::uint64_t release_count_ = 0;
    SchedulerCounters counters_;
};

}  // namespace motion_tennis::physics
