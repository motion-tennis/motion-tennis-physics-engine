#include "motion_tennis/physics/input_scheduler.hpp"

#include <tuple>

namespace motion_tennis::physics {
namespace {

// Pi arrival sequence first; controller/session/request identities break ties.
bool ordered_before(const PendingInput& left, const PendingInput& right) {
    const Transaction& a = left.input.transaction;
    const Transaction& b = right.input.transaction;
    return std::tie(left.arrival_seq, a.controller_id, a.session_id, a.request_seq) <
           std::tie(right.arrival_seq, b.controller_id, b.session_id, b.request_seq);
}

}  // namespace

InputScheduler::InputScheduler(const MatchConfiguration& configuration)
    : bindings_(configuration.players) {}

int InputScheduler::slot_index_for(const Transaction& transaction) const {
    for (std::size_t index = 0; index < bindings_.size(); ++index) {
        const PlayerBinding& binding = bindings_[index];
        if (binding.controller_id == transaction.controller_id &&
            binding.controller_session_id == transaction.session_id) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool InputScheduler::is_queued(const Transaction& transaction) const {
    for (const Ring& ring : queues_) {
        for (std::size_t offset = 0; offset < ring.count; ++offset) {
            const PendingInput& item = ring.items[(ring.head + offset) % kQueuePerController];
            if (item.input.transaction == transaction) {
                return true;
            }
        }
    }
    return false;
}

AdmitResult InputScheduler::admit(const ControllerInput& input, std::uint64_t arrival_pi_us) {
    const int index = slot_index_for(input.transaction);
    if (index < 0 || input.transaction.request_seq == 0) {
        ++counters_.rejected_unknown;
        return AdmitResult::UnknownController;
    }
    if (is_queued(input.transaction)) {
        ++counters_.rejected_duplicate;
        return AdmitResult::DuplicateTransaction;
    }
    Ring& ring = queues_[static_cast<std::size_t>(index)];
    if (ring.count == kQueuePerController) {
        ++counters_.rejected_overflow;
        return AdmitResult::QueueFull;
    }
    PendingInput& slot_item = ring.items[(ring.head + ring.count) % kQueuePerController];
    slot_item.input = input;
    slot_item.arrival_seq = next_arrival_seq_++;
    slot_item.arrival_pi_us = arrival_pi_us;
    slot_item.player_slot = bindings_[static_cast<std::size_t>(index)].player_slot;
    ++ring.count;
    ++counters_.admitted;
    return AdmitResult::Queued;
}

BoundedVector<PendingInput, kMaxBatchQueries> InputScheduler::select_batch() {
    BoundedVector<PendingInput, kMaxBatchQueries> selected;
    const std::size_t start = static_cast<std::size_t>(release_count_ % kPlayers);
    for (std::size_t step = 0; step < kPlayers && !selected.full(); ++step) {
        Ring& ring = queues_[(start + step) % kPlayers];
        for (std::size_t taken = 0;
             taken < kMaxQueriesPerController && ring.count > 0 && !selected.full(); ++taken) {
            selected.push_back(ring.items[ring.head]);
            ring.items[ring.head] = PendingInput{};
            ring.head = (ring.head + 1) % kQueuePerController;
            --ring.count;
        }
    }
    // Stable insertion sort over at most eight entries; avoids library sort
    // instantiations that GCC 13 misreports under -Warray-bounds.
    for (std::size_t index = 1; index < selected.size(); ++index) {
        const PendingInput item = selected[index];
        std::size_t position = index;
        while (position > 0 && ordered_before(item, selected[position - 1])) {
            selected[position] = selected[position - 1];
            --position;
        }
        selected[position] = item;
    }
    ++release_count_;
    ++counters_.releases;
    counters_.selected += selected.size();
    return selected;
}

std::size_t InputScheduler::pending_for_slot(std::uint32_t slot) const {
    for (std::size_t index = 0; index < bindings_.size(); ++index) {
        if (bindings_[index].player_slot == slot) {
            return queues_[index].count;
        }
    }
    return 0;
}

std::size_t InputScheduler::total_pending() const {
    std::size_t total = 0;
    for (const Ring& ring : queues_) {
        total += ring.count;
    }
    return total;
}

}  // namespace motion_tennis::physics
