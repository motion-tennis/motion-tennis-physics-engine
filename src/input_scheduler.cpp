#include "motion_tennis/physics/input_scheduler.hpp"

#include <algorithm>
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

const FeedbackResult* InputScheduler::terminal_result(const Transaction& transaction) const {
    const int index = slot_index_for(transaction);
    if (index >= 0 && transaction.request_seq != 0) {
        for (const auto& result : histories_[static_cast<std::size_t>(index)].results) {
            if (result.transaction == transaction) {
                return &result;
            }
        }
    }
    return nullptr;
}

void InputScheduler::remember_terminal(const FeedbackResult& result) {
    const int index = slot_index_for(result.transaction);
    if (index < 0) {
        return;
    }
    auto active = std::find(selected_transactions_.begin(), selected_transactions_.end(), result.transaction);
    if (active != selected_transactions_.end()) {
        *active = Transaction{};
    }
    History& history = histories_[static_cast<std::size_t>(index)];
    FeedbackResult& oldest = history.results[history.next];
    history.retired_through = std::max(history.retired_through, oldest.transaction.request_seq);
    oldest = result;
    history.next = (history.next + 1) % history.results.size();
}

AdmitResult InputScheduler::admit(const ControllerInput& input, std::uint64_t arrival_pi_us) {
    const int index = slot_index_for(input.transaction);
    if (index < 0 || input.transaction.request_seq == 0 || input.transaction.request_seq > kMaxWireUint64) {
        ++counters_.rejected_unknown;
        return AdmitResult::UnknownController;
    }
    Ring& ring = queues_[static_cast<std::size_t>(index)];
    // Empty ring cells have a zero request sequence and cannot match a valid input.
    const bool queued = std::any_of(ring.items.begin(), ring.items.end(), [&](const PendingInput& item) {
        return item.input.transaction == input.transaction;
    });
    const bool selected = std::find(selected_transactions_.begin(), selected_transactions_.end(),
                                    input.transaction) != selected_transactions_.end();
    if (queued || selected || terminal_result(input.transaction)) {
        ++counters_.rejected_duplicate;
        return AdmitResult::DuplicateTransaction;
    }
    if (input.transaction.request_seq <= histories_[static_cast<std::size_t>(index)].retired_through) {
        ++counters_.rejected_outside_window;
        return AdmitResult::OutsideReplayWindow;
    }
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
    selected_transactions_.clear();
    BoundedVector<PendingInput, kMaxBatchQueries> selected;
    const std::size_t start = static_cast<std::size_t>(release_count_ % kPlayers);
    for (std::size_t step = 0; step < kPlayers && !selected.full(); ++step) {
        Ring& ring = queues_[(start + step) % kPlayers];
        for (std::size_t taken = 0;
             taken < kMaxQueriesPerController && ring.count > 0 && !selected.full(); ++taken) {
            selected.push_back(ring.items[ring.head]);
            selected_transactions_.push_back(ring.items[ring.head].input.transaction);
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
