#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "fixture_proto_json.hpp"
#include "fixtures.hpp"
#include "motion_tennis/physics/batch_core.hpp"
#include "motion_tennis/physics/fakes.hpp"

namespace {

using namespace motion_tennis::physics;
using namespace motion_tennis::physics::fixtures;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::string hex_digest(const Sha256& digest) {
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (const auto byte : digest) text << std::setw(2) << static_cast<unsigned>(byte);
    return text.str();
}

std::string run(const char* expected_digest) {
    const MatchSetup setup = build_setup();
    if (expected_digest) {
        require(hex_digest(setup.configuration_sha256) == expected_digest, "Canonical setup digest mismatch");
    }
    const Snapshot initial = build_snapshot(setup);
    fakes::FakeStateService source(setup, initial);
    fakes::FakePoseSource poses;
    fakes::ScriptedContactEvaluator evaluator;
    BatchCore core(setup, initial.simulation_tick, poses, evaluator);

    const Vector3 first_hit{0.0f, 5.0f, -10.0f};
    evaluator.script_hit(controller_for_slot(1), 1, first_hit);
    evaluator.script_hit(controller_for_slot(2), 2, Vector3{7.0f, 1.0f, 1.0f});
    for (std::uint32_t slot = 1; slot <= 3; ++slot) {
        const auto person = person_for_slot(slot);
        poses.set(person, tracked_pose(10, kInitialPiTimeUs, person));
        const auto capture = slot == 2 ? kInitialPiTimeUs - 10000 : kInitialPiTimeUs + 4000;
        const auto input = input_for(slot, slot, capture);
        require(core.admit(input, kInitialPiTimeUs + 4500 + slot) == AdmitResult::Queued,
                "Input was not admitted to the batch");
    }
    const auto query = core.release(initial.simulation_tick + 1);
    require(query && query->queries.size() == 3, "Expected all three inputs in the released query");
    const auto response = source.query(*query);
    require(response.status == QueryStatus::Ok && response.has_snapshot, "Could not obtain the initial snapshot");

    // The middle sample is 25 ms old at response and 31 ms old at submission.
    // It must participate in the first candidate and drop out when the 30 ms check repeats.
    core.on_query_response(response, kInitialPiTimeUs + 15000);
    const auto& staged = evaluator.observations();
    require(staged.size() == 3 && staged[1].transaction == query->queries[1].transaction,
            "Middle input was not evaluated in the first candidate");
    require(core.take_feedback().empty(), "Unexpected feedback before the precommit eligibility check");
    const auto update = core.prepare_commit(kInitialPiTimeUs + 21000);
    require(update && core.counters().rebuilds == 1 && update->decisions.size() == 2, "Mixed batch was not rebuilt");
    const auto& first = update->decisions[0];
    const auto& last = update->decisions[1];
    require(first.transaction == query->queries[0].transaction && first.outcome == Outcome::Hit && first.event_index == 0 &&
            last.transaction == query->queries[2].transaction && last.outcome == Outcome::Miss && last.event_index == 1,
            "Decisions lost source order, identity or outcome");
    require(update->match.ball.velocity_m_s == first_hit && update->match.rally.contact_id == 1 &&
            update->match.rally.last_hitter_slot == 1 && update->match.rally.contact_slot_mask == 1,
            "Rebuilt state still contains effects of the rejected hit");

    // Rejections accompany the state update as a host-fixture sidecar. Join independent core
    // outputs rather than inferring a rejection by subtracting the surviving decision keys.
    const auto failed = core.take_feedback();
    require(failed.size() == 1 && !failed[0].committed && failed[0].outcome == Outcome::StaleInput &&
            failed[0].transaction == query->queries[1].transaction && failed[0].game_session_id == query->game_session_id &&
            failed[0].batch_id == query->batch_id && failed[0].writer_epoch == query->writer_epoch,
            "Expected one terminal stale-input outcome for this batch and game session");
    const auto trace = core.take_trace();
    const TraceRecord* rejection = nullptr;
    for (const auto& record : trace) {
        if (record.kind == TraceKind::InputFailed) {
            require(!rejection, "Duplicate failure trace");
            rejection = &record;
        }
    }
    require(rejection && rejection->transaction == failed[0].transaction && rejection->outcome == failed[0].outcome &&
            rejection->query_id == query->queries[1].query_id && rejection->batch_id == query->batch_id &&
            rejection->writer_epoch == query->writer_epoch && rejection->observed_pi_us == kInitialPiTimeUs + 21000,
            "Terminal feedback and failure trace do not match the original query");

    using proto_json::json;
    using proto_json::object;
    const auto rejected = object({{"query_id", json(rejection->query_id)}, {"transaction", json(failed[0].transaction)},
                                  {"outcome", json(failed[0].outcome)}, {"observed_pi_us", json(rejection->observed_pi_us)}});
    return object({{"configuration", json(setup.configuration)}, {"snapshot", json(response.snapshot)},
                   {"query", json(*query)}, {"update", json(*update)}, {"rejected", "[" + rejected + "]"}});
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 1 || argc == 2, "Usage: s1_game_fixture [lowercase-hex-configuration-sha256]");
        std::cout << run(argc == 2 ? argv[1] : nullptr) << '\n';
        return std::cout ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
