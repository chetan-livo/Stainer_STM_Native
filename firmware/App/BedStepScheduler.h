#pragma once
#include <stddef.h>
#include <stdint.h>

// Production events use a shared, forward-only bed coordinate. No elapsed-time
// conversion or extrapolation is permitted here. Positions and sequence numbers
// may wrap; each outstanding horizon must remain below 2^31 steps.
namespace BedSteps {
inline int32_t difference(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b);
}

enum class Fault : uint8_t {
    None, NotSynchronized, WrongEpoch, InvalidBatch, QueueFull,
    LateEvent, StationNotReady, ActionFailed, ReverseTravel
};
enum class Admission : uint8_t { Accepted, Duplicate, Rejected };
enum class Execution : uint8_t { Started, NotReady, Failed };

struct Event {
    uint32_t step;
    uint32_t sequence;
    uint16_t action;
    uint8_t stage;  // 0=IR4, 1=IR6, 2=IR12
};

template<size_t Capacity> class Scheduler {
    static_assert(Capacity > 0, "A scheduler needs event capacity");
    struct Slot { Event event = {}; bool used = false; } slots[Capacity];
    uint32_t epoch_ = 0, position_ = 0;
    uint32_t lastSequence_[3] = {};
    bool seenStage_[3] = {}, synchronized_ = false;
    size_t depth_ = 0;
    Fault fault_ = Fault::NotSynchronized;

    bool fail(Fault value) { fault_ = value; return false; }
public:
    // Only the owning controller may establish a new epoch after explicitly
    // quiescing outputs and invalidating the previous physical run.
    void reset(uint32_t epoch, uint32_t position) {
        for (auto& slot : slots) slot.used = false;
        for (size_t i = 0; i < 3; ++i) {
            lastSequence_[i] = 0;
            seenStage_[i] = false;
        }
        epoch_ = epoch; position_ = position; depth_ = 0;
        synchronized_ = true; fault_ = Fault::None;
    }
    bool synchronized() const { return synchronized_; }
    bool healthy() const { return synchronized_ && fault_ == Fault::None; }
    Fault fault() const { return fault_; }
    size_t depth() const { return depth_; }
    uint32_t position() const { return position_; }
    uint32_t epoch() const { return epoch_; }

    // A stale position packet must never silently start work in a new run.
    bool observe(uint32_t epoch, uint32_t position) {
        if (!healthy()) return false;
        if (epoch != epoch_) return fail(Fault::WrongEpoch);
        if (difference(position, position_) < 0) return fail(Fault::ReverseTravel);
        position_ = position;
        return true;
    }

    // Reserve the entire slide/stage transaction before inserting anything.
    // Duplicate retries are acknowledged without re-creating completed work.
    Admission admit(uint32_t epoch, uint8_t stage, uint32_t sequence,
                    const Event* events, size_t count) {
        if (!healthy()) return Admission::Rejected;
        if (epoch != epoch_) { fail(Fault::WrongEpoch); return Admission::Rejected; }
        if (stage >= 3 || events == nullptr || count == 0) {
            fail(Fault::InvalidBatch); return Admission::Rejected;
        }
        if (seenStage_[stage] && difference(sequence, lastSequence_[stage]) <= 0)
            return Admission::Duplicate;
        if (count > Capacity - depth_) {
            fail(Fault::QueueFull); return Admission::Rejected;
        }
        for (size_t i = 0; i < count; ++i) {
            if (events[i].stage != stage || events[i].sequence != sequence) {
                fail(Fault::InvalidBatch); return Admission::Rejected;
            }
            if (difference(events[i].step, position_) < 0) {
                fail(Fault::LateEvent); return Admission::Rejected;
            }
        }
        size_t source = 0;
        for (auto& slot : slots) {
            if (!slot.used && source < count) {
                slot.event = events[source++]; slot.used = true; ++depth_;
            }
        }
        seenStage_[stage] = true; lastSequence_[stage] = sequence;
        return Admission::Accepted;
    }

    // Independent event positions: an earlier S2 event is never clamped to SX,
    // and an earlier deadline is never hidden behind a FIFO head.
    bool next(uint32_t& step) const {
        bool found = false;
        for (const auto& slot : slots) if (slot.used) {
            if (!found || difference(slot.event.step, step) < 0) {
                step = slot.event.step; found = true;
            }
        }
        return found;
    }

    // The caller checks resource readiness before advancing the bed across a
    // boundary. A missed boundary is a latched fault, never a late dispense.
    // A blocked/failed event remains available for diagnostic reconciliation.
    template<class Ready, class Start> bool dispatch(Ready ready, Start start) {
        if (!healthy()) return false;
        for (const auto& slot : slots)
            if (slot.used && difference(position_, slot.event.step) > 0)
                return fail(Fault::LateEvent);
        // Check all simultaneous actions before starting the first one. The
        // readiness callback is read-only and must account for shared resources.
        for (const auto& slot : slots)
            if (slot.used && slot.event.step == position_ && !ready(slot.event))
                return fail(Fault::StationNotReady);
        for (auto& slot : slots) if (slot.used && slot.event.step == position_) {
            const Execution result = start(slot.event);
            if (result == Execution::NotReady) return fail(Fault::StationNotReady);
            if (result != Execution::Started) return fail(Fault::ActionFailed);
            slot.used = false; --depth_;
        }
        return true;
    }
};
} // namespace BedSteps
