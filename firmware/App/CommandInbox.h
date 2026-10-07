#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Bounded FIFO: operational commands retain order and take precedence over
// background queries. A query can never evict an operational command.
template <size_t Capacity, size_t MaxLength> class CommandInbox {
    struct Entry { uint8_t data[MaxLength]; size_t length; bool background; };
    Entry entries[Capacity] = {};
    size_t count = 0;
    constexpr void remove(size_t at) {
        for (size_t i = at + 1; i < count; ++i) entries[i - 1] = entries[i];
        --count;
    }
public:
    uint32_t rejected = 0, evictedQueries = 0;
    constexpr bool push(const uint8_t* data, size_t length, bool background) {
        if (!length || length > MaxLength) { ++rejected; return false; }
        if (count == Capacity) {
            size_t victim = count;
            if (!background) for (size_t i = 0; i < count; ++i)
                if (entries[i].background) { victim = i; break; }
            if (victim == count) { ++rejected; return false; }
            remove(victim); ++evictedQueries;
        }
        for (size_t i = 0; i < length; ++i) entries[count].data[i] = data[i];
        entries[count].length = length;
        entries[count++].background = background;
        return true;
    }
    constexpr bool pop(uint8_t* data, size_t& length) {
        if (!count) return false;
        size_t next = 0;
        for (size_t i = 0; i < count; ++i)
            if (!entries[i].background) { next = i; break; }
        length = entries[next].length;
        for (size_t i = 0; i < length; ++i) data[i] = entries[next].data[i];
        remove(next);
        return true;
    }
};
