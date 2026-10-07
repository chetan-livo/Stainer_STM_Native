#pragma once
#include <stddef.h>
#include <stdint.h>

// Single-producer/single-consumer byte FIFO; one side may be an interrupt.
// Size must be a power of two. One slot stays empty to tell full from empty.
template <size_t Size> class RingBuffer {
    static_assert(Size >= 2 && (Size & (Size - 1)) == 0, "Size must be a power of two");
public:
    bool push(uint8_t value)
    {
        const size_t head = head_, next = (head + 1) & (Size - 1);
        if (next == tail_) return false;
        data_[head] = value;
        head_ = next;
        return true;
    }
    int pop()
    {
        const size_t tail = tail_;
        if (tail == head_) return -1;
        const uint8_t value = data_[tail];
        tail_ = (tail + 1) & (Size - 1);
        return value;
    }
    int peek() const { return tail_ == head_ ? -1 : data_[tail_]; }
    size_t available() const { return (head_ - tail_) & (Size - 1); }
    size_t space() const { return Size - 1 - available(); }
    bool empty() const { return head_ == tail_; }
    void clear() { tail_ = head_; } // consumer side only
    // Contiguous readable bytes starting at the tail (for block transfers).
    size_t contiguous(const uint8_t*& start) const
    {
        const size_t head = head_, tail = tail_;
        start = &data_[tail];
        return head >= tail ? head - tail : Size - tail;
    }
    void consume(size_t count) { tail_ = (tail_ + count) & (Size - 1); }

private:
    uint8_t data_[Size];
    volatile size_t head_ = 0, tail_ = 0;
};
