#pragma once
#include "Print.h"
#include "RingBuffer.h"

// USB CDC virtual COM port (OTG FS, PA11/PA12).
//
// Receive: packets go to a 1 KiB buffer; the OUT endpoint is re-armed only
// when a full packet fits, so the host is throttled instead of losing data.
// Transmit: 2 KiB buffer. With no terminal open (DTR low) output is dropped
// so the control loop never waits on an absent host; with a terminal open,
// write() waits at most WriteTimeoutMs for space.
class UsbSerial : public Stream {
public:
    static constexpr size_t RxSize = 1024, TxSize = 2048;
    static constexpr uint32_t WriteTimeoutMs = 20;

    void begin(uint32_t baud = 0); // baud is ignored by USB
    explicit operator bool() const { return connected(); }
    bool connected() const;       // configured and DTR asserted by the host

    int available() override { return (int)rx_.available(); }
    int read() override;
    int peek() override { return rx_.peek(); }
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t* buffer, size_t size) override;
    using Print::write;
    int availableForWrite() override { return connected() ? (int)tx_.space() : (int)TxSize; }
    void flush() override;

    // USB stack callbacks (interrupt context).
    void onReceive(const uint8_t* data, uint32_t length);
    void onTransmitComplete();
    void onLineState(bool dtr);

private:
    void startTransmit();
    void rearmReceive();
    RingBuffer<RxSize> rx_;
    RingBuffer<TxSize> tx_;
    volatile bool dtr_ = false, rxPaused_ = false, txBusy_ = false;
};

extern UsbSerial Serial;
