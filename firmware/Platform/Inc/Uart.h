#pragma once
#include "Print.h"
#include "RingBuffer.h"
#include "stm32f4xx.h"

enum UartFraming : uint8_t {
    SERIAL_8N1, // 8 data bits, no parity, 1 stop
    SERIAL_8E1, // 8 data bits, even parity, 1 stop (STM32 ROM bootloader)
};

// Interrupt-driven U(S)ART with 1 KiB receive and transmit buffers.
// write() blocks only while the transmit buffer is full.
class Uart : public Stream {
public:
    static constexpr size_t RxSize = 1024, TxSize = 1024;

    // The instance is resolved from the pin pair (see Uart.cpp's table);
    // pass `instance` where a pair is shared by two peripherals.
    Uart(uint8_t rxPin, uint8_t txPin, USART_TypeDef* instance = nullptr);

    bool begin(uint32_t baud, UartFraming framing = SERIAL_8N1);
    void end();
    explicit operator bool() const { return started_; }

    int available() override { return (int)rx_.available(); }
    int read() override { return rx_.pop(); }
    int peek() override { return rx_.peek(); }
    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buffer, size_t size) override;
    using Print::write;
    int availableForWrite() override { return (int)tx_.space(); }
    void flush() override; // wait until every queued byte has left the wire

    uint32_t overruns() const { return overruns_; } // bytes lost to a full buffer or ORE
    void irq();

private:
    USART_TypeDef* usart_ = nullptr;
    uint8_t rxPin_, txPin_, alternate_ = 0;
    bool started_ = false;
    volatile uint32_t overruns_ = 0;
    RingBuffer<RxSize> rx_;
    RingBuffer<TxSize> tx_;
};
