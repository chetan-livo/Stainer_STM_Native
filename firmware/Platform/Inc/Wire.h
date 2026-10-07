#pragma once
#include "Print.h"
#include "Pins.h"
#include "stm32f4xx_hal.h"

// I2C master/slave with the Arduino TwoWire interface used by the firmware.
//
// Master: blocking HAL transfers with a 100 ms timeout (as STM32duino).
// endTransmission() returns 0 ok, 1 too long, 2 address NACK, 3 data NACK,
// 4 other error, 5 timeout. endTransmission(false) followed by requestFrom()
// to the same address performs a repeated-start register read.
// Slave: begin(address) plus onReceive()/onRequest(); callbacks run in the
// I2C interrupt (IRQ_PRIO_I2C), as in STM32duino.
class TwoWire : public Stream {
public:
    static constexpr size_t BufferLength = 64;
    static constexpr uint32_t TimeoutMs = 100;

    TwoWire(uint8_t sda, uint8_t scl);
    void setSDA(uint8_t pin) { sda_ = pin; }
    void setSCL(uint8_t pin) { scl_ = pin; }
    void begin();                       // master
    void begin(uint8_t address, bool generalCall = false, bool noStretch = false); // slave
    void end();
    void setClock(uint32_t hz);

    void beginTransmission(uint8_t address);
    uint8_t endTransmission(bool sendStop = true);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop = true);

    size_t write(uint8_t value) override;
    size_t write(const uint8_t* data, size_t length) override;
    using Print::write;
    int available() override { return (int)(rxLength_ - rxIndex_); }
    int read() override { return rxIndex_ < rxLength_ ? rxBuffer_[rxIndex_++] : -1; }
    int peek() override { return rxIndex_ < rxLength_ ? rxBuffer_[rxIndex_] : -1; }
    void flush() override {}

    void onReceive(void (*handler)(int));
    void onRequest(void (*handler)());

    // Clock SCL until a stuck slave releases SDA, then issue STOP (master only).
    bool recoverBus();
    I2C_HandleTypeDef* getHandle() { return &handle_; }

    // HAL callback routing (interrupt context).
    void slaveAddressed(uint8_t direction);
    void slaveByteReceived();
    void slaveListenComplete();
    void slaveError();

private:
    bool resolve();
    bool init(uint32_t ownAddress);
    uint8_t mapError(size_t unsent, size_t length);

    I2C_HandleTypeDef handle_ = {};
    uint8_t sda_, scl_;
    uint8_t alternate_ = 0;
    uint32_t clockHz_ = 100000;
    bool slave_ = false, started_ = false;

    uint8_t txAddress_ = 0;
    uint8_t txBuffer_[BufferLength];
    size_t txLength_ = 0;
    bool txActive_ = false, txOverflow_ = false;
    bool pendingRestart_ = false;      // endTransmission(false) awaiting requestFrom()

    uint8_t rxBuffer_[BufferLength];
    volatile size_t rxLength_ = 0, rxIndex_ = 0;
    uint8_t slaveRxByte_ = 0;

    void (*onReceive_)(int) = nullptr;
    void (*onRequest_)() = nullptr;
};
