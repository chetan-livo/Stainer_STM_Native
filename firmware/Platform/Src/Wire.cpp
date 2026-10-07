#include "Wire.h"
#include "Gpio.h"
#include "PinMaps.h"
#include "Timebase.h"
#include "platform_irq.h"

namespace {
TwoWire* owners[3] = {};

int indexOf(I2C_TypeDef* i2c)
{
    if (i2c == I2C1) return 0;
    if (i2c == I2C2) return 1;
#ifdef I2C3
    if (i2c == I2C3) return 2;
#endif
    return -1;
}

void enableClock(I2C_TypeDef* i2c)
{
    if (i2c == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    else if (i2c == I2C2) RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
#ifdef I2C3
    else if (i2c == I2C3) RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
#endif
    (void)RCC->APB1ENR;
}

void setIrq(I2C_TypeDef* i2c, bool on)
{
    IRQn_Type ev = I2C1_EV_IRQn, er = I2C1_ER_IRQn;
    if (i2c == I2C2) { ev = I2C2_EV_IRQn; er = I2C2_ER_IRQn; }
#ifdef I2C3
    if (i2c == I2C3) { ev = I2C3_EV_IRQn; er = I2C3_ER_IRQn; }
#endif
    if (on) {
        HAL_NVIC_SetPriority(ev, IRQ_PRIO_I2C, 0);
        HAL_NVIC_SetPriority(er, IRQ_PRIO_I2C, 0);
        HAL_NVIC_EnableIRQ(ev);
        HAL_NVIC_EnableIRQ(er);
    } else {
        HAL_NVIC_DisableIRQ(ev);
        HAL_NVIC_DisableIRQ(er);
    }
}

TwoWire* ownerOf(I2C_HandleTypeDef* h)
{
    const int i = indexOf(h->Instance);
    return i >= 0 ? owners[i] : nullptr;
}
}

TwoWire::TwoWire(uint8_t sda, uint8_t scl) : sda_(sda), scl_(scl) {}

bool TwoWire::resolve()
{
    const AfPin* sda = findPin(i2cSdaPins, i2cSdaPinCount, sda_);
    const AfPin* scl = findPin(i2cSclPins, i2cSclPinCount, scl_);
    if (!sda || !scl || sda->instance != scl->instance) return false;
    handle_.Instance = (I2C_TypeDef*)sda->instance;
    alternate_ = sda->alternate;
    return true;
}

bool TwoWire::init(uint32_t ownAddress)
{
    if (!resolve()) return false;
    const int index = indexOf(handle_.Instance);
    if (index < 0) return false;
    enableClock(handle_.Instance);
    pinAlternate(sda_, alternate_, true);
    pinAlternate(scl_, alternate_, true);
    // Clear a BUSY flag left by an interrupted transfer (STM32F4 errata 2.9.7).
    handle_.Instance->CR1 |= I2C_CR1_SWRST;
    handle_.Instance->CR1 &= ~I2C_CR1_SWRST;
    handle_.Init.ClockSpeed = clockHz_;
    handle_.Init.DutyCycle = I2C_DUTYCYCLE_2;
    handle_.Init.OwnAddress1 = ownAddress << 1;
    handle_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    handle_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    handle_.Init.OwnAddress2 = 0;
    handle_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    handle_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    handle_.State = HAL_I2C_STATE_RESET;
    if (HAL_I2C_Init(&handle_) != HAL_OK) return false;
    owners[index] = this;
    started_ = true;
    return true;
}

void TwoWire::begin()
{
    slave_ = false;
    init(0);
}

void TwoWire::begin(uint8_t address, bool generalCall, bool noStretch)
{
    slave_ = true;
    handle_.Init.GeneralCallMode = generalCall ? I2C_GENERALCALL_ENABLE : I2C_GENERALCALL_DISABLE;
    (void)noStretch; // clock stretching stays enabled, as in the Arduino build
    if (!init(address)) return;
    setIrq(handle_.Instance, true);
    HAL_I2C_EnableListen_IT(&handle_);
}

void TwoWire::end()
{
    if (!started_) return;
    if (slave_) setIrq(handle_.Instance, false);
    HAL_I2C_DeInit(&handle_);
    const int index = indexOf(handle_.Instance);
    if (index >= 0 && owners[index] == this) owners[index] = nullptr;
    started_ = false;
}

void TwoWire::setClock(uint32_t hz)
{
    clockHz_ = hz;
    if (started_ && !slave_) { HAL_I2C_DeInit(&handle_); init(0); }
}

void TwoWire::beginTransmission(uint8_t address)
{
    if (pendingRestart_) { pendingRestart_ = false; endTransmission(true); }
    txAddress_ = address;
    txLength_ = 0;
    txOverflow_ = false;
    txActive_ = true;
}

size_t TwoWire::write(uint8_t value)
{
    if (slave_ || !txActive_) {
        // Slave: bytes for the pending read request (written from onRequest).
        if (txLength_ >= BufferLength) return 0;
        txBuffer_[txLength_++] = value;
        return 1;
    }
    if (txLength_ >= BufferLength) { txOverflow_ = true; return 0; }
    txBuffer_[txLength_++] = value;
    return 1;
}

size_t TwoWire::write(const uint8_t* data, size_t length)
{
    size_t n = 0;
    while (n < length && write(data[n])) ++n;
    return n;
}

uint8_t TwoWire::mapError(size_t unsent, size_t length)
{
    // HAL sets XferCount = length before the address phase, so nothing sent
    // means the address was NACKed.
    const uint32_t error = HAL_I2C_GetError(&handle_);
    if (error & HAL_I2C_ERROR_TIMEOUT) return 5;
    if (error & HAL_I2C_ERROR_AF) return unsent >= length ? 2 : 3;
    return 4;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    if (!started_ || slave_) return 4;
    txActive_ = false;
    if (txOverflow_) return 1;
    if (!sendStop) { pendingRestart_ = true; return 0; } // sent with the following read
    if (txLength_ == 0)
        return HAL_I2C_IsDeviceReady(&handle_, (uint16_t)(txAddress_ << 1), 1, TimeoutMs) == HAL_OK ? 0 : 2;
    if (HAL_I2C_Master_Transmit(&handle_, (uint16_t)(txAddress_ << 1), txBuffer_, (uint16_t)txLength_, TimeoutMs) == HAL_OK)
        return 0;
    return mapError(handle_.XferCount, txLength_);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, bool sendStop)
{
    (void)sendStop; // HAL always ends a read with STOP
    rxIndex_ = rxLength_ = 0;
    if (!started_ || slave_ || quantity == 0) return 0;
    if (quantity > BufferLength) quantity = BufferLength;
    HAL_StatusTypeDef status;
    if (pendingRestart_ && address == txAddress_ && (txLength_ == 1 || txLength_ == 2)) {
        // Register read: START, addr+W, register, repeated START, addr+R, data, STOP.
        const uint16_t reg = txLength_ == 1 ? txBuffer_[0] : (uint16_t)((txBuffer_[0] << 8) | txBuffer_[1]);
        status = HAL_I2C_Mem_Read(&handle_, (uint16_t)(address << 1), reg,
                                  txLength_ == 1 ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT,
                                  rxBuffer_, quantity, TimeoutMs);
    } else {
        if (pendingRestart_) { pendingRestart_ = false; endTransmission(true); }
        status = HAL_I2C_Master_Receive(&handle_, (uint16_t)(address << 1), rxBuffer_, quantity, TimeoutMs);
    }
    pendingRestart_ = false;
    if (status != HAL_OK) return 0;
    rxLength_ = quantity;
    return quantity;
}

bool TwoWire::recoverBus()
{
    if (slave_) return false;
    if (started_) HAL_I2C_DeInit(&handle_);
    started_ = false;
    pinMode(sda_, INPUT);
    pinMode(scl_, OUTPUT_OPEN_DRAIN);
    digitalWrite(scl_, HIGH);
    for (int i = 0; i < 9 && digitalRead(sda_) == LOW; ++i) {
        digitalWrite(scl_, LOW); delayMicroseconds(5);
        digitalWrite(scl_, HIGH); delayMicroseconds(5);
    }
    // STOP: SDA low -> high while SCL is high.
    pinMode(sda_, OUTPUT_OPEN_DRAIN);
    digitalWrite(sda_, LOW); delayMicroseconds(5);
    digitalWrite(scl_, HIGH); delayMicroseconds(5);
    digitalWrite(sda_, HIGH); delayMicroseconds(5);
    const bool released = digitalRead(sda_) == HIGH;
    init(0);
    return released;
}

void TwoWire::onReceive(void (*handler)(int)) { onReceive_ = handler; }
void TwoWire::onRequest(void (*handler)()) { onRequest_ = handler; }

// --- Slave state machine (interrupt context) -------------------------------

void TwoWire::slaveAddressed(uint8_t direction)
{
    if (direction == I2C_DIRECTION_TRANSMIT) {
        // Master writes: collect bytes one at a time until STOP.
        rxIndex_ = rxLength_ = 0;
        HAL_I2C_Slave_Seq_Receive_IT(&handle_, &slaveRxByte_, 1, I2C_NEXT_FRAME);
    } else {
        txLength_ = 0;
        if (onRequest_) onRequest_();
        if (txLength_ == 0) txBuffer_[txLength_++] = 0xFF; // nothing queued: send idle byte
        HAL_I2C_Slave_Seq_Transmit_IT(&handle_, txBuffer_, (uint16_t)txLength_, I2C_LAST_FRAME);
    }
}

void TwoWire::slaveByteReceived()
{
    if (rxLength_ < BufferLength) rxBuffer_[rxLength_++] = slaveRxByte_;
    HAL_I2C_Slave_Seq_Receive_IT(&handle_, &slaveRxByte_, 1, I2C_NEXT_FRAME);
}

void TwoWire::slaveListenComplete()
{
    if (rxLength_ && onReceive_) { rxIndex_ = 0; onReceive_((int)rxLength_); }
    rxLength_ = rxIndex_ = 0;
    HAL_I2C_EnableListen_IT(&handle_);
}

void TwoWire::slaveError()
{
    // A NACK after the last byte of a slave transmit is the normal end of a read.
    HAL_I2C_EnableListen_IT(&handle_);
}

extern "C" {
void HAL_I2C_AddrCallback(I2C_HandleTypeDef* h, uint8_t direction, uint16_t)
{
    if (TwoWire* w = ownerOf(h)) w->slaveAddressed(direction);
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* h) { if (TwoWire* w = ownerOf(h)) w->slaveByteReceived(); }
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef*) {}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* h) { if (TwoWire* w = ownerOf(h)) w->slaveListenComplete(); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* h) { if (TwoWire* w = ownerOf(h)) w->slaveError(); }

void I2C1_EV_IRQHandler(void) { if (owners[0]) HAL_I2C_EV_IRQHandler(owners[0]->getHandle()); }
void I2C1_ER_IRQHandler(void) { if (owners[0]) HAL_I2C_ER_IRQHandler(owners[0]->getHandle()); }
void I2C2_EV_IRQHandler(void) { if (owners[1]) HAL_I2C_EV_IRQHandler(owners[1]->getHandle()); }
void I2C2_ER_IRQHandler(void) { if (owners[1]) HAL_I2C_ER_IRQHandler(owners[1]->getHandle()); }
#ifdef I2C3
void I2C3_EV_IRQHandler(void) { if (owners[2]) HAL_I2C_EV_IRQHandler(owners[2]->getHandle()); }
void I2C3_ER_IRQHandler(void) { if (owners[2]) HAL_I2C_ER_IRQHandler(owners[2]->getHandle()); }
#endif
}
