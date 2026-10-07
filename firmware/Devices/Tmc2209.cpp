// TMC2209 UART driver; see Tmc2209.h. Derived from TMCStepper 0.7.3
// (teemuatlut, MIT License; Devices/LICENSE-TMCStepper.txt).
#include "Tmc2209.h"
#include "Timebase.h"

namespace {
enum : uint8_t { REG_GCONF = 0x00, REG_GSTAT = 0x01, REG_IFCNT = 0x02, REG_IOIN = 0x06,
                 REG_IHOLD_IRUN = 0x10, REG_CHOPCONF = 0x6C, REG_DRV_STATUS = 0x6F };
constexpr uint8_t Sync = 0x05, WriteBit = 0x80;

constexpr uint32_t GCONF_PDN_DISABLE = 1U << 6, GCONF_MSTEP_REG_SELECT = 1U << 7;
constexpr uint32_t CHOPCONF_VSENSE = 1U << 17;
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24, CHOPCONF_MRES_MASK = 0xFU << 24;

uint32_t setBit(uint32_t reg, uint32_t bit, bool on) { return on ? (reg | bit) : (reg & ~bit); }

// Float/double -> uint8_t as the Cortex-M4 FPU converts (VCVT to unsigned
// saturates at 0), so results match the target even for negative inputs.
uint8_t toUint8(double value) { return value <= 0.0 ? 0 : (uint8_t)(uint32_t)value; }
}

Tmc2209::Tmc2209(Stream& port, float rSense, uint8_t address)
    : port_(port), rSense_(rSense), address_(address) {}

uint8_t Tmc2209::calcCRC(const uint8_t* data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = ((crc >> 7) ^ (byte & 0x01)) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
            byte >>= 1;
        }
    }
    return crc;
}

void Tmc2209::write(uint8_t reg, uint32_t value)
{
    uint8_t datagram[8] = {Sync, address_, (uint8_t)(reg | WriteBit),
                           (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                           (uint8_t)(value >> 8), (uint8_t)value, 0};
    datagram[7] = calcCRC(datagram, 7);
    port_.write(datagram, sizeof(datagram));
    delay(ReplyDelayMs);
}

uint64_t Tmc2209::sendReadRequest(const uint8_t request[4])
{
    while (port_.available() > 0) port_.read(); // drop stale bytes
    port_.write(request, 4);
    delay(ReplyDelayMs);

    // The reply starts 0x05 0xFF <register>; our own request echoes back
    // first on the single-wire bus and never matches (address != 0xFF).
    const uint32_t target = ((uint32_t)Sync << 16) | 0xFF00U | request[2];
    uint32_t sync = 0;
    uint32_t start = millis();
    while (sync != target) {
        if (millis() - start >= AbortWindowMs) return 0;
        const int c = port_.read();
        if (c < 0) continue;
        sync = ((sync << 8) | (uint32_t)(c & 0xFF)) & 0xFFFFFFU;
    }
    uint64_t out = sync;
    start = millis();
    for (uint8_t i = 0; i < 5;) {
        if (millis() - start >= AbortWindowMs) return 0;
        const int c = port_.read();
        if (c < 0) continue;
        out = (out << 8) | (uint32_t)(c & 0xFF);
        ++i;
    }
    while (port_.available() > 0) port_.read();
    return out;
}

uint32_t Tmc2209::read(uint8_t reg)
{
    uint8_t request[4] = {Sync, address_, reg, 0};
    request[3] = calcCRC(request, 3);
    uint64_t out = 0;
    for (uint8_t attempt = 0; attempt < MaxRetries; ++attempt) {
        out = sendReadRequest(request);
        delay(ReplyDelayMs);
        uint8_t reply[8];
        for (int i = 0; i < 8; ++i) reply[i] = (uint8_t)(out >> (56 - 8 * i));
        const uint8_t crc = calcCRC(reply, 7);
        CRCerror = crc != reply[7] || crc == 0;
        if (!CRCerror) break;
        out = 0;
    }
    return (uint32_t)(out >> 8);
}

void Tmc2209::begin()
{
    pdn_disable(true);
    mstep_reg_select(true);
}

void Tmc2209::GCONF(uint32_t value) { gconf_ = value; write(REG_GCONF, gconf_); }
uint32_t Tmc2209::GCONF() { return read(REG_GCONF); }
void Tmc2209::CHOPCONF(uint32_t value) { chopconf_ = value; write(REG_CHOPCONF, chopconf_); }
uint32_t Tmc2209::CHOPCONF() { return read(REG_CHOPCONF); }
uint32_t Tmc2209::DRV_STATUS() { return read(REG_DRV_STATUS); }
uint32_t Tmc2209::IOIN() { return read(REG_IOIN); }
uint8_t Tmc2209::GSTAT() { return (uint8_t)read(REG_GSTAT); }
uint8_t Tmc2209::IFCNT() { return (uint8_t)read(REG_IFCNT); }

void Tmc2209::pdn_disable(bool on) { GCONF(setBit(gconf_, GCONF_PDN_DISABLE, on)); }
void Tmc2209::mstep_reg_select(bool on) { GCONF(setBit(gconf_, GCONF_MSTEP_REG_SELECT, on)); }
void Tmc2209::vsense(bool on) { CHOPCONF(setBit(chopconf_, CHOPCONF_VSENSE, on)); }
void Tmc2209::mres(uint8_t value) { CHOPCONF((chopconf_ & ~CHOPCONF_MRES_MASK) | (((uint32_t)value & 0xFU) << CHOPCONF_MRES_SHIFT)); }

void Tmc2209::irun(uint8_t value)
{
    ihold_irun_ = (ihold_irun_ & ~(0x1FU << 8)) | (((uint32_t)value & 0x1FU) << 8);
    write(REG_IHOLD_IRUN, ihold_irun_);
}
void Tmc2209::ihold(uint8_t value)
{
    ihold_irun_ = (ihold_irun_ & ~0x1FU) | ((uint32_t)value & 0x1FU);
    write(REG_IHOLD_IRUN, ihold_irun_);
}
void Tmc2209::iholddelay(uint8_t value)
{
    ihold_irun_ = (ihold_irun_ & ~(0xFU << 16)) | (((uint32_t)value & 0xFU) << 16);
    write(REG_IHOLD_IRUN, ihold_irun_);
}

void Tmc2209::microsteps(uint16_t ms)
{
    switch (ms) {
    case 256: mres(0); break;
    case 128: mres(1); break;
    case 64:  mres(2); break;
    case 32:  mres(3); break;
    case 16:  mres(4); break;
    case 8:   mres(5); break;
    case 4:   mres(6); break;
    case 2:   mres(7); break;
    case 0:   mres(8); break;
    default:  break; // TMCStepper 0.7.3 ignores 1 and other values
    }
}

void Tmc2209::rms_current(uint16_t mA)
{
    // TMCStepper formula, evaluated in double like the library.
    uint8_t cs = toUint8(32.0 * 1.41421 * mA / 1000.0 * (rSense_ + 0.02) / 0.325 - 1);
    if (cs < 16) {
        vsense(true);
        cs = toUint8(32.0 * 1.41421 * mA / 1000.0 * (rSense_ + 0.02) / 0.180 - 1);
    } else {
        vsense(false);
    }
    if (cs > 31) cs = 31;
    irun(cs);
    ihold(toUint8(cs * holdMultiplier_));
}

void Tmc2209::rms_current(uint16_t mA, float holdMultiplier)
{
    holdMultiplier_ = holdMultiplier;
    rms_current(mA);
}

uint16_t Tmc2209::cs2rms(uint8_t cs)
{
    const bool high = chopconf_ & CHOPCONF_VSENSE;
    return (uint16_t)((cs + 1) / 32.0 * (high ? 0.180 : 0.325) / (rSense_ + 0.02) / 1.41421 * 1000);
}

uint16_t Tmc2209::rms_current() { return cs2rms(irun()); }
