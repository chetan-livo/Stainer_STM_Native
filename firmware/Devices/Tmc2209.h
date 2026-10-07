#pragma once
#include <stdint.h>
#include "Print.h"

// TMC2209 UART driver: the subset of TMCStepper 0.7.3 the Livo firmware uses,
// with the same method names, shadow registers, defaults and arithmetic, so
// every datagram is byte-identical (tests/host/test_tmc2209.cpp checks this
// against the library itself).
//
// Derived from TMCStepper by teemuatlut, MIT License:
//   Copyright (c) 2019 teemuatlut
//   Permission is hereby granted, free of charge, to any person obtaining a
//   copy of this software ... (full text in Devices/LICENSE-TMCStepper.txt)
class Tmc2209 {
public:
    // TMCStepper waits replyDelay (2 ms) after every write and around every
    // read. Kept for parity until bench-verified; motion no longer stalls on
    // it because STEP pulses come from the timer interrupt.
    static constexpr uint32_t ReplyDelayMs = 2;
    static constexpr uint32_t AbortWindowMs = 5;
    static constexpr uint8_t MaxRetries = 2;

    Tmc2209(Stream& port, float rSense, uint8_t address);

    void begin();                          // pdn_disable + mstep_reg_select

    // Whole registers. Writes update the shadow copy first.
    void GCONF(uint32_t value);
    uint32_t GCONF();                      // reads the chip
    void CHOPCONF(uint32_t value);
    uint32_t CHOPCONF();                   // reads the chip
    uint32_t IHOLD_IRUN() const { return ihold_irun_; } // write-only on chip: shadow
    uint32_t DRV_STATUS();
    uint32_t IOIN();
    uint8_t GSTAT();
    uint8_t IFCNT();
    uint8_t version() { return (uint8_t)(IOIN() >> 24); }

    // Fields (each writes its register).
    void pdn_disable(bool on);
    void mstep_reg_select(bool on);
    void vsense(bool on);
    void mres(uint8_t value);
    void irun(uint8_t value);
    void ihold(uint8_t value);
    void iholddelay(uint8_t value);
    uint8_t irun() const { return (ihold_irun_ >> 8) & 0x1F; }
    uint8_t ihold() const { return ihold_irun_ & 0x1F; }

    void microsteps(uint16_t ms);          // 1 and other values are ignored, as in TMCStepper
    void rms_current(uint16_t mA);
    void rms_current(uint16_t mA, float holdMultiplier);
    uint16_t rms_current();
    float hold_multiplier() const { return holdMultiplier_; }
    void hold_multiplier(float value) { holdMultiplier_ = value; }

    bool CRCerror = false;                 // last read failed CRC on every retry
    static uint8_t calcCRC(const uint8_t* data, uint8_t length);

private:
    void write(uint8_t reg, uint32_t value);
    uint32_t read(uint8_t reg);
    uint64_t sendReadRequest(const uint8_t request[4]);
    uint16_t cs2rms(uint8_t cs);

    Stream& port_;
    float rSense_;
    uint8_t address_;
    float holdMultiplier_ = 0.5f;
    // Power-on shadow values from TMCStepper defaults().
    uint32_t gconf_ = 0x00000101;          // i_scale_analog, multistep_filt
    uint32_t ihold_irun_ = 0x00010000;     // iholddelay = 1
    uint32_t chopconf_ = 0x10000053;
};
