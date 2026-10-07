#pragma once
#include <Arduino.h>
#include "StepEngine.h"

// Lets sources shared with host tests (MasterBedProduction.inc) compile the
// hooks only in firmware builds.
#define LIVO_LOOPSTATS 1

// Main-loop and stepper-service timing reported by LOOPSTAT.
//
// Every stepper pulse is produced by polling TMCModule::loop(), so the longest
// gap between two services of a running motor bounds step-timing jitter. Each
// LOOPSTAT reports the window since the previous one and starts a new window.
// Uses the DWT cycle counter that the STM32 core already enables; one interval
// is meaningful up to ~25 s at 168 MHz (counter wrap).
namespace LoopStats {
inline uint32_t generation = 1;
inline uint32_t intervals = 0, lastLoopCycles = 0, maxLoopCycles = 0;
inline uint64_t totalLoopCycles = 0;
inline uint32_t maxStepGapCycles = 0, windowStartMs = 0;
// Master bed: every T step waits for the Nozzle's A barrier. If that round trip
// exceeds the step period, the bed runs slower than the commanded FEED rate.
inline uint32_t bedAcks = 0, maxBedAckCycles = 0;
inline uint64_t totalBedAckCycles = 0;
inline float bedCommandedStepsPerS = 0.0f;

inline uint32_t cycles() { return DWT->CYCCNT; }

inline void onBedAck(uint32_t sentCycles) {
    const uint32_t latency = DWT->CYCCNT - sentCycles;
    ++bedAcks;
    totalBedAckCycles += latency;
    if (latency > maxBedAckCycles) maxBedAckCycles = latency;
}

// Called once at the top of loop().
inline void onLoop() {
    const uint32_t now = DWT->CYCCNT;
    static uint32_t loopGeneration = 0;
    if (loopGeneration == generation) {
        const uint32_t elapsed = now - lastLoopCycles;
        ++intervals;
        totalLoopCycles += elapsed;
        if (elapsed > maxLoopCycles) maxLoopCycles = elapsed;
    }
    loopGeneration = generation;
    lastLoopCycles = now;
}

// Called for each service of a running motor; `lastGeneration` is 0 while the
// motor is stopped so the idle period is never counted as a gap.
inline void onStepService(uint32_t& lastCycles, uint32_t& lastGeneration) {
    const uint32_t now = DWT->CYCCNT;
    if (lastGeneration == generation) {
        const uint32_t gap = now - lastCycles;
        if (gap > maxStepGapCycles) maxStepGapCycles = gap;
    }
    lastCycles = now;
    lastGeneration = generation;
}

template <class Out> void report(Out& out) {
    const uint32_t cyclesPerUs = SystemCoreClock / 1000000UL;
    out.print("LOOPSTAT WINDOW_MS:"); out.print(millis() - windowStartMs);
    out.print(" LOOPS:"); out.print(intervals);
    out.print(" AVG_LOOP_US:");
    out.print(intervals ? (uint32_t)(totalLoopCycles / intervals / cyclesPerUs) : 0UL);
    out.print(" MAX_LOOP_US:"); out.print(maxLoopCycles / cyclesPerUs);
    out.print(" MAX_STEP_GAP_US:"); out.print(maxStepGapCycles / cyclesPerUs);
    if (bedAcks) {
        const uint32_t windowMs = millis() - windowStartMs;
        out.print(" BED_STEPS:"); out.print(bedAcks);
        out.print(" BED_STEPS_PER_S:"); out.print(windowMs ? bedAcks * 1000.0f / windowMs : 0.0f, 2);
        out.print(" BED_CMD_STEPS_PER_S:"); out.print(bedCommandedStepsPerS, 2);
        out.print(" AVG_BED_ACK_US:"); out.print((uint32_t)(totalBedAckCycles / bedAcks / cyclesPerUs));
        out.print(" MAX_BED_ACK_US:"); out.print(maxBedAckCycles / cyclesPerUs);
    }
    StepEngine::reportStats(out);
    out.println();
    intervals = 0; totalLoopCycles = 0; maxLoopCycles = 0; maxStepGapCycles = 0;
    bedAcks = 0; totalBedAckCycles = 0; maxBedAckCycles = 0;
    windowStartMs = millis();
    if (++generation == 0) generation = 1;
}
}
