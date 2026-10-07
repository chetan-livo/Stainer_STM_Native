/*
 * LidarReader.cpp
 *
 * Created on: Apr 2, 2026
 * Author: OpenAI Codex
 */

#include "LidarReader.h"
#include "SensorFault.h"

// Mirror Serial.print(...) on every Master build (gantry/nozzle → MasterSerial, master → PiSerial)
#ifdef Master
#define Serial mirroredSerial
#endif

#ifdef Master
#ifdef Stainer_Gantry_PCB
namespace {
constexpr float GX_HOME_DISTANCE_MM = 156.6f;
constexpr float LIDAR_CORRECTION_SCALE = 0.9375f;
constexpr float LIDAR_CORRECTION_OFFSET_MM = 9.375f;
constexpr uint16_t LIDAR_MAX_VALID_RAW_MM = 2000;
constexpr uint16_t LIDAR_TIMEOUT_MS = 500;
constexpr uint32_t LIDAR_TIMING_BUDGET_US = 100000;
}

LidarReader::LidarReader(TwoWire& wire) : wire_(wire) {}

void LidarReader::setup() {
    if (initialized_) {
        return;
    }

    sensor_.setBus(&wire_);
    sensor_.setTimeout(LIDAR_TIMEOUT_MS);

    // STM32duino 3.0.0's HAL_I2C_ErrorCallback incorrectly enables slave
    // listen mode on master errors. An absent LiDAR's register read can
    // therefore strand the shared Hall bus in HAL_I2C_STATE_LISTEN.
    // Address-only probing avoids that interrupt-driven error path.
    wire_.beginTransmission(sensor_.getAddress());
    const uint8_t probeResult = wire_.endTransmission(true);
    if (probeResult != 0) {
        SensorFault::report("LIVO-SEN-003", "LIDAR", probeResult, "probe_failed");
        wire_.end();
        wire_.begin();
        return;
    }

    if (!sensor_.init()) {
        SensorFault::report("LIVO-SEN-003","LIDAR",-1,"initialization_failed");
        initialized_ = false;
        // Restore the shared master bus after a failed register transaction.
        wire_.end();
        wire_.begin();
        return;
    }

    sensor_.setMeasurementTimingBudget(LIDAR_TIMING_BUDGET_US);
    initialized_ = true;
}

float LidarReader::correctDistance(uint16_t raw) const {
    float correctedDistance = (LIDAR_CORRECTION_SCALE * raw) - LIDAR_CORRECTION_OFFSET_MM;
    if (correctedDistance < 0.0f) {
        correctedDistance = 0.0f;
    }
    return correctedDistance;
}

uint16_t LidarReader::readAverageRaw(uint8_t count) {
    if (!initialized_ || count == 0) {
        SensorFault::report("LIVO-SEN-003","LIDAR",-1,"not_initialized_or_invalid_count");
        return 0;
    }

    uint32_t sum = 0;
    uint8_t validCount = 0;
    long lastRaw=-1; bool timedOut=false;

    for (uint8_t sampleIndex = 0; sampleIndex < count; ++sampleIndex) {
        const uint16_t rawDistance = sensor_.readRangeSingleMillimeters();

        lastRaw=rawDistance; timedOut=sensor_.timeoutOccurred();
        if (!timedOut && rawDistance > 0 && rawDistance < LIDAR_MAX_VALID_RAW_MM) {
            sum += rawDistance;
            validCount++;
        }

        delay(10);
    }

    if (validCount != count) {
        SensorFault::report("LIVO-SEN-003","LIDAR",lastRaw,timedOut?"range_timeout":"invalid_or_incomplete_samples");
        return 0;
    }

    return (uint16_t)(sum / validCount);
}

bool LidarReader::readCorrectedDistanceMm(float& correctedDistanceMm, uint8_t count) {
    const uint16_t rawDistance = readAverageRaw(count);
    if (rawDistance == 0) {
        return false;
    }

    correctedDistanceMm = correctDistance(rawDistance);
    return true;
}

bool LidarReader::readGxPositionMm(float& gxPositionMm, uint8_t count) {
    float correctedDistanceMm = 0.0f;
    if (!readCorrectedDistanceMm(correctedDistanceMm, count)) {
        return false;
    }

    gxPositionMm = GX_HOME_DISTANCE_MM - correctedDistanceMm;
    return true;
}
#endif
#endif
