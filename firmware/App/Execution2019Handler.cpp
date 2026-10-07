#include "Stm32SerialCompat.h"
/*
 * Execution2019Handler.cpp
 *
 * Created on: June11, 2025
 * Author: Varalakshmi
 */

#include "Execution2019Handler.h"
#include "LidarReader.h"
#include "STAIN-PARAMETERS.h"
#include "NozzleDHT.h"
#include "SlideDetectionFilter.h"
#include "SensorFault.h"
#include "ServiceDiagnostics.h"
#include "ParaCheck.h"
#include "UartLineGuard.h"
#include "BedStepLink.h"
#include "SlidePositionTracker.h"
#include "LoopStats.h"

static String controllerHardwareUid() {
    // STM32F446 96-bit factory-programmed unique device identifier.
    const volatile uint32_t* uid = reinterpret_cast<const volatile uint32_t*>(0x1FFF7A10UL);
    char text[25];
    snprintf(text, sizeof(text), "%08lX%08lX%08lX",
             (unsigned long)uid[0], (unsigned long)uid[1], (unsigned long)uid[2]);
    return String(text);
}

Execution2019Handler execution2019Handler;

#ifdef Master
#ifdef Stainer_Gantry_PCB

// UART to Stainer Master PCB (MS_TX=PC10, MS_RX=PC11) — non-static so other TUs can extern it
LivoHardwareSerial MasterSerial(MX_RX, MS_TX);
MirrorStream mirroredSerial(Serial, MasterSerial);
// Local mirror redirect — only this file's Serial.print(...) calls go to both USB and master.
// Per-file scope avoids breaking other TUs (LivoCommunication.cpp uses Serial.begin(), !Serial, etc.).
#define Serial mirroredSerial

extern TwoWire Wire1;  // Acc1_SDA/Acc1_SCL = PB7/PB8
extern TwoWire Wire2;  // Acc2_SDA/Acc2_SCL = PB11/PB10
extern TwoWire Wire3;  // I2C3_SDA/I2C3_SCL = PC9/PA8

namespace {
constexpr int MAGAZINE2_LOCK_GUARD_INDEX = 23;
constexpr long MAGAZINE2_LOCK_GUARD_THRESHOLD = 1800;
constexpr unsigned long MAGAZINE2_SAMPLE_INTERVAL_MS = 10;
constexpr int MAGAZINE_LOCK_STATUS_INDEX = 23;
constexpr long MAGAZINE_LOCKED_THRESHOLD = 2300;
constexpr unsigned long MAGAZINE_LOCK_SETTLE_MS = 100;
constexpr float GX_HOME_LIDAR_DISTANCE_MM = 158.0f;
constexpr float GX_HOME_PITCH_MM_PER_ROTATION = 6.0f;
constexpr float GX_HOME_FULL_STEPS_PER_ROTATION = 200.0f;
constexpr float GX_HOME_MICROSTEPS_PER_FULL_STEP = 8.0f;
constexpr float GX_HOME_STEPS_PER_MM =    (GX_HOME_FULL_STEPS_PER_ROTATION * GX_HOME_MICROSTEPS_PER_FULL_STEP) / GX_HOME_PITCH_MM_PER_ROTATION;
constexpr uint8_t GX_HOME_LIDAR_SAMPLE_COUNT = 20;
constexpr long GX_HOME_HOMING_SPEED_MIN = 50000;
constexpr long GX_HOME_HOMING_SPEED_BOOST = 50000;
constexpr long GX_HOME_HOMING_SPEED_MAX = 600000;   // 150000 × 4 (8→32 microsteps)
constexpr long GX_HOME_APPROACH_ACCELERATION = 5000; // low accel — limits peak speed → less overshoot
constexpr unsigned long GX_HOME_TIMEOUT_MS = 90000;
constexpr long GZ_HOME_HALL_THRESHOLD = 1160;
constexpr long GZ_HOME_STEP_CHUNK = 2000;
constexpr long GZ_HOME_HOMING_SPEED = 2000;         // 10000 × 4 (8→32 microsteps)
constexpr int GR_HOME_IR3_THRESHOLD = 1300;
constexpr long GLOAD_GX_POSITION = -128000;          // -32300 × 4 (8→32 microsteps)
constexpr long GLOAD_GY_POSITION = 0;
constexpr long GLOAD_GZ_POSITION = 3000;          // 1500 × 4 (8→32 microsteps)
constexpr long GLOAD_GR_POSITION = 0;
constexpr long MAGAZINE1_PICK_GX_POSITION = -300;   // -1500 × 4 (8→32 microsteps)
constexpr long MAGAZINE2_PICK_GX_POSITION = -83000;  // -21875 × 4 (8→32 microsteps)
constexpr long LOAD_ENTRY_GZ_POSITION = 6000;       // 2750 × 4 (8→32 microsteps)
constexpr long LOAD_SLOT_GZ_BASE_POSITION = 68500;   // 16400 × 4 (8→32 microsteps)
constexpr long LOAD_SLOT_GZ_PITCH = 3500;            // 875 × 4 (8→32 microsteps)
constexpr long LOAD_GR_STROKE       = 12800;
constexpr long LOAD_GY_STROKE       = 114000;       // safety max-travel for GY sensor-driven moves (was 57750, +50%)
constexpr long LOAD_GX_SPEED        = 56000;         // 14000 × 4 (8→32 microsteps)
constexpr long LOAD_GX_ACCELERATION = 60000;         // 15000 × 4 (8→32 microsteps)
constexpr long LOAD_GY_SPEED        = 5000;
constexpr long LOAD_GY_ACCELERATION = 3500;
// LOAD GY sensor-driven motion thresholds
// Empirically (see debug log) with a clean carriage start:
//   IR1: baseline ~2460, slide-detect settles at ~568  → threshold midway = 1500
//   IR4: baseline ~2556, home-detect settles at ~1256  → threshold midway = 1800
// If a stroke ever starts with sensor already in "detect" range (e.g., previous
// load left the slide blocking IR1), the arming logic suppresses the immediate
// trip and waits for the sensor to clear above (threshold + margin) first.
constexpr int  LOAD_GY_IR1_THRESHOLD = 1500;         // -ve direction stops when IR1 drops below this
constexpr int  LOAD_GY_IR4_THRESHOLD = 1800;         // +ve direction stops when IR4 drops below this
constexpr int  LOAD_GY_ARM_MARGIN    = 200;          // sensor must rise above (threshold+margin) to "arm"
constexpr long LOAD_GY_STOP_ACCEL    = 400000;       // very high decel for near-instant stop on IR trip
constexpr unsigned long LOAD_GY_TIMEOUT_MS = 30000;  // safety timeout per sensor-driven stroke
constexpr long LOAD_GZ_SPEED        = 48000;         // 12000 × 4 (8→32 microsteps)
constexpr long LOAD_GZ_ACCELERATION = 60000;         // 15000 × 4 (8→32 microsteps)
constexpr int RST_IR1_INDEX = 0;
constexpr int RST_IR2_INDEX = 1;
constexpr int RST_IR3_INDEX = 2;
constexpr int RST_IR4_INDEX = 3;
constexpr int RST_IR1_ACTIVE_THRESHOLD = 500;
constexpr int RST_IR2_ACTIVE_THRESHOLD = 2150;
constexpr int RST_IR3_ACTIVE_THRESHOLD = 1300;
constexpr int RST_IR4_ACTIVE_THRESHOLD = 200;
constexpr long RST_GY_NEGATIVE_STEP_CHUNK = -1000;
constexpr unsigned long RST_GY_NEGATIVE_TIMEOUT_MS = 90000;

// Gantry IR4 is active-high at the Y home position:
// approximately 180 away from home and at least 1500 at home.
constexpr int GYHO_IR4_THRESHOLD     = 1500;
constexpr long GYHO_SPEED            = 200000;
constexpr long GYHO_STEP_CHUNK       = -1000;
constexpr unsigned long GYHO_TIMEOUT_MS = 100000;

#if GANTRY_MAG_HALL_CAL_EXPERIMENT
struct MagazineGxHallCalibration {
    bool valid;
    bool applied;
    long peakPosition[2];
    int peakAdc[2];
    int baselineAdc[2];
};

MagazineGxHallCalibration magazineGxHallCalibration = {
    false, false, {0L, 0L}, {0, 0}, {4095, 4095}
};
bool magazineGxHallCalibrationInProgress = false;
#endif

bool magazine2PulseMonitorActive = false;
long magazine2PulseMonitorTargetSamples = 0;
long magazine2PulseMonitorCompletedSamples = 0;
unsigned long magazine2PulseMonitorNextSampleMs = 0;
bool magazine2PulseMonitorSavedStartChecks = false;
bool magazine2PulseMonitorPreviousStartChecks = false;
enum class MagazineAutomationPollStep : uint8_t { Mag1Slides, Mag2Slides };
MagazineAutomationPollStep magazineAutomationPollStep = MagazineAutomationPollStep::Mag1Slides;
bool magazineStatusKnown[2] = {false, false};
int lastMagazineStatus[2] = {0, 0};
int lastMagazineType[2]   = {0, 0};
static unsigned long lastMagazineCalRequestMs[2] = {0, 0};
static unsigned long lastMagazineFullCalRequestMs[2] = {0, 0};
static uint8_t lastMagazineCalMask[2] = {0, 0};
static bool magazineCalMaskKnown[2] = {false, false};
static constexpr unsigned long MAGAZINE_CAL_RETRY_MS = 2000UL;
int activeMagazineForLoad = 0;
volatile bool magazine1AlertPending = false;
volatile bool magazine2AlertPending = false;

void magazine1AlertISR() { magazine1AlertPending = true; }
void magazine2AlertISR() { magazine2AlertPending = true; }
bool moveGantryToLoadPositionBlocking(bool printStatus);
bool runGhomeBlocking(bool printStatus);
int resolveLoadMagazine(long loadValue);
bool executeMagazinePriorityLoadBlocking(int magazine, int requestedSlideNumber);
static void requestCalFromEsp(int holder, int magType);
int resolvePriorityEmptyMagazineSlot(const MagazineIrSensorModule& irModule, int requestedSlideNumber);
int countDetectedMagazineSlides(const MagazineIrSensorModule& irModule);
static void serviceMasterUartIntercept();
static void stopAutoLoadAndMoveToZero(const char* reason);
static bool anyMagazineCanAcceptSlideBlocking();

I2CInstance& getMagazinePort(int magazine) {
    return (magazine == 1) ? i2c1 : i2c2;
}

// Reads only the second half of IR sensors (indices 12-23, 1 I2C transfer).
// Used when only the lock sensor (index 23) is needed — avoids the full 2-transfer read.
bool requestHalfMagazineIr2Blocking(MagazineIrSensorModule& irModule, I2CInstance& i2cPort, unsigned long timeoutMs = 2000) {
    const unsigned long startMs = millis();
    const uint32_t previousSequence=irModule.sensorSequence();
    irModule.setSilent(true);
    while (!irModule.isIrRequestPending() && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
        irModule.loop();
        irModule.requestHalfIrValues(2);
    }
    while (irModule.isIrRequestPending() && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
        irModule.loop();
    }
    const bool ok=SensorFaultPolicy::fresh(previousSequence,irModule.sensorSequence(),irModule.sensorDataValid());
    if(!ok) {
        SensorFault::report("LIVO-MAG-010",irModule.getPcbID()==Stainer_Magzine1_PCB_ID?"HOLDER1":"HOLDER2",-1,"no_fresh_sensor_response");
        irModule.clearPendingRequests(); i2cPort.clearPendingData();
    }
    return ok;
}

bool requestFullMagazineIrValuesBlocking(MagazineIrSensorModule& irModule, I2CInstance& i2cPort, unsigned long timeoutMs = 5000) {
    const unsigned long startMs = millis();
    const uint32_t previousSequence=irModule.sensorSequence();

    irModule.setSilent(true);

    while (!irModule.isIrRequestPending() && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
        irModule.loop();
        irModule.requestAllIrValues();
    }

    while (irModule.isIrRequestPending() && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
        irModule.loop();
    }

    const bool ok=SensorFaultPolicy::fresh(previousSequence,irModule.sensorSequence(),irModule.sensorDataValid());
    if(!ok) {
        SensorFault::report("LIVO-MAG-010",irModule.getPcbID()==Stainer_Magzine1_PCB_ID?"HOLDER1":"HOLDER2",-1,"no_fresh_sensor_response");
        irModule.clearPendingRequests(); i2cPort.clearPendingData();
    }
    return ok;
}

bool requestMagazineCommandBlocking(I2CInstance& i2cPort, uint8_t pcbID, uint8_t command, uint8_t responseLength = 1, unsigned long timeoutMs = 1000) {
    const unsigned long startMs = millis();

    while (!i2cPort.requestSensorData(pcbID, command, responseLength) && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
    }

    while (!i2cPort.hasNewData() && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
    }

    if (!i2cPort.hasNewData()) {
        return false;
    }

    (void)i2cPort.getData();
    return true;
}

bool requestMagazineTriggerCommandBlocking(I2CInstance& i2cPort, uint8_t pcbID, uint8_t command, unsigned long timeoutMs = 1000) {
    return requestMagazineCommandBlocking(i2cPort, pcbID, command, 1, timeoutMs);
}

bool requestMagazineLockCommandBlocking(I2CInstance& i2cPort, uint8_t pcbID, unsigned long timeoutMs = 1000) {
    return requestMagazineCommandBlocking(i2cPort, pcbID, CMD_LOCK_MAGZINE, 1, timeoutMs);
}

bool requestMagazineUnlockCommandBlocking(I2CInstance& i2cPort, uint8_t pcbID, unsigned long timeoutMs = 1000) {
    return requestMagazineCommandBlocking(i2cPort, pcbID, CMD_UNLOCK_MAGZINE, 1, timeoutMs);
}

bool shouldStopMagazine2PulseMonitor(const MagazineIrSensorModule& irModule) {
    const long* irValues = irModule.getIrValues();
    return irValues[MAGAZINE2_LOCK_GUARD_INDEX] < MAGAZINE2_LOCK_GUARD_THRESHOLD;
}

void stopMagazine2PulseMonitor() {
    magazine2PulseMonitorActive = false;
    magazine2PulseMonitorTargetSamples = 0;
    magazine2PulseMonitorCompletedSamples = 0;
    magazine2PulseMonitorNextSampleMs = 0;
    Magazine1Ir.clearPendingRequests();
    Magazine2Ir.clearPendingRequests();
    i2c1.clearPendingData();
    i2c2.clearPendingData();
    if (magazine2PulseMonitorSavedStartChecks) {
        startMagzinechecks = magazine2PulseMonitorPreviousStartChecks;
        magazine2PulseMonitorSavedStartChecks = false;
    }
}

void startMagazine2PulseMonitor(long sampleCount) {
    if (sampleCount <= 0) {
        stopMagazine2PulseMonitor();
        Serial.println(" CMD");
        return;
    }

    magazine2PulseMonitorActive = true;
    magazine2PulseMonitorTargetSamples = sampleCount;
    magazine2PulseMonitorCompletedSamples = 0;
    magazine2PulseMonitorNextSampleMs = millis();
    magazine2PulseMonitorPreviousStartChecks = startMagzinechecks;
    magazine2PulseMonitorSavedStartChecks = true;
    startMagzinechecks = false;
    Magazine1Ir.clearPendingRequests();
    Magazine2Ir.clearPendingRequests();
    i2c1.clearPendingData();
    i2c2.clearPendingData();
    Serial.println(" CMD");
}

void serviceMagazine2PulseMonitor() {
    if (!magazine2PulseMonitorActive) {
        return;
    }

    const unsigned long now = millis();
    if ((long)(now - magazine2PulseMonitorNextSampleMs) < 0) { // wrap-safe
        return;
    }

    if (!requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
        Serial.println("MHP2 TRIG ON TIMEOUT");
        stopMagazine2PulseMonitor();
        return;
    }

    const bool readOk = requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2);
    if (!requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF)) {
        Serial.println("MHP2 TRIG OFF TIMEOUT");
        stopMagazine2PulseMonitor();
        return;
    }

    if (!readOk) {
        Serial.println("MHP2 TIMEOUT");
        stopMagazine2PulseMonitor();
        return;
    }

    if (shouldStopMagazine2PulseMonitor(Magazine2Ir)) {
        Serial.println(" CMD");
        stopMagazine2PulseMonitor();
        return;
    }

    magazine2PulseMonitorCompletedSamples++;
    if (magazine2PulseMonitorCompletedSamples >= magazine2PulseMonitorTargetSamples) {
        magazine2PulseMonitorCompletedSamples = 0;
    }

    magazine2PulseMonitorNextSampleMs = now + MAGAZINE2_SAMPLE_INTERVAL_MS;
}

void printIrValuesLine(const long* irValues, int count) {
    for (int i = 0; i < count; ++i) {
        Serial.print(irValues[i]);
        if (i < count - 1) {
            Serial.print(",");
        }
    }
    Serial.print(" CMD");
    Serial.println();
}

void printIrValuesCsvLine(long sampleNumber, const long* irValues, int count) {
    Serial.print(sampleNumber);
    Serial.print(",");
    for (int i = 0; i < count; ++i) {
        Serial.print(irValues[i]);
        if (i < count - 1) {
            Serial.print(",");
        }
    }
    Serial.print(" CMD");
    Serial.println();
}

void serviceGantryMotionWhileWaiting() {
    serviceMasterUartIntercept();
    GXMotor.loop();
    GYMotor.loop();
    GZMotor.loop();
    GRMotor.loop();
    XMotor.loop();
    limitSwitch.loop();
}

template <typename MotorType>
void moveMotorToAndWait(MotorType& motor, long position) {
    motor.moveTo(position);
    while (motor.isMoving()) {
        serviceGantryMotionWhileWaiting();
    }
}

template <typename MotorType>
void moveMotorRelativeAndWait(MotorType& motor, long delta) {
    motor.move(delta);
    while (motor.isMoving()) {
        serviceGantryMotionWhileWaiting();
    }
}

void clearMagazineBusState() {
    Magazine1Ir.clearPendingRequests();
    Magazine2Ir.clearPendingRequests();
    i2c1.clearPendingData();
    i2c2.clearPendingData();
}

void clearGantrySensorTransportState() {
    clearMagazineBusState();
}

bool readLidarDistanceBlocking(float& correctedDistanceMm, uint8_t sampleCount = 10) {
    clearGantrySensorTransportState();
    return lidarReader.readCorrectedDistanceMm(correctedDistanceMm, sampleCount);
}

bool readLidarGxPositionBlocking(float& gxPositionMm, uint8_t sampleCount = 10) {
    clearGantrySensorTransportState();
    return lidarReader.readGxPositionMm(gxPositionMm, sampleCount);
}

bool sampleCurrentGxLidarDistance(float& correctedDistanceMm, uint8_t sampleCount) {
    return readLidarDistanceBlocking(correctedDistanceMm, sampleCount);
}

// OLD GX HOMING (LIDAR-BASED) — kept for reference
// bool homeGxWithLidarBlocking(const char* errorPrefix) {
//     const long originalGxSpeed = GXMotor.getMaxSpeed();
//     long homingGxSpeed = originalGxSpeed + GX_HOME_HOMING_SPEED_BOOST;
//     if (homingGxSpeed < GX_HOME_HOMING_SPEED_MIN) {
//         homingGxSpeed = GX_HOME_HOMING_SPEED_MIN;
//     } else if (homingGxSpeed > GX_HOME_HOMING_SPEED_MAX) {
//         homingGxSpeed = GX_HOME_HOMING_SPEED_MAX;
//     }
//     GXMotor.setMaxSpeed(homingGxSpeed);
//
//     float currentDistanceMm = 0.0f;
//     if (!sampleCurrentGxLidarDistance(currentDistanceMm, GX_HOME_LIDAR_SAMPLE_COUNT)) {
//         GXMotor.setMaxSpeed(originalGxSpeed);
//         Serial.print(errorPrefix);
//         Serial.println(": lidar read failed");
//         return false;
//     }
//
//     const unsigned long startMs = millis();
//     while (millis() - startMs < GX_HOME_TIMEOUT_MS) {
//         if (currentDistanceMm >= GX_HOME_LIDAR_DISTANCE_MM) {
//             float confirmedDistanceMm = 0.0f;
//             if (!sampleCurrentGxLidarDistance(confirmedDistanceMm, GX_HOME_LIDAR_SAMPLE_COUNT)) {
//                 GXMotor.setMaxSpeed(originalGxSpeed);
//                 Serial.print(errorPrefix);
//                 Serial.println(": lidar confirm read failed");
//                 return false;
//             }
//
//             if (confirmedDistanceMm >= GX_HOME_LIDAR_DISTANCE_MM) {
//                 GXMotor.stop();
//                 GXMotor.setCurrentPosition(0);
//                 GXMotor.setMaxSpeed(originalGxSpeed);
//                 return true;
//             }
//
//             currentDistanceMm = confirmedDistanceMm;
//         }
//
//         const float remainingDistanceMm = GX_HOME_LIDAR_DISTANCE_MM - currentDistanceMm;
//         long stepMagnitude = (long)((remainingDistanceMm * GX_HOME_STEPS_PER_MM) + 0.5f);
//         if (stepMagnitude <= 0) {
//             stepMagnitude = 1;
//         }
//
//         moveMotorRelativeAndWait(GXMotor, stepMagnitude);
//
//         if (!sampleCurrentGxLidarDistance(currentDistanceMm, GX_HOME_LIDAR_SAMPLE_COUNT)) {
//             GXMotor.setMaxSpeed(originalGxSpeed);
//             Serial.print(errorPrefix);
//             Serial.println(": lidar read failed during homing");
//             return false;
//         }
//     }
//
//     GXMotor.setMaxSpeed(originalGxSpeed);
//     Serial.print(errorPrefix);
//     Serial.println(": timeout");
//     return false;
// }

// PREVIOUS GX HOMING — four-pass sequence: HOME → back 2000 → HOME → back 1000 → HOME → back 500 → HOME
// static bool gxApproachLimitBlocking(const char* errorPrefix) {
//     if (digitalRead(PA0) == LOW) return true;
//     const unsigned long startMs = millis();
//     while (digitalRead(PA0) != LOW) {
//         if (!GXMotor.isMoving()) { GXMotor.move(2000); }
//         GXMotor.loop();
//         limitSwitch.loop();
//         if (millis() - startMs > GX_HOME_TIMEOUT_MS) {
//             GXMotor.stop();
//             while (GXMotor.isMoving()) { GXMotor.loop(); }
//             Serial.print(errorPrefix);
//             Serial.println(": timeout");
//             return false;
//         }
//     }
//     GXMotor.stop();
//     while (GXMotor.isMoving()) { GXMotor.loop(); }
//     return true;
// }
//
// bool homeGxWithLidarBlocking(const char* errorPrefix) {
//     if (!gxApproachLimitBlocking(errorPrefix)) return false;
//     GXMotor.setCurrentPosition(0);
//     moveMotorRelativeAndWait(GXMotor, -2000);
//     if (!gxApproachLimitBlocking(errorPrefix)) return false;
//     GXMotor.setCurrentPosition(0);
//     moveMotorRelativeAndWait(GXMotor, -1000);
//     if (!gxApproachLimitBlocking(errorPrefix)) return false;
//     GXMotor.setCurrentPosition(0);
//     moveMotorRelativeAndWait(GXMotor, -500);
//     if (!gxApproachLimitBlocking(errorPrefix)) return false;
//     GXMotor.setCurrentPosition(0);
//     return true;
// }

// NEW GX HOMING — full speed continuous to first limit click, back 2500, slow 500-step approach
bool homeGxWithLidarBlocking(const char* errorPrefix) {
    const long savedSpeed = GXMotor.getMaxSpeed();
    const long savedAccel = GXMotor.getAcceleration();

    // Phase 1: continuous full-speed approach with low acceleration so the motor
    // never reaches a speed high enough to overshoot badly on endstop trigger.
    GXMotor.setMaxSpeed(GX_HOME_HOMING_SPEED_MAX);
    GXMotor.setAcceleration(GX_HOME_APPROACH_ACCELERATION);
    GXMotor.move(2000000000L);
    const unsigned long startMs = millis();
    while (digitalRead(PA0) != LOW) {
        GXMotor.loop();
        limitSwitch.loop();
        if (millis() - startMs > GX_HOME_TIMEOUT_MS) {
            GXMotor.setCurrentPosition(GXMotor.getCurrentPosition()); // zero speed, cancel move
            GXMotor.setMaxSpeed(savedSpeed);
            GXMotor.setAcceleration(savedAccel);
            Serial.print(errorPrefix); Serial.println(": phase1 timeout");
            SensorFault::report("LIVO-GAN-002","GX",digitalRead(PA0),"home_input_never_active_cause_unconfirmed");
            return false;
        }
    }
    GXMotor.setCurrentPosition(GXMotor.getCurrentPosition()); // zero speed, cancel move

    // Phase 2: back off 2500 steps
    GXMotor.setMaxSpeed(savedSpeed);
    moveMotorRelativeAndWait(GXMotor, -2500);

    if(digitalRead(PA0)==LOW) {
        GXMotor.setMaxSpeed(savedSpeed); GXMotor.setAcceleration(savedAccel);
        SensorFault::report("LIVO-SEN-022","GX",LOW,"active_after_backoff_sensor_or_motion_unconfirmed");
        return false;
    }

    // Phase 3: slow 200-step chunks for precise home detection
    GXMotor.setMaxSpeed(GX_HOME_HOMING_SPEED_MIN);
    GXMotor.setAcceleration(savedAccel);
    const unsigned long startMs2 = millis();
    while (digitalRead(PA0) != LOW) {
        if (millis() - startMs2 > GX_HOME_TIMEOUT_MS) {
            GXMotor.setCurrentPosition(GXMotor.getCurrentPosition());
            GXMotor.setMaxSpeed(savedSpeed);
            GXMotor.setAcceleration(savedAccel);
            Serial.print(errorPrefix); Serial.println(": phase3 timeout");
            SensorFault::report("LIVO-GAN-003","GX",digitalRead(PA0),"home_input_never_active_cause_unconfirmed");
            return false;
        }
        GXMotor.move(200);
        while (GXMotor.isMoving() && digitalRead(PA0) != LOW) {
            GXMotor.loop();
            limitSwitch.loop();
        }
    }
    GXMotor.setCurrentPosition(0);

    GXMotor.setMaxSpeed(savedSpeed);
    GXMotor.setAcceleration(savedAccel);
    return true;
}


// PREVIOUS GZ HOMING — four-pass sequence
// static bool gzApproachLimitBlocking(const char* errorPrefix) {
//     if (digitalRead(Lim2) == LOW) return true;
//     const unsigned long startMs = millis();
//     while (digitalRead(Lim2) != LOW) {
//         if (millis() - startMs > GX_HOME_TIMEOUT_MS) {
//             GZMotor.stop();
//             while (GZMotor.isMoving()) { GZMotor.loop(); }
//             Serial.print(errorPrefix);
//             Serial.println(": timeout");
//             return false;
//         }
//         GZMotor.move(-GZ_HOME_STEP_CHUNK);
//         while (GZMotor.isMoving() && digitalRead(Lim2) != LOW) {
//             GZMotor.loop();
//         }
//     }
//     GZMotor.stop();
//     while (GZMotor.isMoving()) { GZMotor.loop(); }
//     return true;
// }
//
// bool homeGzWithHallBlocking(const char* errorPrefix) {
//     const long originalGzSpeed = GZMotor.getMaxSpeed();
//     GZMotor.setMaxSpeed(GZ_HOME_HOMING_SPEED);
//     if (!gzApproachLimitBlocking(errorPrefix)) { GZMotor.setMaxSpeed(originalGzSpeed); return false; }
//     GZMotor.setCurrentPosition(0);
//     moveMotorRelativeAndWait(GZMotor, 6000);
//     if (!gzApproachLimitBlocking(errorPrefix)) { GZMotor.setMaxSpeed(originalGzSpeed); return false; }
//     GZMotor.setCurrentPosition(0);
//     moveMotorRelativeAndWait(GZMotor, 3000);
//     if (!gzApproachLimitBlocking(errorPrefix)) { GZMotor.setMaxSpeed(originalGzSpeed); return false; }
//     GZMotor.setCurrentPosition(0);
//     moveMotorRelativeAndWait(GZMotor, 1500);
//     if (!gzApproachLimitBlocking(errorPrefix)) { GZMotor.setMaxSpeed(originalGzSpeed); return false; }
//     GZMotor.setCurrentPosition(0);
//     GZMotor.setMaxSpeed(originalGzSpeed);
//     return true;
// }

// NEW GZ HOMING — mirrors GX: low-accel continuous approach, back off 2500, slow 200-step approach
bool homeGzWithHallBlocking(const char* errorPrefix) {
    const long savedSpeed = GZMotor.getMaxSpeed();
    const long savedAccel = GZMotor.getAcceleration();

    // Phase 1: continuous in -ve direction with low acceleration until Lim2 (PC0) goes LOW
    GZMotor.setMaxSpeed(GZ_HOME_HOMING_SPEED);
    GZMotor.setAcceleration(GX_HOME_APPROACH_ACCELERATION);
    GZMotor.move(-2000000000L);
    const unsigned long startMs = millis();
    while (digitalRead(Lim2) != LOW) {
        GZMotor.loop();
        limitSwitch.loop();
        if (millis() - startMs > GX_HOME_TIMEOUT_MS) {
            GZMotor.setCurrentPosition(GZMotor.getCurrentPosition());
            GZMotor.setMaxSpeed(savedSpeed);
            GZMotor.setAcceleration(savedAccel);
            Serial.print(errorPrefix); Serial.println(": phase1 timeout");
            SensorFault::report("LIVO-GAN-004","GZ",digitalRead(Lim2),"home_input_never_active_cause_unconfirmed");
            return false;
        }
    }
    GZMotor.setCurrentPosition(GZMotor.getCurrentPosition());

    // Phase 2: back off 2500 steps
    GZMotor.setMaxSpeed(savedSpeed);
    GZMotor.setAcceleration(savedAccel);
    moveMotorRelativeAndWait(GZMotor, 2500);

    if(digitalRead(Lim2)==LOW) {
        GZMotor.setMaxSpeed(savedSpeed); GZMotor.setAcceleration(savedAccel);
        SensorFault::report("LIVO-SEN-022","GZ",LOW,"active_after_backoff_sensor_or_motion_unconfirmed");
        return false;
    }

    // Phase 3: slow 200-step chunks until Lim2 triggers again for precise home
    GZMotor.setMaxSpeed(GZ_HOME_HOMING_SPEED);
    GZMotor.setAcceleration(savedAccel);
    const unsigned long startMs2 = millis();
    while (digitalRead(Lim2) != LOW) {
        if (millis() - startMs2 > GX_HOME_TIMEOUT_MS) {
            GZMotor.setCurrentPosition(GZMotor.getCurrentPosition());
            GZMotor.setMaxSpeed(savedSpeed);
            GZMotor.setAcceleration(savedAccel);
            Serial.print(errorPrefix); Serial.println(": phase3 timeout");
            SensorFault::report("LIVO-GAN-005","GZ",digitalRead(Lim2),"home_input_never_active_cause_unconfirmed");
            return false;
        }
        GZMotor.move(-200);
        while (GZMotor.isMoving() && digitalRead(Lim2) != LOW) {
            GZMotor.loop();
            limitSwitch.loop();
        }
    }
    GZMotor.setCurrentPosition(0);

    GZMotor.setMaxSpeed(savedSpeed);
    GZMotor.setAcceleration(savedAccel);
    return true;
}

static bool gr_home_single_pass(const char* statusPrefix, bool printAlreadyAtHome) {
    const int targetSensorIndex = 2;               // IR3 is third element in irsensorPins[]
    const long homeStepChunk = 100;                // Smaller chunks so we re-check IR3 sooner
    const int homeOffsetSteps = 0;               // Final offset after the IR3 edge is found
    const unsigned long timeoutMs = 10000;
    const unsigned long fastIrReadIntervalMicros = 2000; // 500 reads/sec during GR homing

    const unsigned long savedIrReadIntervalMicros = limitSwitch.getMinReadIntervalMicros();
    limitSwitch.setMinReadIntervalMicros(fastIrReadIntervalMicros);

    unsigned long startMs = millis();
    int* irVals = limitSwitch.getSmoothedSensorValues();
    if (irVals == nullptr || !SensorFaultPolicy::adcValid(irVals[targetSensorIndex])) {
        limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);
        GRMotor.stop();
        Serial.print(statusPrefix);
        Serial.println(": IR read failed");
        SensorFault::report("LIVO-SEN-021","GR-IR3",-1,"invalid_or_missing_reading");
        return false;
    }

    if (irVals[targetSensorIndex] < GR_HOME_IR3_THRESHOLD) {
        limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);
        GRMotor.stop();
        while (GRMotor.isMoving()) {
            GRMotor.loop();
        }
        GRMotor.setCurrentPosition(GRMotor.getCurrentPosition());
        if (printAlreadyAtHome) {
            Serial.print(statusPrefix);
            Serial.println(": input active; verification pending");
        }
        return true;
    }

    while (irVals[targetSensorIndex] >= GR_HOME_IR3_THRESHOLD) {
        if (!GRMotor.isMoving()) {
            GRMotor.move(homeStepChunk);
        }
        GRMotor.loop();
        limitSwitch.loop();
        irVals = limitSwitch.getSmoothedSensorValues();

        if (irVals == nullptr || !SensorFaultPolicy::adcValid(irVals[targetSensorIndex])) {
            limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);
            GRMotor.stop();
            Serial.print(statusPrefix);
            Serial.println(": lost IR values");
            return false;
        }

        if (irVals[targetSensorIndex] < GR_HOME_IR3_THRESHOLD) {
            GRMotor.stop();
        }

        if (millis() - startMs > timeoutMs) {
            limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);
            GRMotor.stop();
            Serial.print(statusPrefix);
            Serial.println(": timeout");
            SensorFault::report("LIVO-SEN-023","GR-IR3",irVals[targetSensorIndex],"home_never_active_cause_unconfirmed");
            return false;
        }
    }

    GRMotor.stop();
    unsigned long stopStart = millis();
    while (GRMotor.isMoving()) {
        GRMotor.loop();
        limitSwitch.loop();
        if (millis() - stopStart > timeoutMs) {
            limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);
            GRMotor.stop();
            Serial.print(statusPrefix);
            Serial.println(": stop timeout");
            return false;
        }
    }

    GRMotor.move(homeOffsetSteps);
    unsigned long moveStart = millis();
    while (GRMotor.isMoving()) {
        GRMotor.loop();
        limitSwitch.loop();
        if (millis() - moveStart > timeoutMs) {
            limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);
            GRMotor.stop();
            Serial.print(statusPrefix);
            Serial.println(": offset timeout");
            return false;
        }
    }

    GRMotor.stop();
    GRMotor.setCurrentPosition(GRMotor.getCurrentPosition());
    limitSwitch.setMinReadIntervalMicros(savedIrReadIntervalMicros);

    Serial.print(statusPrefix);
    Serial.println(": done");
    return true;
}

bool homeGrWithIr3Blocking(const char* statusPrefix, bool printAlreadyAtHome) {
    // First pass — find IR3 edge from current position.
    if (!gr_home_single_pass(statusPrefix, printAlreadyAtHome)) {
        return false;
    }
    // Back off so the second pass approaches the edge from a consistent direction.
    moveMotorRelativeAndWait(GRMotor, -2500);
    limitSwitch.loop();
    int* released=limitSwitch.getSmoothedSensorValues();
    if(!released || !SensorFaultPolicy::adcValid(released[2]) || released[2]<GR_HOME_IR3_THRESHOLD) {
        SensorFault::report("LIVO-SEN-022","GR-IR3",released?released[2]:-1,"active_after_backoff_sensor_or_motion_unknown");
        return false;
    }
    // Second pass — re-home for a precise final position.
    if(!gr_home_single_pass(statusPrefix, false)) return false;
    GRMotor.setCurrentPosition(0);return true;
}


static bool homeGyIr4Blocking() {
    const long savedSpeed=GYMotor.getMaxSpeed(); GYMotor.setMaxSpeed(GYHO_SPEED);
    auto read=[]()->int {limitSwitch.loop();int* values=limitSwitch.getSmoothedSensorValues();return values?values[RST_IR4_INDEX]:-1;};
    auto fail=[&](const char* code,int raw,const char* reason)->bool {
        GYMotor.setCurrentPosition(GYMotor.getCurrentPosition());GYMotor.setMaxSpeed(savedSpeed);
        SensorFault::report(code,"GY-IR4",raw,reason);return false;
    };
    int raw=read();
    if(!SensorFaultPolicy::adcValid(raw)) return fail("LIVO-SEN-021",raw,"invalid_home_sample");
    if(raw>=GYHO_IR4_THRESHOLD) {
        // Bounded opposite-direction release of an initially active input.
        GYMotor.move(-GYHO_STEP_CHUNK);const uint32_t releaseAt=millis();
        while(GYMotor.isMoving() && (uint32_t)(millis()-releaseAt)<5000UL) {
            GYMotor.loop();raw=read();
            if(!SensorFaultPolicy::adcValid(raw)) return fail("LIVO-SEN-021",raw,"invalid_release_sample");
            if(raw<GYHO_IR4_THRESHOLD) break;
        }
        GYMotor.setCurrentPosition(GYMotor.getCurrentPosition());
        if(raw>=GYHO_IR4_THRESHOLD) return fail("LIVO-SEN-022",raw,"home_release_not_confirmed_sensor_or_motion_unknown");
    }
    const uint32_t start=millis();
    while(raw<GYHO_IR4_THRESHOLD) {
        if(!GYMotor.isMoving()) GYMotor.move(GYHO_STEP_CHUNK);
        GYMotor.loop();raw=read();
        if(!SensorFaultPolicy::adcValid(raw)) return fail("LIVO-SEN-021",raw,"invalid_home_sample");
        if((uint32_t)(millis()-start)>GYHO_TIMEOUT_MS) return fail("LIVO-GAN-006",raw,"home_transition_failed_cause_unconfirmed");
    }
    GYMotor.setCurrentPosition(0);GYMotor.setMaxSpeed(savedSpeed);return true;
}

int* readCurrentIrValues() {
    limitSwitch.loop();
    return limitSwitch.getSmoothedSensorValues();
}

bool isRstIr1Active(int value) {
    return SensorFaultPolicy::adcValid(value) && value < RST_IR1_ACTIVE_THRESHOLD;
}

bool isRstIr2Active(int value) {
    return SensorFaultPolicy::adcValid(value) && value < RST_IR2_ACTIVE_THRESHOLD;
}

bool isRstIr3Active(int value) {
    return SensorFaultPolicy::adcValid(value) && value < RST_IR3_ACTIVE_THRESHOLD;
}

bool isRstIr4Active(int value) {
    return SensorFaultPolicy::adcValid(value) && value < RST_IR4_ACTIVE_THRESHOLD;
}

void zeroAllGantryAxesCurrentPosition() {
    GXMotor.setCurrentPosition(0);
    GYMotor.setCurrentPosition(0);
    GZMotor.setCurrentPosition(0);
    GRMotor.setCurrentPosition(0);
}

bool moveGyNegativeUntilIr1ActiveBlocking() {
    const unsigned long startMs = millis();
    bool gyNegativeMoveStarted = false;

    while (millis() - startMs < RST_GY_NEGATIVE_TIMEOUT_MS) {
        int* irValues = readCurrentIrValues();
        if (gyNegativeMoveStarted && irValues != nullptr && isRstIr1Active(irValues[RST_IR1_INDEX])) {
            GYMotor.stop();
            return true;
        }

        moveMotorRelativeAndWait(GYMotor, RST_GY_NEGATIVE_STEP_CHUNK);
        gyNegativeMoveStarted = true;
    }

    GYMotor.stop();
    Serial.println("RST: GY move timeout");
    return false;
}

MagazineIrSensorModule& getMagazineIrModule(int magazine);   // forward declaration
int detectMagazineType(const MagazineIrSensorModule& irModule); // forward declaration

// Non-blocking auto-load state machine
static bool autoLoadActive = false;
static int  autoLoadMagazine = 0;
static bool autoLoadStopRequested = false;
enum class AutoLoadState : uint8_t { WaitIr2Clear, WaitIr2Trigger, Debounce };
static AutoLoadState autoLoadState = AutoLoadState::WaitIr2Clear;
static unsigned long autoLoadDebounceMs = 0;

// Detects which holder has a magazine inserted. Returns 1, 2, or 0 (none).
// Prefers `preferred` if it still has a magazine; otherwise picks whichever does.
static int pickAvailableMagazine(int preferred) {
    int mag1Type = 0, mag2Type = 0;

    clearMagazineBusState();
    if (requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
        if (requestFullMagazineIrValuesBlocking(Magazine1Ir, i2c1)) {
            mag1Type = detectMagazineType(Magazine1Ir);
        }
        requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
    }
    clearMagazineBusState();
    if (requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
        if (requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2)) {
            mag2Type = detectMagazineType(Magazine2Ir);
        }
        requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
    }

    if (preferred == 1 && mag1Type != 0) return 1;
    if (preferred == 2 && mag2Type != 0) return 2;
    if (mag1Type != 0) return 1;
    if (mag2Type != 0) return 2;
    return 0;
}

static void requestAutoLoadStop() {
    if (autoLoadActive) {
        // A blocking load operation checks this flag at its safe interruption
        // points before the main loop performs the final zero-position move.
        autoLoadStopRequested = true;
        return;
    }
    stopAutoLoadAndMoveToZero("stop requested");
}

static void stopAutoLoadAndMoveToZero(const char* reason) {
    static bool moveToZeroActive = false;
    if (moveToZeroActive) return;
    moveToZeroActive = true;
    autoLoadActive = false;
    autoLoadMagazine = 0;
    autoLoadState = AutoLoadState::WaitIr2Clear;
    autoLoadStopRequested = false;
    Serial.print("LOAD: stopped - ");
    Serial.println(reason);

    // Stop Run returns to the established coordinate origin. It does not run
    // GHOME or redefine coordinates from sensors.
    GXMotor.moveTo(0);
    GYMotor.moveTo(0);
    GZMotor.moveTo(0);
    GRMotor.moveTo(0);
    while (GXMotor.isMoving() || GYMotor.isMoving() ||
           GZMotor.isMoving() || GRMotor.isMoving()) {
        serviceGantryMotionWhileWaiting();
    }
    Serial.println("LOADSTOP: GX/GY/GZ/GR at 0");
    moveToZeroActive = false;
}

static void tickAutoLoad() {
    if (!autoLoadActive) return;
    if (autoLoadStopRequested) {
        stopAutoLoadAndMoveToZero("stop requested");
        return;
    }
    const int ir2val = analogRead(irsensorPins[RST_IR2_INDEX]);
    switch (autoLoadState) {
    case AutoLoadState::WaitIr2Clear:
        if (ir2val >= RST_IR2_ACTIVE_THRESHOLD) {
            autoLoadState = AutoLoadState::WaitIr2Trigger;
            Serial.println("WAITING FOR SLIDE");
        }
        break;
    case AutoLoadState::WaitIr2Trigger:
        if (ir2val < RST_IR2_ACTIVE_THRESHOLD) {
            autoLoadDebounceMs = millis();
            autoLoadState = AutoLoadState::Debounce;
        }
        break;
    case AutoLoadState::Debounce:
        if (millis() - autoLoadDebounceMs >= 30) {
            if (analogRead(irsensorPins[RST_IR2_INDEX]) < RST_IR2_ACTIVE_THRESHOLD) {
                // Re-detect available magazine — user may have swapped after both filled.
                const int target = pickAvailableMagazine(autoLoadMagazine);
                if (target != 0) {
                    autoLoadMagazine = target;
                    const bool loaded = executeMagazinePriorityLoadBlocking(autoLoadMagazine, 1);
                    if (autoLoadStopRequested) {
                        stopAutoLoadAndMoveToZero("stop requested");
                        return;
                    }
                    if (!loaded && !anyMagazineCanAcceptSlideBlocking()) {
                        stopAutoLoadAndMoveToZero("all magazine slots full");
                        return;
                    }
                } else {
                    Serial.println("LOAD: waiting for magazine in either holder");
                }
            }
            autoLoadState = AutoLoadState::WaitIr2Clear;
        }
        break;
    }
}

// Like requestMagazineCommandBlocking but returns the raw data buffer instead of discarding it.
// Caller must copy the data before the next I2C request overwrites the buffer.
static const uint8_t* requestMagazineDataBlocking(I2CInstance& i2cPort, uint8_t pcbID,
                                                   uint8_t command, uint8_t responseLength,
                                                   unsigned long timeoutMs = 5000) {
    const unsigned long startMs = millis();
    while (!i2cPort.requestSensorData(pcbID, command, responseLength) &&
           (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
    }
    while (!i2cPort.hasNewData() && (millis() - startMs < timeoutMs)) {
        i2cPort.loop();
    }
    if (!i2cPort.hasNewData()) return nullptr;
    return i2cPort.getData();
}

static bool runMagazineUartLinkTest(uint8_t holder) {
    I2CInstance& port = (holder == 1) ? i2c1 : i2c2;
    MagazineIrSensorModule& irModule = getMagazineIrModule(holder);
    const uint8_t expectedId = (holder == 1) ? Stainer_Magzine1_PCB_ID : Stainer_Magzine2_PCB_ID;
    // MUTALL may arrive between two background status polls. Cancel the
    // module-side request as well as the transport buffer so its reply cannot
    // be mistaken for the link-test response.
    irModule.clearPendingRequests();
    port.clearPendingData();
    const unsigned long startedUs = micros();
    const uint8_t* reply = requestMagazineDataBlocking(port, expectedId, CMD_UART_LINK_TEST, 3, 1000);
    const unsigned long elapsedUs = micros() - startedUs;

    Serial.print("UARTTEST H"); Serial.print(holder); Serial.print(' ');
    if (!reply) {
        Serial.println("FAIL TIMEOUT");
        return false;
    }
    if (reply[0] != expectedId || reply[1] != 0x55 || reply[2] != 0x01) {
        Serial.print("FAIL BAD_REPLY ID=0x"); Serial.print(reply[0], HEX);
        Serial.print(" MARK=0x"); Serial.print(reply[1], HEX);
        Serial.print(" VER="); Serial.println(reply[2]);
        return false;
    }
    Serial.print("PASS ID=0x"); Serial.print(reply[0], HEX);
    Serial.print(" PROTO="); Serial.print(reply[2]);
    Serial.print(" RTT_US="); Serial.println(elapsedUs);
    return true;
}

static bool reportMagazineFirmwareVersion(uint8_t holder) {
    if (holder < 1 || holder > 2) {
        Serial.println("MAGID: use MAGID <1|2>");
        return false;
    }
    I2CInstance& port = (holder == 1) ? i2c1 : i2c2;
    MagazineIrSensorModule& irModule = getMagazineIrModule(holder);
    irModule.clearPendingRequests();
    port.clearPendingData();
    const uint8_t* reply = requestMagazineDataBlocking(
        port, irModule.getPcbID(), CMD_FIRMWARE_IDENTITY, 48, 1500);
    if (!reply) {
        // Backward-compatible fallback for a holder that has not yet received
        // the identity-capable UART application.
        port.clearPendingData();
        reply = requestMagazineDataBlocking(
            port, irModule.getPcbID(), CMD_FIRMWARE_VERSION, 4, 1500);
        if (!reply) {
            Serial.print("MAGID"); Serial.print(holder); Serial.println(": no response");
            return false;
        }
    }
    const uint8_t expectedId = holder == 1 ? Stainer_Magzine1_PCB_ID : Stainer_Magzine2_PCB_ID;
    if (reply[0] != expectedId) {
        Serial.print("MAGID"); Serial.print(holder); Serial.println(": wrong holder response");
        return false;
    }
    // Keep the same telemetry shape used by every directly connected PCB so
    // ESP32 can verify this holder against the active device-release manifest.
    Serial.print("ID:"); Serial.print(reply[0]);
    Serial.print(",IDHEX:"); Serial.print(reply[0], HEX);
    Serial.print(",FW:V"); Serial.print(reply[1]); Serial.print('.');
    Serial.print(reply[2]); Serial.print('.'); Serial.print(reply[3]);
    if (port.getDataLength() == 49) {
        String build;
        for (uint8_t i = 4; i < 24; ++i) build += (char)reply[i];
        build.trim();
        String uid;
        for (uint8_t i = 24; i < 48; ++i) uid += (char)reply[i];
        Serial.print(",BUILD:"); Serial.print(build);
        Serial.print(",UID:"); Serial.println(uid);
    } else {
        Serial.print(",BUILD:LEGACY,UID:HOLDER"); Serial.println(holder);
    }
    return true;
}

// Magazine application OTA is relayed over the dedicated holder UARTs. The
// bootloader remains protected at 0x08000000; only the offset application at
// 0x08020000 is erased and programmed.
struct MagazineFlashSession {
    bool active = false;
    bool savedPolling = false;
    uint8_t holder = 0;
    uint8_t pcbId = 0;
    I2CInstance* port = nullptr;
    uint32_t size = 0;
    uint32_t received = 0;
    uint16_t sequence = 0;
};

static MagazineFlashSession magazineFlash;

static uint8_t magazineFlashCrc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) crc ^= data[i];
    return crc;
}

static void magazinePutU16(uint8_t* out, uint16_t value) {
    out[0] = (uint8_t)(value >> 8); out[1] = (uint8_t)value;
}

static void magazinePutU32(uint8_t* out, uint32_t value) {
    out[0] = (uint8_t)(value >> 24); out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8); out[3] = (uint8_t)value;
}

static bool magazineWritePacket(I2CInstance& port, uint8_t pcbId,
                                const uint8_t* packet, uint8_t length) {
    if (!packet || length == 0) return false;
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (port.writeCommandData(pcbId, packet[0], packet + 1, length - 1)) return true;
        delay(10);
    }
    return false;
}

static bool magazineReadFlashStatus(I2CInstance& port, uint8_t pcbId,
                                    uint8_t& status, uint8_t& error,
                                    uint32_t& received, uint32_t& expected,
                                    unsigned long timeoutMs = 2000) {
    port.clearPendingData();
    const uint8_t* reply = requestMagazineDataBlocking(
        port, pcbId, CMD_MAGFW_STATUS, 10, timeoutMs);
    if (!reply) return false;
    status = reply[0]; error = reply[1];
    received = ((uint32_t)reply[2] << 24) | ((uint32_t)reply[3] << 16) |
               ((uint32_t)reply[4] << 8) | reply[5];
    expected = ((uint32_t)reply[6] << 24) | ((uint32_t)reply[7] << 16) |
               ((uint32_t)reply[8] << 8) | reply[9];
    return true;
}

static void endMagazineFlashSession() {
    if (magazineFlash.active || magazineFlash.port) {
        startMagzinechecks = magazineFlash.savedPolling;
    }
    magazineFlash = MagazineFlashSession();
}

static bool reportMagazineFlashStatus(uint8_t holder) {
    if (holder < 1 || holder > 2) return false;
    I2CInstance& port = holder == 1 ? i2c1 : i2c2;
    MagazineIrSensorModule& module = getMagazineIrModule(holder);
    uint8_t status = 0, error = 0; uint32_t received = 0, expected = 0;
    if (!magazineReadFlashStatus(port, module.getPcbID(), status, error, received, expected)) {
        Serial.print("MAGFWSTAT"); Serial.print(holder); Serial.println(": no response");
        return false;
    }
    Serial.print("MAGFWSTAT"); Serial.print(holder);
    Serial.print(": status="); Serial.print(status); Serial.print(" err="); Serial.print(error);
    Serial.print(" received="); Serial.print(received); Serial.print(" expected="); Serial.println(expected);
    return true;
}

static bool beginMagazineFlash(uint8_t holder, uint32_t size, uint32_t crc) {
    if (holder < 1 || holder > 2 || size < 8 || (size & 3U) || crc == 0) {
        Serial.println("MAGFAIL bad-args"); return false;
    }
    endMagazineFlashSession();
    magazineFlash.savedPolling = startMagzinechecks;
    startMagzinechecks = false;
    Magazine1Ir.clearPendingRequests(); Magazine2Ir.clearPendingRequests();
    I2CInstance& port = holder == 1 ? i2c1 : i2c2;
    MagazineIrSensorModule& module = getMagazineIrModule(holder);
    const uint8_t pcbId = module.getPcbID();
    uint8_t status = 0, error = 0; uint32_t received = 0, expected = 0;
    if (!magazineReadFlashStatus(port, pcbId, status, error, received, expected)) {
        Serial.println("MAGFAIL pre-status-no-response"); endMagazineFlashSession(); return false;
    }
    if (status == MAGFW_STATUS_IDLE || status == MAGFW_STATUS_UNSUPPORTED) {
        const uint8_t reboot[] = { CMD_MAGFW_REBOOT };
        if (!magazineWritePacket(port, pcbId, reboot, sizeof(reboot))) {
            Serial.println("MAGFAIL reboot-no-ack"); endMagazineFlashSession(); return false;
        }
        delay(750);
        const unsigned long deadline = millis() + 8000UL;
        bool ready = false;
        while ((long)(millis() - deadline) < 0) {
            if (magazineReadFlashStatus(port, pcbId, status, error, received, expected, 500) &&
                status == MAGFW_STATUS_READY) { ready = true; break; }
            delay(100);
        }
        if (!ready) { Serial.println("MAGFAIL bootloader-no-response"); endMagazineFlashSession(); return false; }
    }

    uint8_t begin[13] = { CMD_MAGFW_BEGIN };
    magazinePutU32(begin + 1, size); magazinePutU32(begin + 5, crc);
    magazinePutU32(begin + 9, MAGFW_APP_BASE_ADDRESS);
    if (!magazineWritePacket(port, pcbId, begin, sizeof(begin))) {
        Serial.println("MAGFAIL begin-no-ack"); endMagazineFlashSession(); return false;
    }
    delay(100);
    if (!magazineReadFlashStatus(port, pcbId, status, error, received, expected) ||
        status != MAGFW_STATUS_READY || error != MAGFW_ERR_NONE || expected != size) {
        Serial.println("MAGFAIL begin-status"); endMagazineFlashSession(); return false;
    }
    const uint8_t erase[] = { CMD_MAGFW_ERASE };
    if (!magazineWritePacket(port, pcbId, erase, sizeof(erase))) {
        Serial.println("MAGFAIL erase-no-ack"); endMagazineFlashSession(); return false;
    }
    delay(3000); // STM32F446 single-bank erase blocks the holder UART ISR.
    const unsigned long eraseDeadline = millis() + 70000UL;
    bool eraseReady = false;
    while ((long)(millis() - eraseDeadline) < 0) {
        if (magazineReadFlashStatus(port, pcbId, status, error, received, expected, 500)) {
            if (status == MAGFW_STATUS_RECEIVING && error == MAGFW_ERR_NONE) {
                eraseReady = true; break;
            }
            if (status == MAGFW_STATUS_ERROR) break;
        }
        delay(250);
    }
    if (!eraseReady) { Serial.println("MAGFAIL erase-status"); endMagazineFlashSession(); return false; }

    magazineFlash.active = true; magazineFlash.holder = holder;
    magazineFlash.pcbId = pcbId; magazineFlash.port = &port;
    magazineFlash.size = size; magazineFlash.received = 0; magazineFlash.sequence = 0;
    Serial.println("MAGRDY");
    return true;
}

static bool writeMagazineFlashData(uint32_t offset, const uint8_t* data, uint16_t length) {
    if (!magazineFlash.active || !data || length == 0 || length > 20 ||
        (length & 3U) || offset != magazineFlash.received || offset + length > magazineFlash.size) {
        Serial.println("MAGFAIL data-args"); return false;
    }
    uint8_t packet[29] = { CMD_MAGFW_DATA };
    magazinePutU16(packet + 1, magazineFlash.sequence);
    magazinePutU32(packet + 3, offset);
    packet[7] = (uint8_t)length;
    memcpy(packet + 8, data, length);
    packet[8 + length] = magazineFlashCrc8(packet, 8 + length);
    if (!magazineWritePacket(*magazineFlash.port, magazineFlash.pcbId,
                             packet, (uint8_t)(9 + length))) {
        Serial.println("MAGFAIL data-write"); return false;
    }
    uint8_t status = 0, error = 0; uint32_t received = 0, expected = 0;
    const uint32_t target = offset + length;
    const unsigned long deadline = millis() + 2000UL;
    do {
        delay(3);
        if (magazineReadFlashStatus(*magazineFlash.port, magazineFlash.pcbId,
                                    status, error, received, expected, 250) &&
            status == MAGFW_STATUS_RECEIVING && error == MAGFW_ERR_NONE && received == target) {
            magazineFlash.received = target; ++magazineFlash.sequence;
            Serial.print("MAGACK "); Serial.println(target);
            return true;
        }
    } while ((long)(millis() - deadline) < 0);
    Serial.println("MAGFAIL data-status"); return false;
}

static int magazineHexNibble(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
}

static bool writeMagazineFlashDataLine(const String& arguments) {
    const int first = arguments.indexOf(' ');
    const int second = first < 0 ? -1 : arguments.indexOf(' ', first + 1);
    if (first < 0 || second < 0) { Serial.println("MAGFAIL data-line"); return false; }
    const uint32_t offset = (uint32_t)strtoul(arguments.substring(0, first).c_str(), nullptr, 10);
    const uint16_t length = (uint16_t)arguments.substring(first + 1, second).toInt();
    String hex = arguments.substring(second + 1); hex.trim();
    if (length == 0 || length > 20 || hex.length() != length * 2) {
        Serial.println("MAGFAIL data-len"); return false;
    }
    uint8_t data[20];
    for (uint16_t i = 0; i < length; ++i) {
        const int high = magazineHexNibble(hex[i * 2]);
        const int low = magazineHexNibble(hex[i * 2 + 1]);
        if (high < 0 || low < 0) { Serial.println("MAGFAIL data-hex"); return false; }
        data[i] = (uint8_t)((high << 4) | low);
    }
    return writeMagazineFlashData(offset, data, length);
}

static bool commitMagazineFlash() {
    if (!magazineFlash.active || magazineFlash.received != magazineFlash.size) {
        Serial.println("MAGFAIL commit-state"); endMagazineFlashSession(); return false;
    }
    const uint8_t verify[] = { CMD_MAGFW_VERIFY };
    if (!magazineWritePacket(*magazineFlash.port, magazineFlash.pcbId, verify, sizeof(verify))) {
        Serial.println("MAGFAIL verify-write"); endMagazineFlashSession(); return false;
    }
    uint8_t status = 0, error = 0; uint32_t received = 0, expected = 0;
    const unsigned long deadline = millis() + 30000UL;
    bool verified = false;
    do {
        delay(100);
        if (magazineReadFlashStatus(*magazineFlash.port, magazineFlash.pcbId,
                                    status, error, received, expected, 500)) {
            if (status == MAGFW_STATUS_COMPLETE && error == MAGFW_ERR_NONE) { verified = true; break; }
            if (status == MAGFW_STATUS_ERROR) break;
        }
    } while ((long)(millis() - deadline) < 0);
    if (!verified) { Serial.println("MAGFAIL verify-status"); endMagazineFlashSession(); return false; }
    const uint8_t commit[] = { CMD_MAGFW_COMMIT, 0x4D, 0x46, 0x57, 0x43 };
    if (!magazineWritePacket(*magazineFlash.port, magazineFlash.pcbId, commit, sizeof(commit))) {
        Serial.println("MAGFAIL commit-write"); endMagazineFlashSession(); return false;
    }
    Serial.println("MAGDONE"); endMagazineFlashSession(); return true;
}

bool runMagazineCalibrationBlocking(int magazine) {
    // Magazine PCB runs the full 2-run × 10-batch × 100-sample loop internally.
    // Gantry sends CMD_CALIBRATE, polls status, then reads 20 averaged values and prints.

    MagazineIrSensorModule& irModule = getMagazineIrModule(magazine);
    const uint8_t pcbID = irModule.getPcbID();
    I2CInstance& magazinePort = (magazine == 1) ? i2c1 : i2c2;

    // Pause background holder polling while calibration owns this transport.
    const bool savedStartMagzineChecks = startMagzinechecks;
    startMagzinechecks = false;
    Magazine1Ir.clearPendingRequests();
    Magazine2Ir.clearPendingRequests();
    magazinePort.clearPendingData();

    // ---- Reset Magazine PCB state so calibration always runs fresh ----
    requestMagazineCommandBlocking(magazinePort, pcbID, CMD_SOFT_RESET, 1);
    magazinePort.clearPendingData();

    // ---- Send CMD_CALIBRATE ----
    Serial.print("CAL"); Serial.print(magazine); Serial.println(": sending CMD_CALIBRATE");
    if (!requestMagazineCommandBlocking(magazinePort, pcbID, CMD_CALIBRATE, 1)) {
        Serial.println("CAL: CMD_CALIBRATE send failed");
        startMagzinechecks = savedStartMagzineChecks;
        return false;
    }

    // ---- Poll status until complete or timeout (30 s) ----
    const unsigned long CAL_TIMEOUT_MS = 30000UL;
    const unsigned long startMs = millis();
    uint8_t status = CAL_STATUS_RUNNING;

    while (status == CAL_STATUS_RUNNING && (millis() - startMs < CAL_TIMEOUT_MS)) {
        delay(500);
        magazinePort.clearPendingData();
        const uint8_t* resp = requestMagazineDataBlocking(magazinePort, pcbID, CMD_CALIBRATE, 1);
        if (resp) {
            status = resp[0];
        }
    }

    if (status != CAL_STATUS_COMPLETE) {
        Serial.print("CAL"); Serial.print(magazine);
        Serial.println(": FAILED or TIMEOUT");
        startMagzinechecks = savedStartMagzineChecks;
        return false;
    }

#if MAGAZINE_UART_TEST
    // UART has no Wire 32-byte ceiling. Recall the status, type, and all twenty
    // values in one atomic response so two independently timed reads cannot mix.
    magazinePort.clearPendingData();
    const uint8_t* combinedResult = requestMagazineDataBlocking(
        magazinePort, pcbID, CMD_CALIBRATE_RESULT_ALL, 42);
    if (!combinedResult || combinedResult[0] != CAL_STATUS_COMPLETE) {
        Serial.println("CAL: combined result read failed");
        startMagzinechecks = savedStartMagzineChecks;
        return false;
    }

    const uint8_t combinedMagType = combinedResult[1];
    const int combinedHolderIndex = magazine - 1;
    Serial.print("CAL: Magazine 00"); Serial.print(combinedMagType);
    Serial.print(", Holder "); Serial.print(magazine);
    Serial.println(" CALIBRATION RESULTS:");
    Serial.print("// Slide_Threshold[2][2][20]  index [");
    Serial.print(combinedMagType - 1); Serial.print("][");
    Serial.print(combinedHolderIndex); Serial.println("]");
    Serial.println("{");
    for (int s = 0; s < 20; ++s) {
        const uint16_t rawValue = ((uint16_t)combinedResult[2 + s * 2] << 8) |
                                  combinedResult[3 + s * 2];
        const long calValue = (long)rawValue - CAL_ERROR_MARGIN[combinedHolderIndex][s];
        Serial.print("    "); Serial.print(calValue);
        Serial.print(",  // S"); Serial.println(s + 1);
    }
    Serial.println("},");
    startMagzinechecks = savedStartMagzineChecks;
    return true;
#endif

    // ---- Read result part 1: [status][magType][S1..S10 × 2B] ----
    // responseLength=22 → master requests 23 bytes (22 + 1 checksum)
    magazinePort.clearPendingData();
    const uint8_t* r1 = requestMagazineDataBlocking(magazinePort, pcbID, CMD_CALIBRATE_RESULT_1, 22);
    if (!r1 || r1[0] != CAL_STATUS_COMPLETE) {
        Serial.println("CAL: result-1 read failed");
        startMagzinechecks = savedStartMagzineChecks;
        return false;
    }
    // Copy before the next requestSensorData overwrites dataBuffer
    uint8_t res1[22];
    memcpy(res1, r1, 22);

    // ---- Read result part 2: [S11..S20 × 2B] ----
    // responseLength=20 → master requests 21 bytes (20 + 1 checksum)
    magazinePort.clearPendingData();
    const uint8_t* r2 = requestMagazineDataBlocking(magazinePort, pcbID, CMD_CALIBRATE_RESULT_2, 20);
    if (!r2) {
        Serial.println("CAL: result-2 read failed");
        startMagzinechecks = savedStartMagzineChecks;
        return false;
    }
    uint8_t res2[20];
    memcpy(res2, r2, 20);

    startMagzinechecks = savedStartMagzineChecks;

    // ---- Decode values and apply CAL_ERROR_MARGIN, then print ----
    const uint8_t magType    = res1[1];
    const int     holderIndex = magazine - 1;

    long results[20];
    for (int s = 0; s < 10; s++) {
        results[s]      = ((uint16_t)res1[2 + s * 2] << 8) | res1[3 + s * 2];
    }
    for (int s = 0; s < 10; s++) {
        results[10 + s] = ((uint16_t)res2[s * 2] << 8) | res2[1 + s * 2];
    }

    Serial.print("CAL: Magazine 00"); Serial.print(magType);
    Serial.print(", Holder "); Serial.print(magazine);
    Serial.println(" CALIBRATION RESULTS:");
    Serial.print("// Slide_Threshold[2][2][20]  index [");
    Serial.print(magType - 1); Serial.print("]["); Serial.print(holderIndex); Serial.println("]");
    Serial.println("{");
    for (int s = 0; s < 20; s++) {
        long calVal = results[s] - CAL_ERROR_MARGIN[holderIndex][s];
        Serial.print("    "); Serial.print(calVal);
        Serial.print(",  // S"); Serial.println(s + 1);
    }
    Serial.println("},");

    return true;
}

static uint8_t scanWireBus(TwoWire& wire, const char* label) {
    Serial.print("I2CSCAN ");
    Serial.print(label);
    Serial.println(": scanning 0x01..0x7F");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 0x80; addr++) {
        wire.beginTransmission(addr);
        if (wire.endTransmission(true) == 0) {
            Serial.print("I2CSCAN ");
            Serial.print(label);
            Serial.print(": found 0x");
            if (addr < 0x10) Serial.print('0');
            Serial.println(addr, HEX);
            found++;
        }
        delay(2);
    }
    Serial.print("I2CSCAN ");
    Serial.print(label);
    Serial.print(": found count=");
    Serial.println(found);
    return found;
}

static void runI2cScanBlocking() {
    const bool savedStartMagzineChecks = startMagzinechecks;
    startMagzinechecks = false;
    Magazine1Ir.clearPendingRequests();
    Magazine2Ir.clearPendingRequests();
    i2c1.clearPendingData();
    i2c2.clearPendingData();

    uint8_t total = 0;
    #if MAGAZINE_UART_TEST
    total += scanWireBus(Wire3, "LIDAR PC9/PA8");
    #else
    total += scanWireBus(Wire3, "MAG/SLAVE PC9/PA8");
    #endif
    total += scanWireBus(Wire1, "ACC1 PB7/PB8");
    #if !MAGAZINE_UART_TEST
    total += scanWireBus(Wire2, "ACC2 PB11/PB10");
    #endif
    Serial.print("I2CSCAN total found=");
    Serial.println(total);
    startMagzinechecks = savedStartMagzineChecks;
}

bool runGhomeBlocking(bool printStatus = true) {
    if (printStatus) {
        Serial.println("GHOME: homing all axes");
    }

    // 1. GYHO
    delay(2000);
    if(!homeGyIr4Blocking()) {Serial.println("GHOME: GY FAILED");return false;}
    Serial.println("GHOME: GY done");

	    // 4. GZHO
    if (!homeGzWithHallBlocking("GHOME: GZ")) {
        Serial.println("GHOME: GZ failed");
        return false;
    }
    Serial.println("GHOME: GZ done");

    // 2. GRHO
    {
        if (!homeGrWithIr3Blocking("GHOME: GR", true)) {
            return false;
        }
    }

    // 3. GXHO
    if (!homeGxWithLidarBlocking("GHOME: GX")) {
        Serial.println("GHOME: GX failed");
        return false;
    }
    Serial.println("GHOME: GX done");


    if (printStatus) {
        Serial.println("GHOME: all axes homed");
    }

    return true;
}

bool executeLoadAndZeroAxesBlocking() {
    if (!executeMagazinePriorityLoadBlocking(1, 1)) {
        return false;
    }

    zeroAllGantryAxesCurrentPosition();
    Serial.println("RST: GX/GY/GZ/GR set to 0");
    return true;
}

// Boot helper — mirrors the LOAD command: detect both holders, arm auto-load if any magazine present.
// Returns true if a magazine was detected and auto-load is armed; false if neither holder has one.
bool runBootLoadActivationBlocking() {
    int mag1Type = 0, mag2Type = 0;
    for (int attempt = 0; attempt < 3 && mag1Type == 0 && mag2Type == 0; attempt++) {
        clearMagazineBusState();

        if (requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
            if (requestFullMagazineIrValuesBlocking(Magazine1Ir, i2c1)) {
                mag1Type = detectMagazineType(Magazine1Ir);
            }
            requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
        }
        clearMagazineBusState();
        if (requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
            if (requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2)) {
                mag2Type = detectMagazineType(Magazine2Ir);
            }
            requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
        }

        if (mag1Type == 0 && mag2Type == 0 && attempt < 2) {
            delay(50);
        }
    }

    if (mag1Type == 0 && mag2Type == 0) {
        Serial.println("LOAD Error: No magazine detected in any holder");
        return false;
    }

    autoLoadMagazine = (mag1Type != 0) ? 1 : 2;
    autoLoadState    = AutoLoadState::WaitIr2Clear;
    autoLoadStopRequested = false;
    autoLoadActive   = true;
    moveGantryToLoadPositionBlocking(false);
    return true;
}

bool executeRstBlocking() {
    int* irValues = readCurrentIrValues();
    if (irValues == nullptr) {
        Serial.println("RST Error: failed to read IR values");
        return false;
    }

    const int ir1State = isRstIr1Active(irValues[RST_IR1_INDEX]) ? 1 : 0;
    const int ir2State = isRstIr2Active(irValues[RST_IR2_INDEX]) ? 1 : 0;
    const int ir3State = isRstIr3Active(irValues[RST_IR3_INDEX]) ? 1 : 0;
    const int ir4State = isRstIr4Active(irValues[RST_IR4_INDEX]) ? 1 : 0;

    Serial.print("RST: IR1,IR2,IR3,IR4=");
    Serial.print(ir1State);
    Serial.print(",");
    Serial.print(ir2State);
    Serial.print(",");
    Serial.print(ir3State);
    Serial.print(",");
    Serial.println(ir4State);

    if (ir2State == 0 && ir3State == 1 && ir4State == 1) {
        return runGhomeBlocking(true);
    }

    if ((ir2State == 1 && ir3State == 1 && ir4State == 1) ||
        (ir2State == 1 && ir3State == 0 && ir4State == 1)) {
        return runGhomeBlocking(true) && executeLoadAndZeroAxesBlocking();
    }

    if (ir2State == 1 && ir3State == 0 && ir4State == 0) {
        if (!moveGyNegativeUntilIr1ActiveBlocking()) {
            return false;
        }
        return runGhomeBlocking(true);
    }

    if (ir2State == 0 && ir3State == 0 && ir4State == 0) {
        return runGhomeBlocking(true);
    }

    if (ir2State == 0 && ir3State == 0 && ir4State == 1) {
        return runGhomeBlocking(true);
    }

    Serial.println("RST Error: Unsupported IR state");
    return false;
}

MagazineIrSensorModule& getMagazineIrModule(int magazine) {
    return (magazine == 1) ? Magazine1Ir : Magazine2Ir;
}

long getMagazinePickGxPosition(int magazine) {
#if GANTRY_MAG_HALL_CAL_EXPERIMENT
    if (magazineGxHallCalibration.valid && magazineGxHallCalibration.applied) {
        return magazineGxHallCalibration.peakPosition[magazine == 1 ? 0 : 1];
    }
#endif
    return (magazine == 1) ? MAGAZINE1_PICK_GX_POSITION : MAGAZINE2_PICK_GX_POSITION;
}

#if GANTRY_MAG_HALL_CAL_EXPERIMENT
static int readMagazineHallAveraged() {
    long total = 0;
    constexpr uint8_t sampleCount = 4;
    for (uint8_t i = 0; i < sampleCount; ++i) {
        total += analogRead(GANTRY_MAG_HALL_PIN);
    }
    return (int)(total / sampleCount);
}

static void updateMagazineHallPeak(uint8_t holderIndex, long position, int reading) {
    if (reading < magazineGxHallCalibration.baselineAdc[holderIndex]) {
        magazineGxHallCalibration.baselineAdc[holderIndex] = reading;
    }
    if (reading > magazineGxHallCalibration.peakAdc[holderIndex]) {
        magazineGxHallCalibration.peakAdc[holderIndex] = reading;
        magazineGxHallCalibration.peakPosition[holderIndex] = position;
    }
}

static void printMagazineHallCalibrationStatus() {
    Serial.print("MAGCAL STATUS valid=");
    Serial.print(magazineGxHallCalibration.valid ? 1 : 0);
    Serial.print(" applied=");
    Serial.println(magazineGxHallCalibration.applied ? 1 : 0);

    for (uint8_t i = 0; i < 2; ++i) {
        Serial.print("MAGCAL H"); Serial.print(i + 1);
        Serial.print(" peakStep="); Serial.print(magazineGxHallCalibration.peakPosition[i]);
        Serial.print(" peakAdc="); Serial.print(magazineGxHallCalibration.peakAdc[i]);
        Serial.print(" baselineAdc="); Serial.print(magazineGxHallCalibration.baselineAdc[i]);
        Serial.print(" rise=");
        Serial.println(magazineGxHallCalibration.peakAdc[i] - magazineGxHallCalibration.baselineAdc[i]);
    }

    Serial.print("MAGCAL ACTIVE H1="); Serial.print(getMagazinePickGxPosition(1));
    Serial.print(" H2="); Serial.println(getMagazinePickGxPosition(2));
}

static bool runMagazineHallCalibrationBlocking() {
    if (magazineGxHallCalibrationInProgress) {
        Serial.println("MAGCAL ERROR: calibration already running");
        return false;
    }
    if (autoLoadActive) {
        Serial.println("MAGCAL ERROR: stop automatic LOAD before calibration");
        return false;
    }
    if (GXMotor.isMoving() || GYMotor.isMoving() || GZMotor.isMoving() || GRMotor.isMoving()) {
        Serial.println("MAGCAL ERROR: gantry axis is moving");
        return false;
    }

    magazineGxHallCalibrationInProgress = true;
    magazineGxHallCalibration.valid = false;
    magazineGxHallCalibration.applied = false;
    magazineGxHallCalibration.peakPosition[0] = 0L;
    magazineGxHallCalibration.peakPosition[1] = 0L;
    magazineGxHallCalibration.peakAdc[0] = 0;
    magazineGxHallCalibration.peakAdc[1] = 0;
    magazineGxHallCalibration.baselineAdc[0] = 4095;
    magazineGxHallCalibration.baselineAdc[1] = 4095;

    const bool savedMagazineChecks = startMagzinechecks;
    startMagzinechecks = false;
    clearMagazineBusState();

    Serial.println("MAGCAL: EXPERIMENT started; homing all gantry axes");
    if (!runGhomeBlocking(true)) {
        Serial.println("MAGCAL ERROR: GHOME failed; existing pickup positions retained");
        startMagzinechecks = savedMagazineChecks;
        magazineGxHallCalibrationInProgress = false;
        return false;
    }

    Serial.print("MAGCAL: moving GZ to scan position ");
    Serial.println(GANTRY_MAG_HALL_GZ_SCAN_POSITION);
    moveMotorToAndWait(GZMotor, GANTRY_MAG_HALL_GZ_SCAN_POSITION);

    const long savedSpeed = GXMotor.getMaxSpeed();
    const long savedAccel = GXMotor.getAcceleration();
    GXMotor.setMaxSpeed(GANTRY_MAG_HALL_SCAN_SPEED);
    GXMotor.setAcceleration(GANTRY_MAG_HALL_SCAN_ACCEL);

    Serial.print("MAGCAL: scanning IR5/PA7 from 0 to ");
    Serial.println(GANTRY_MAG_HALL_SCAN_END_STEPS);
    GXMotor.moveTo(GANTRY_MAG_HALL_SCAN_END_STEPS);
    const unsigned long scanStartedMs = millis();
    long lastSamplePosition = 0L;
    long nextProgressPosition = -10000L;
    bool firstSample = true;
    bool scanTimedOut = false;

    while (GXMotor.isMoving()) {
        GXMotor.loop();
        limitSwitch.loop();
        const long position = GXMotor.getCurrentPosition();
        if (firstSample || position <= lastSamplePosition - GANTRY_MAG_HALL_SAMPLE_STEPS) {
            firstSample = false;
            lastSamplePosition = position;
            const int reading = readMagazineHallAveraged();
            if (position <= GANTRY_MAG_HALL_H1_WINDOW_MAX &&
                position >= GANTRY_MAG_HALL_H1_WINDOW_MIN) {
                updateMagazineHallPeak(0, position, reading);
            }
            if (position <= GANTRY_MAG_HALL_H2_WINDOW_MAX &&
                position >= GANTRY_MAG_HALL_H2_WINDOW_MIN) {
                updateMagazineHallPeak(1, position, reading);
            }
        }
        if (position <= nextProgressPosition) {
            Serial.print("MAGCAL: GX="); Serial.print(position);
            Serial.print(" Hall="); Serial.println(readMagazineHallAveraged());
            nextProgressPosition -= 10000L;
        }
        if (millis() - scanStartedMs > GANTRY_MAG_HALL_LEG_TIMEOUT_MS) {
            GXMotor.stop();
            while (GXMotor.isMoving()) GXMotor.loop();
            scanTimedOut = true;
            break;
        }
    }

    // Always return GX to the coordinate origin after the experimental scan.
    Serial.println("MAGCAL: returning GX to home position 0");
    GXMotor.moveTo(0L);
    const unsigned long returnStartedMs = millis();
    bool returnTimedOut = false;
    while (GXMotor.isMoving()) {
        GXMotor.loop();
        limitSwitch.loop();
        if (millis() - returnStartedMs > GANTRY_MAG_HALL_LEG_TIMEOUT_MS) {
            GXMotor.stop();
            while (GXMotor.isMoving()) GXMotor.loop();
            returnTimedOut = true;
            break;
        }
    }
    GXMotor.setMaxSpeed(savedSpeed);
    GXMotor.setAcceleration(savedAccel);
    Serial.println("MAGCAL: returning GZ to home position 0");
    moveMotorToAndWait(GZMotor, 0L);
    startMagzinechecks = savedMagazineChecks;
    magazineGxHallCalibrationInProgress = false;

    if (scanTimedOut || returnTimedOut) {
        Serial.println("MAGCAL ERROR: motion timeout; existing pickup positions retained");
        printMagazineHallCalibrationStatus();
        return false;
    }

    const int h1Rise = magazineGxHallCalibration.peakAdc[0] - magazineGxHallCalibration.baselineAdc[0];
    const int h2Rise = magazineGxHallCalibration.peakAdc[1] - magazineGxHallCalibration.baselineAdc[1];
    if (h1Rise < GANTRY_MAG_HALL_MIN_RISE || h2Rise < GANTRY_MAG_HALL_MIN_RISE) {
        Serial.print("MAGCAL ERROR: Hall rise too small; required >= ");
        Serial.println(GANTRY_MAG_HALL_MIN_RISE);
        Serial.println("MAGCAL: check magnet polarity, spacing, wiring, and IR5 signal");
        printMagazineHallCalibrationStatus();
        return false;
    }

    magazineGxHallCalibration.peakPosition[0] += GANTRY_MAG_HALL_H1_PICK_OFFSET_STEPS;
    magazineGxHallCalibration.peakPosition[1] += GANTRY_MAG_HALL_H2_PICK_OFFSET_STEPS;
    magazineGxHallCalibration.valid = true;
    Serial.println("MAGCAL: scan valid; positions are report-only until MAGCAL APPLY");
    printMagazineHallCalibrationStatus();
    return true;
}

static void handleMagazineHallCalibrationCommand(const String& line) {
    const int prefixLength = line.startsWith("MAGXCAL") ? 7 : 6;
    String arg = line.substring(prefixLength);
    arg.trim();
    arg.toUpperCase();

    if (arg.length() == 0 || arg == "RUN") {
        (void)runMagazineHallCalibrationBlocking();
    } else if (arg == "STATUS") {
        printMagazineHallCalibrationStatus();
    } else if (arg == "APPLY") {
        if (!magazineGxHallCalibration.valid) {
            Serial.println("MAGCAL ERROR: run a valid calibration before APPLY");
        } else {
            magazineGxHallCalibration.applied = true;
            Serial.println("MAGCAL: calibrated GX pickup positions enabled for this boot");
            printMagazineHallCalibrationStatus();
        }
    } else if (arg == "CLEAR") {
        magazineGxHallCalibration.applied = false;
        Serial.println("MAGCAL: hard-coded GX pickup positions restored");
        printMagazineHallCalibrationStatus();
    } else {
        Serial.println("MAGCAL ERROR: use RUN, STATUS, APPLY, or CLEAR");
    }
}
#endif

bool isMagazineLockedForLoad(const MagazineIrSensorModule& irModule) {
    return irModule.getIrValues()[MAGAZINE_LOCK_STATUS_INDEX] > MAGAZINE_LOCKED_THRESHOLD;
}

bool ensureMagazineLockedForLoadBlocking(int magazine) {
    if (magazine != 1 && magazine != 2) {
        return false;
    }

    MagazineIrSensorModule& irModule = getMagazineIrModule(magazine);
    I2CInstance& magazinePort = getMagazinePort(magazine);

    clearMagazineBusState();
    if (!requestHalfMagazineIr2Blocking(irModule, magazinePort)) {
        Serial.println("LOAD LOCK TIMEOUT");
        return false;
    }

    if (isMagazineLockedForLoad(irModule)) {
        return true;
    }

    clearMagazineBusState();
    if (!requestMagazineLockCommandBlocking(magazinePort, irModule.getPcbID())) {
        Serial.print("M");
        Serial.print(magazine);
        Serial.println("L TIMEOUT");
        return false;
    }

    delay(MAGAZINE_LOCK_SETTLE_MS);

    clearMagazineBusState();
    if (!requestHalfMagazineIr2Blocking(irModule, magazinePort)) {
        Serial.println("LOAD LOCK TIMEOUT");
        return false;
    }

    if (!isMagazineLockedForLoad(irModule)) {
        Serial.print("M");
        Serial.print(magazine);
        Serial.println("L FAILED");
        return false;
    }

    return true;
}

bool moveGantryToLoadPositionBlocking(bool printStatus = true) {
    if (printStatus) {
        Serial.println("GLOAD: moving to load position");
    }

    GXMotor.moveTo(GLOAD_GX_POSITION);
    //GYMotor.moveTo(GLOAD_GY_POSITION);
    GZMotor.moveTo(GLOAD_GZ_POSITION);
    GRMotor.moveTo(GLOAD_GR_POSITION);

    while (GXMotor.isMoving() || GYMotor.isMoving() || GZMotor.isMoving() || GRMotor.isMoving()) {
        serviceGantryMotionWhileWaiting();
    }

    if (printStatus) {
        Serial.println("GLOAD: done");
    }

    return true;
}

// Returns the raw S21/S22/S23 3-bit code — 0..7. 0 = no magazine.
//   1 = MAG001, 2 = MAG010, 3 = MAG011, 4 = MAG100, 5 = MAG101, 6 = MAG110, 7 = MAG111
int detectMagazineType(const MagazineIrSensorModule& irModule) {
    const long* h = irModule.getIrValues();
    int s21 = (h[20] < MAG_TYPE_S21_THRESHOLD) ? 1 : 0;
    int s22 = (h[21] < MAG_TYPE_S22_THRESHOLD) ? 1 : 0;
    int s23 = (h[22] < MAG_TYPE_S23_THRESHOLD) ? 1 : 0;
    return (s23 << 2) | (s22 << 1) | s21;
}

} // close anonymous namespace

// Runtime calibration storage — populated exclusively by SETCAL command from ESP32.
// 7 magazine types × 2 holders × 20 slots. Source of truth lives on the ESP32 (NVS).
static constexpr int MAX_MAG_TYPES = 7;  // Holder-local table types 1..7.
static constexpr unsigned long CAL_FETCH_TIMEOUT_MS = 3000UL;
static int  slideThresholdPending[MAX_MAG_TYPES][2][20] = {{{0}}};
static uint8_t slideThresholdPendingMask[MAX_MAG_TYPES][2] = {{0}};

// Returns the Gantry LOAD threshold array, or nullptr if ESP32 has no calibration
// for this type/holder. The same values are forwarded to the selected holder LEDs.
namespace {

static bool pushHolderLedCalibration(int holder, int magType, const int values[20]) {
    if (holder < 1 || holder > 2 || magType < 1 || magType > MAX_MAG_TYPES) return false;
    // SETCAL may arrive asynchronously while background status polling is in
    // flight. Cancel those snapshots before starting an acknowledged write.
    clearMagazineBusState();
    MagazineIrSensorModule& irModule = getMagazineIrModule(holder);
#if MAGAZINE_UART_TEST
    uint8_t payload[41];
    payload[0] = (uint8_t)magType;
    for (uint8_t i = 0; i < 20; ++i) {
        const int bounded = constrain(values[i], 0, 65535);
        payload[1 + i * 2] = (uint8_t)((uint16_t)bounded >> 8);
        payload[2 + i * 2] = (uint8_t)bounded;
    }
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (getMagazinePort(holder).writeCommandData(
                irModule.getPcbID(), CMD_SET_LED_CAL_ALL, payload, sizeof(payload))) {
            return true;
        }
        delay(2);
    }
    return false;
#else
    uint8_t payload[21];
    payload[0] = (uint8_t)magType;

    for (uint8_t part = 0; part < 2; ++part) {
        for (uint8_t i = 0; i < 10; ++i) {
            const int bounded = constrain(values[part * 10 + i], 0, 65535);
            payload[1 + i * 2] = (uint8_t)((uint16_t)bounded >> 8);
            payload[2 + i * 2] = (uint8_t)bounded;
        }
        const uint8_t command = (part == 0) ? CMD_SET_LED_CAL_1 : CMD_SET_LED_CAL_2;
        bool sent = false;
        for (uint8_t attempt = 0; attempt < 3 && !sent; ++attempt) {
            sent = getMagazinePort(holder).writeCommandData(irModule.getPcbID(), command, payload, sizeof(payload));
            if (!sent) delay(2);
        }
        if (!sent) return false;
        delay(10); // let the holder consume part 1 before part 2 arrives
    }
    return true;
#endif
}

// Parse "SETCAL<H><T> <s1> <s2> ... <s20>" — overrides compile-time thresholds at runtime.
// Example: "SETCAL11 3178 2894 3929 ... 3199"
static bool parseSetCalHeader(const String& line, int prefixLen, int& holder, int& magType) {
    if (line.length() < prefixLen + 2) {
        Serial.println("SETCAL: too short");
        return false;
    }
    holder = line.charAt(prefixLen) - '0';
    magType = line.charAt(prefixLen + 1) - '0';
    if (holder < 1 || holder > 2 || magType < 1 || magType > MAX_MAG_TYPES) {
        Serial.println("SETCAL: bad H/T (H=1..2, T=1..7)");
        return false;
    }
    return true;
}

static int parseSpaceValues(const String& text, int outValues[], int maxCount) {
    int count = 0;
    int start = 0;
    const int len = text.length();
    while (count < maxCount && start < len) {
        while (start < len && text.charAt(start) == ' ') start++;
        if (start >= len) break;
        int end = text.indexOf(' ', start);
        if (end < 0) end = len;
        outValues[count++] = text.substring(start, end).toInt();
        start = end + 1;
    }
    return count;
}

static void applySetCalValues(int holder, int magType, const int values[20]) {
    slideThresholdPendingMask[magType - 1][holder - 1] = 0;
    Serial.print("SETCAL: applied h"); Serial.print(holder);
    Serial.print(" t");                Serial.println(magType);

    if (!pushHolderLedCalibration(holder, magType, values)) {
        Serial.print("SETCAL: holder LED threshold push failed h"); Serial.print(holder);
        Serial.print(" t"); Serial.println(magType);
    }
}

static void handleSetCalPacketCommand(const String& line) {
    int holder = 0;
    int magType = 0;
    if (!parseSetCalHeader(line, 7, holder, magType)) return;

    String payload = line.substring(9);
    payload.trim();
    int parsed[6];
    const int count = parseSpaceValues(payload, parsed, 6);
    if (count < 2) {
        Serial.print("SETCALP: expected offset plus values, got ");
        Serial.println(count);
        return;
    }

    const int offset = parsed[0];
    if (offset < 0 || offset >= 20 || (offset % 5) != 0) {
        Serial.print("SETCALP: bad offset ");
        Serial.println(offset);
        return;
    }
    const int expected = min(5, 20 - offset);
    if (count - 1 != expected) {
        Serial.print("SETCALP: expected ");
        Serial.print(expected);
        Serial.print(" values at offset ");
        Serial.print(offset);
        Serial.print(", got ");
        Serial.println(count - 1);
        return;
    }

    if (offset == 0) {
        slideThresholdPendingMask[magType - 1][holder - 1] = 0;
    }
    for (int i = 0; i < expected; i++) {
        slideThresholdPending[magType - 1][holder - 1][offset + i] = parsed[i + 1];
    }
    slideThresholdPendingMask[magType - 1][holder - 1] |= (uint8_t)(1U << (offset / 5));
    Serial.print("SETCALP: part h"); Serial.print(holder);
    Serial.print(" t"); Serial.print(magType);
    Serial.print(" o"); Serial.println(offset);

    if (slideThresholdPendingMask[magType - 1][holder - 1] == 0x0F) {
        applySetCalValues(holder, magType, slideThresholdPending[magType - 1][holder - 1]);
    }
}

static void handleSetCalCommand(const String& line) {
    int holder = 0;
    int magType = 0;
    if (!parseSetCalHeader(line, 6, holder, magType)) return;

    String values = line.substring(8);
    values.trim();
    int v[20];
    const int count = parseSpaceValues(values, v, 20);
    if (count != 20) {
        Serial.print("SETCAL: expected 20 values, got ");
        Serial.println(count);
        return;
    }
    applySetCalValues(holder, magType, v);
}

// Custom MasterSerial reader: intercepts SETCAL lines, forwards everything else
// into the standard livoCommunication parser. Replaces serialEventStream(MasterSerial).
static void serviceMasterUartIntercept() {
    static bool bufDiscarding = false;
    if (livoCommunication.getinputBytesAvailable()) return;
    static String buf = "";
    unsigned byteBudget = 64;
    while (byteBudget-- && MasterSerial.available()) {
        char c = (char)MasterSerial.read();
        if (receiveUartLineByte(c, buf, bufDiscarding)) {
            buf.trim();
            if (buf.length() > 0) {
                if (serviceProductionBusy() && serviceBackgroundCommand(buf)) {
                    // Discovery is retried when idle, never ahead of LOAD/SETCAL.
                } else if (ServiceDiagnostics::locked() && buf != "ID" &&
                           !buf.startsWith("DLOCK ") && !buf.startsWith("DTEST ") &&
                           buf != "DSTOP" && buf != "LOADSTOP") {
                    // Service controls use the FIFO too; never overtake queued LOAD.
                } else if (buf == "LOADSTOP") {
                    requestAutoLoadStop();
                } else if (buf.startsWith("SETCALP")) {
                    handleSetCalPacketCommand(buf);
                } else if (buf.startsWith("SETCAL")) {
                    handleSetCalCommand(buf);
                } else if (buf == "I" || buf == "I2C" || buf == "I2CSCAN" || buf == "ISCAN") {
                    runI2cScanBlocking();
#if GANTRY_MAG_HALL_CAL_EXPERIMENT
                } else if (buf.startsWith("MAGCAL") || buf.startsWith("MAGXCAL")) {
                    handleMagazineHallCalibrationCommand(buf);
#endif
                } else if (buf.startsWith("MAGID")) {
                    String arg = buf.substring(5);
                    arg.trim();
                    reportMagazineFirmwareVersion((uint8_t)arg.toInt());
                } else if (buf.startsWith("MAGFWSTAT")) {
                    String arg = buf.substring(9);
                    arg.trim();
                    reportMagazineFlashStatus((uint8_t)arg.toInt());
                } else if (buf.startsWith("MAGFLASH")) {
                    String arg = buf.substring(8); arg.trim();
                    const int first = arg.indexOf(' ');
                    const int second = first < 0 ? -1 : arg.indexOf(' ', first + 1);
                    if (first < 0 || second < 0) {
                        Serial.println("MAGFAIL bad-args");
                    } else {
                        const uint8_t holder = (uint8_t)arg.substring(0, first).toInt();
                        const uint32_t size = (uint32_t)arg.substring(first + 1, second).toInt();
                        const uint32_t crc = (uint32_t)strtoul(arg.substring(second + 1).c_str(), nullptr, 0);
                        beginMagazineFlash(holder, size, crc);
                    }
                } else if (buf.startsWith("MAGDATA")) {
                    String arg = buf.substring(7); arg.trim();
                    writeMagazineFlashDataLine(arg);
                } else if (buf == "MAGCOMMIT") {
                    commitMagazineFlash();
                } else {
                    for (size_t i = 0; i < buf.length(); i++) {
                        livoCommunication.process2019Byte((byte)buf.charAt(i));
                    }
                    livoCommunication.process2019Byte((byte)'\n');
                }
            }
            buf = "";
            if (livoCommunication.getinputBytesAvailable()) return;
        }
    }
}

static bool refreshHolderSlideStatusBlocking(MagazineIrSensorModule& irModule,
                                             I2CInstance& magazinePort,
                                             unsigned long timeoutMs = 1500UL) {
    clearMagazineBusState();
    const uint32_t previousSequence = irModule.getSlideStatusSequence();
    irModule.setSilent(true);
    irModule.requestSlideStatus();
    const unsigned long startedMs = millis();
    while (irModule.getSlideStatusSequence() == previousSequence &&
           millis() - startedMs < timeoutMs) {
        magazinePort.loop();
        irModule.loop();
        delay(1);
    }
    bool fresh=irModule.getSlideStatusSequence()!=previousSequence;
    if(!fresh) SensorFault::report("LIVO-MAG-010",irModule.getPcbID()==Stainer_Magzine1_PCB_ID?"HOLDER1":"HOLDER2",-1,"slot_status_timeout");
    return fresh;
}

int resolvePriorityEmptyMagazineSlot(const MagazineIrSensorModule& irModule, int requestedSlideNumber) {
    if (requestedSlideNumber < 1 || requestedSlideNumber > 20 ||
        !irModule.isSlideStatusValid()) {
        return 0;
    }

    // Slot state is classified by the holder: 0=empty, 1=full.
    const int* slots = irModule.getSlideStatus();
    int matchedSlides = 0;

    for (int slotIndex = 0; slotIndex < 20; ++slotIndex) {
        if (slots[slotIndex] == 0) {
            matchedSlides++;
            if (matchedSlides == requestedSlideNumber) {
                return slotIndex + 1; // return 1-based slot number for GZ positioning
            }
        }
    }

    return 0;
}

int countDetectedMagazineSlides(const MagazineIrSensorModule& irModule) {
    if (!irModule.isSlideStatusValid()) return 0;
    const int* slots = irModule.getSlideStatus();
    int detectedSlides = 0;

    for (int slotIndex = 0; slotIndex < 20; ++slotIndex) {
        if (slots[slotIndex] == 1) detectedSlides++;
    }

    return detectedSlides;
}

static bool magazineCanAcceptSlideBlocking(int magazine, bool& present, bool& known) {
    present = false;
    known = false;
    if (magazine != 1 && magazine != 2) return false;

    MagazineIrSensorModule& irModule = getMagazineIrModule(magazine);
    I2CInstance& magazinePort = getMagazinePort(magazine);
    if (!refreshHolderSlideStatusBlocking(irModule, magazinePort)) return false;

    const int magType = irModule.getSlideMagazineType();
    if (irModule.getSlideStatusState() == 0 || magType == 0) {
        known = true;
        return false;
    }
    present = true;

    if (!irModule.isSlideStatusValid()) {
        requestCalFromEsp(magazine, magType);
        const unsigned long calRequestStartMs = millis();
        while (!irModule.isSlideStatusValid() &&
               millis() - calRequestStartMs < CAL_FETCH_TIMEOUT_MS) {
            serviceMasterUartIntercept();
            refreshHolderSlideStatusBlocking(irModule, magazinePort, 500UL);
        }
    }
    if (!irModule.isSlideStatusValid()) return false;

    known = true;
    return countDetectedMagazineSlides(irModule) < 20 &&
           resolvePriorityEmptyMagazineSlot(irModule, 1) != 0;
}

static bool anyMagazineCanAcceptSlideBlocking() {
    bool anyPresent = false;
    bool allPresentCapacityKnown = true;
    for (int magazine = 1; magazine <= 2; magazine++) {
        bool present = false;
        bool known = false;
        if (magazineCanAcceptSlideBlocking(magazine, present, known)) {
            return true;
        }
        if (present) {
            anyPresent = true;
            if (!known) allPresentCapacityKnown = false;
        }
    }
    return !anyPresent || !allPresentCapacityKnown;
}

void restoreGantryMotorDefaultMotionForLoad() {
    GXMotor.setMaxSpeed(TMCMotorGXConfig.speed);
    GXMotor.setAcceleration(TMCMotorGXConfig.acceleration);
    GYMotor.setMaxSpeed(TMCMotorGYConfig.speed);
    GYMotor.setAcceleration(TMCMotorGYConfig.acceleration);
    GRMotor.setMaxSpeed(TMCMotorGRConfig.speed);
    GRMotor.setAcceleration(TMCMotorGRConfig.acceleration);
    GZMotor.setMaxSpeed(TMCMotorGZConfig.speed);
    GZMotor.setAcceleration(TMCMotorGZConfig.acceleration);
    XMotor.setMaxSpeed(TMCMotorG2YConfig.speed);
    XMotor.setAcceleration(TMCMotorG2YConfig.acceleration);
}

bool executeMagazineLoadAtSlotBlocking(int magazine, int slotPosition, const char* doneMessage, bool sequentialReturnToLoad = false) {
    if ((magazine != 1 && magazine != 2) || slotPosition < 1 || slotPosition > 20) {
        return false;
    }

    if (!ensureMagazineLockedForLoadBlocking(magazine)) {
        return false;
    }

    restoreGantryMotorDefaultMotionForLoad();

    const long savedGXSpeed = GXMotor.getMaxSpeed();
    const long savedGXAccel = GXMotor.getAcceleration();
    const long savedGYSpeed = GYMotor.getMaxSpeed();
    const long savedGYAccel = GYMotor.getAcceleration();
    const long savedGZSpeed = GZMotor.getMaxSpeed();
    const long savedGZAccel = GZMotor.getAcceleration();
    GXMotor.setMaxSpeed(LOAD_GX_SPEED);
    GXMotor.setAcceleration(LOAD_GX_ACCELERATION);
    GYMotor.setMaxSpeed(LOAD_GY_SPEED);
    GYMotor.setAcceleration(LOAD_GY_ACCELERATION);
    GZMotor.setMaxSpeed(LOAD_GZ_SPEED);
    GZMotor.setAcceleration(LOAD_GZ_ACCELERATION);

    moveGantryToLoadPositionBlocking(false);
    moveMotorToAndWait(GZMotor, LOAD_ENTRY_GZ_POSITION);
    moveMotorToAndWait(GXMotor, getMagazinePickGxPosition(magazine));

    const long gzPos = LOAD_SLOT_GZ_BASE_POSITION - ((long)(slotPosition - 1) * LOAD_SLOT_GZ_PITCH);
    moveMotorToAndWait(GZMotor, gzPos);

    moveMotorRelativeAndWait(GRMotor, -LOAD_GR_STROKE);

    // ---- Sensor-driven GY strokes ----
    // +ve direction until IR1 drops below LOAD_GY_IR1_THRESHOLD (slide pushed home)
    // -ve direction until IR4 drops below LOAD_GY_IR4_THRESHOLD (carriage at home)
    // Single long move = one accel/cruise/decel ramp → smooth and fast.
    // On IR trip, acceleration is bumped to LOAD_GY_STOP_ACCEL for near-instant decel.
    // LOAD_GY_STROKE acts as a safety cap.
    auto moveGyUntilIrDrops = [](int irIdx, int threshold, bool negativeDir, long maxSteps) -> bool {
        const unsigned long startMs  = millis();
        const long          target   = negativeDir ? -maxSteps : maxSteps;
        const int           armLevel = threshold + LOAD_GY_ARM_MARGIN;

        int* irInit = limitSwitch.getSmoothedSensorValues();
        int  irAtStart = (irInit != nullptr) ? irInit[irIdx] : -1;

        // Edge-detect arming: if the sensor is already below threshold at start,
        // wait for it to rise above (threshold + margin) before accepting a trip.
        bool armed = (irAtStart >= armLevel);

        GYMotor.move(target);   // single uninterrupted move at LOAD_GY_SPEED / LOAD_GY_ACCELERATION

        while (GYMotor.isMoving()) {
            GYMotor.loop();
            limitSwitch.loop();

            int* irVals = limitSwitch.getSmoothedSensorValues();
            int  ir     = (irVals != nullptr) ? irVals[irIdx] : -1;

            if (!armed && ir >= armLevel) armed = true;

            if (armed && SensorFaultPolicy::adcValid(ir) && ir < threshold) {
                GYMotor.setAcceleration(LOAD_GY_STOP_ACCEL);
                GYMotor.stop();
                while (GYMotor.isMoving()) { GYMotor.loop(); }
                GYMotor.setAcceleration(LOAD_GY_ACCELERATION);
                return true;
            }
            if (millis() - startMs > LOAD_GY_TIMEOUT_MS) {
                GYMotor.setAcceleration(LOAD_GY_STOP_ACCEL);
                GYMotor.stop();
                while (GYMotor.isMoving()) { GYMotor.loop(); }
                GYMotor.setAcceleration(LOAD_GY_ACCELERATION);
                char channel[12];snprintf(channel,sizeof(channel),"GY-IR%d",irIdx+1);
                SensorFault::report("LIVO-SEN-023",channel,ir,"expected_transition_timeout_cause_unconfirmed");
                Serial.print("LOAD: GY-IR"); Serial.print(irIdx + 1); Serial.println(" timeout");
                return false;
            }
        }
        char channel[12];snprintf(channel,sizeof(channel),"GY-IR%d",irIdx+1);
        SensorFault::report("LIVO-SEN-023",channel,-1,"travel_cap_without_transition");
        Serial.print("LOAD: GY-IR"); Serial.print(irIdx + 1); Serial.println(" safety cap");
        return false;
    };

    // Old fixed-stroke Y motion (kept for reference / quick rollback):
    // moveMotorRelativeAndWait(GYMotor,  LOAD_GY_STROKE);
    // moveMotorRelativeAndWait(GYMotor, -LOAD_GY_STROKE);
    if (!moveGyUntilIrDrops(RST_IR1_INDEX, LOAD_GY_IR1_THRESHOLD, false, LOAD_GY_STROKE) ||
        !moveGyUntilIrDrops(RST_IR4_INDEX, LOAD_GY_IR4_THRESHOLD, true, LOAD_GY_STROKE)) {
        GXMotor.setMaxSpeed(savedGXSpeed); GXMotor.setAcceleration(savedGXAccel);
        GYMotor.setMaxSpeed(savedGYSpeed); GYMotor.setAcceleration(savedGYAccel);
        GZMotor.setMaxSpeed(savedGZSpeed); GZMotor.setAcceleration(savedGZAccel);
        Serial.println("LOAD FAILED: sensor transition not confirmed; motion cause unknown");
        return false;
    }

    // Ask the holder for its freshly averaged and classified slot result.
    {
        MagazineIrSensorModule& irModule = getMagazineIrModule(magazine);
        I2CInstance& magazinePort = getMagazinePort(magazine);
        if (refreshHolderSlideStatusBlocking(irModule, magazinePort) &&
            irModule.isSlideStatusValid()) {
            int slides = countDetectedMagazineSlides(irModule);
            Serial.print("SLIDES: "); Serial.print(magazine); Serial.print(' '); Serial.println(slides);
        }
    }

    moveMotorRelativeAndWait(GRMotor, LOAD_GR_STROKE);
    if (sequentialReturnToLoad) {
        moveMotorToAndWait(GZMotor, 0);
        moveMotorToAndWait(GXMotor, GLOAD_GX_POSITION);
		moveMotorToAndWait(GZMotor, GLOAD_GZ_POSITION);
    } else {
        moveGantryToLoadPositionBlocking(false);
    }

    GXMotor.setMaxSpeed(savedGXSpeed);
    GXMotor.setAcceleration(savedGXAccel);
    GYMotor.setMaxSpeed(savedGYSpeed);
    GYMotor.setAcceleration(savedGYAccel);
    GZMotor.setMaxSpeed(savedGZSpeed);
    GZMotor.setAcceleration(savedGZAccel);
    activeMagazineForLoad = magazine;
    Serial.println(doneMessage);
    return true;
}

// Unlock the just-filled magazine and, if the other holder has a magazine, switch auto-load to it.
static void unlockMagazineAndAutoSwitch(int magazine, MagazineIrSensorModule& irModule) {
    clearMagazineBusState();
    if (!requestMagazineUnlockCommandBlocking(getMagazinePort(magazine), irModule.getPcbID())) {
        Serial.print("M"); Serial.print(magazine); Serial.println("U TIMEOUT");
    }

    Serial.print("MAGZINE");
    Serial.print(magazine);
    Serial.println(" FULL, CHANGE MAGZINE");

    if (activeMagazineForLoad == magazine) {
        activeMagazineForLoad = 0;
    }

    // Detect if the other magazine holder has a magazine inserted; if so, switch auto-load to it.
    const int otherMag = (magazine == 1) ? 2 : 1;
    MagazineIrSensorModule& otherIr = getMagazineIrModule(otherMag);
    int otherMagType = 0;
    clearMagazineBusState();
    I2CInstance& otherPort = getMagazinePort(otherMag);
    if (requestMagazineTriggerCommandBlocking(otherPort, otherIr.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
        if (requestFullMagazineIrValuesBlocking(otherIr, otherPort)) {
            otherMagType = detectMagazineType(otherIr);
        }
        requestMagazineTriggerCommandBlocking(otherPort, otherIr.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
    }
    if (otherMagType != 0) {
        Serial.print("Switching to MAG"); Serial.println(otherMag);
        autoLoadMagazine = otherMag;
    }
}

// Forward decl — defined later, near the magazine insertion automation block
static void requestCalFromEsp(int holder, int magType);

bool executeMagazinePriorityLoadBlocking(int magazine, int requestedSlideNumber) {
    if ((magazine != 1 && magazine != 2) || requestedSlideNumber < 1 || requestedSlideNumber > 20) {
        Serial.println("LOAD Error: Invalid requested slide number");
        return false;
    }

    MagazineIrSensorModule& irModule = getMagazineIrModule(magazine);
    I2CInstance& magazinePort = getMagazinePort(magazine);
    if (!refreshHolderSlideStatusBlocking(irModule, magazinePort)) {
        Serial.println("LOAD TIMEOUT");
        return false;
    }

    const int magType = irModule.getSlideMagazineType();

    if (irModule.getSlideStatusState() == 0 || magType == 0) {
        Serial.println("LOAD Error: No magazine detected in holder");
        return false;
    }
    if (!irModule.isSlideStatusValid()) {
        requestCalFromEsp(magazine, magType);
        const unsigned long calRequestStartMs = millis();
        while (!irModule.isSlideStatusValid() &&
               millis() - calRequestStartMs < CAL_FETCH_TIMEOUT_MS) {
            serviceMasterUartIntercept();
            refreshHolderSlideStatusBlocking(irModule, magazinePort, 500UL);
        }

        if (!irModule.isSlideStatusValid()) {
            Serial.print("LOAD Error: CALIBRATION NOT AVAILABLE for MAG type ");
            Serial.print(magType);
            Serial.print(" Holder ");
            Serial.println(magazine);
            return false;
        }

        Serial.print("LOAD: calibration received for MAG type ");
        Serial.print(magType);
        Serial.print(" Holder ");
        Serial.println(magazine);
    }

    if (countDetectedMagazineSlides(irModule) >= 20) {
        unlockMagazineAndAutoSwitch(magazine, irModule);
        return false;
    }

    const int slotPosition = resolvePriorityEmptyMagazineSlot(irModule, requestedSlideNumber);
    if (slotPosition == 0) {
        Serial.print("LOAD Error: Empty slot ");
        Serial.print(requestedSlideNumber);
        Serial.println(" not found");
        return false;
    }
    if (requestedSlideNumber == 1) {
        Serial.print("LOAD: Mag ");
        Serial.print(magazine);
        Serial.print(" -> Slot ");
        Serial.println(slotPosition);
    } else {
        Serial.print("LOAD: Mag ");
        Serial.print(magazine);
        Serial.print(" Slide ");
        Serial.print(requestedSlideNumber);
        Serial.print(" -> Slot ");
        Serial.println(slotPosition);
    }

    const bool loadOk = executeMagazineLoadAtSlotBlocking(magazine, slotPosition, "LOAD Done", true);

    // If this load filled the last empty slot, unlock immediately and switch to other holder.
    // The holder status was refreshed inside executeMagazineLoadAtSlotBlocking.
    if (loadOk && countDetectedMagazineSlides(irModule) >= 20) {
        unlockMagazineAndAutoSwitch(magazine, irModule);
    }

    return loadOk;
}

bool executeMagazineLoadBlocking(int magazine, int slotPosition) {
    if ((magazine != 1 && magazine != 2) || slotPosition < 1 || slotPosition > 20) {
        Serial.println("LDM Error: Invalid Mag/Slide format. Use LDM<Mag><Slide> e.g. LDM11 or LDM114");
        return false;
    }

    Serial.print("LDM: Mag ");
    Serial.print(magazine);
    Serial.print(" Slide ");
    Serial.println(slotPosition);

    return executeMagazineLoadAtSlotBlocking(magazine, slotPosition, "LDM Done", false);
}

int resolveLoadMagazine(long loadValue) {
    if (loadValue == 1 || loadValue == 2) {
        return (int)loadValue;
    }

    if (activeMagazineForLoad == 1 || activeMagazineForLoad == 2) {
        return activeMagazineForLoad;
    }

    const bool mag1Present = Magazine1Ir.getSlideStatusState() != 0;
    const bool mag2Present = Magazine2Ir.getSlideStatusState() != 0;

    if (mag1Present) {
        return 1;
    }
    if (mag2Present) {
        return 2;
    }

    return 0;
}

void queueSilentMagazineSlideRequest(MagazineIrSensorModule& irModule) {
    irModule.setSilent(true);
    irModule.requestSlideStatus();
}

// Wipe ESP32-pushed runtime cal for a holder. Used on magazine insert/remove
// so a fresh GETCAL → SETCAL exchange overwrites it cleanly.
// Print "GETCAL <holder> <magType>" on MasterSerial only (not USB), so the master
// echoes it as [G] GETCAL ... → ESP32 catches it and replies with SETCAL<H><T>.
static void requestCalFromEsp(int holder, int magType) {
    if (holder < 1 || holder > 2 || magType < 1 || magType > MAX_MAG_TYPES) return;
    lastMagazineCalRequestMs[holder - 1] = millis();
    MasterSerial.print("GETCAL ");
    MasterSerial.print(holder);
    MasterSerial.print(' ');
    MasterSerial.println(magType);
}

static void requestAllCalFromEsp(int holder) {
    if (holder < 1 || holder > 2) return;
    const unsigned long now = millis();
    if (lastMagazineFullCalRequestMs[holder - 1] != 0 &&
        now - lastMagazineFullCalRequestMs[holder - 1] < 1000UL) return;
    lastMagazineFullCalRequestMs[holder - 1] = now;
    lastMagazineCalRequestMs[holder - 1] = now;
    MasterSerial.print("GETCALALL ");
    MasterSerial.println(holder);
}

static void requestMissingCalFromEsp(const MagazineIrSensorModule& irModule,
                                     int holder, int magType) {
    if (magType < 1 || magType > MAX_MAG_TYPES) return;
    const uint8_t bit = (uint8_t)(1U << (magType - 1));
    if ((irModule.getCalibrationValidMask() & bit) != 0) return;
    const unsigned long lastRequest = lastMagazineCalRequestMs[holder - 1];
    if (lastRequest == 0 || millis() - lastRequest >= MAGAZINE_CAL_RETRY_MS) {
        requestCalFromEsp(holder, magType);
    }
}

void handleMagazineInsertedAutomation(int magazine) {
    activeMagazineForLoad = magazine;

    // Fresh trigger-on IR read to accurately detect magazine type at the moment
    // of insertion — cached values from background polling can be from before the
    // magazine was fully seated.
    MagazineIrSensorModule& irModule = getMagazineIrModule(magazine);
    int magType = 0;
    clearMagazineBusState();
    I2CInstance& magazinePort = getMagazinePort(magazine);
    if (requestMagazineTriggerCommandBlocking(magazinePort, irModule.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
        if (requestFullMagazineIrValuesBlocking(irModule, magazinePort)) {
            magType = detectMagazineType(irModule);
        }
        requestMagazineTriggerCommandBlocking(magazinePort, irModule.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
    }

    Serial.print("MAG inserted: holder ");
    Serial.print(magazine);
    Serial.print(" type ");
    Serial.println(magType);

    lastMagazineType[magazine - 1] = magType;   // remember for swap-without-status-drop detection

    if (magType >= 1 && magType <= MAX_MAG_TYPES) {
        // Ask ESP32 immediately for stored cal — reply arrives as SETCAL<H><T>
        // and serviceMasterUartIntercept() applies it before the next LOAD.
        requestMissingCalFromEsp(irModule, magazine, magType);
        // Tell master the slide count
        if (irModule.isSlideStatusValid()) {
            int slides = countDetectedMagazineSlides(irModule);
            Serial.print("SLIDES: "); Serial.print(magazine); Serial.print(' '); Serial.println(slides);
        } else {
            // No cal yet — assume 0 slides until next LOAD prints an updated count
            Serial.print("SLIDES: "); Serial.print(magazine); Serial.println(" 0");
        }
    }
    // If magType == 0 (sensor read failed or non-conforming magazine), leave Gantry
    // LOAD calibration cleared. A later LOAD reports "RUN CALIBRATION" and re-asks.
}

void serviceMagazineInsertionAutomation() {
    for (int magazineIndex = 0; magazineIndex < 2; ++magazineIndex) {
        MagazineIrSensorModule& irModule = (magazineIndex == 0) ? Magazine1Ir : Magazine2Ir;
        const int currentStatus = irModule.getSlideStatusState() != 0 ? 1 : 0;
        const int currentType = irModule.getSlideMagazineType();

        if (!magazineStatusKnown[magazineIndex]) {
            magazineStatusKnown[magazineIndex] = true;
            lastMagazineStatus[magazineIndex] = currentStatus;
            lastMagazineType[magazineIndex] = currentType;
            if (currentStatus == 1) {
                activeMagazineForLoad = magazineIndex + 1;
                requestMissingCalFromEsp(irModule, magazineIndex + 1, currentType);
                MasterSerial.print("MAG inserted: holder ");
                MasterSerial.print(magazineIndex + 1);
                MasterSerial.print(" type ");
                MasterSerial.println(currentType);
            }
            continue;
        }

        // ---- Status transition detection (slow path: 1Hz polling) ----
        if (currentStatus != lastMagazineStatus[magazineIndex]) {
            lastMagazineStatus[magazineIndex] = currentStatus;
            if (currentStatus == 1) {
                activeMagazineForLoad = magazineIndex + 1;
                lastMagazineType[magazineIndex] = currentType;
                requestMissingCalFromEsp(irModule, magazineIndex + 1, currentType);
                MasterSerial.print("MAG inserted: holder ");
                MasterSerial.print(magazineIndex + 1);
                MasterSerial.print(" type ");
                MasterSerial.println(currentType);
            } else {
                // Magazine removed — clear runtime cal so a fresh insertion repulls
                Serial.print("MAG removed: holder ");
                Serial.println(magazineIndex + 1);
                lastMagazineType[magazineIndex] = 0;
                if (activeMagazineForLoad == magazineIndex + 1) {
                    activeMagazineForLoad = 0;
                }
            }
            continue;
        }

        // A type change while continuously present is a direct magazine swap.
        if (currentStatus == 1) {
            if (currentType >= 1 && currentType <= MAX_MAG_TYPES &&
                currentType != lastMagazineType[magazineIndex]) {
                lastMagazineType[magazineIndex] = currentType;
                requestMissingCalFromEsp(irModule, magazineIndex + 1, currentType);
                MasterSerial.print("MAG swap detected: holder ");
                MasterSerial.print(magazineIndex + 1);
                MasterSerial.print(" type ");
                MasterSerial.println(currentType);
            }
        }
    }
}

static void emitMagazineTelemetry(int holder, const MagazineIrSensorModule& irModule) {
    const uint8_t state = irModule.getSlideStatusState();
    const uint8_t magType = irModule.getSlideMagazineType();
    const int* slots = irModule.getSlideStatus();
    MasterSerial.print("MAGSTAT H="); MasterSerial.print(holder);
    MasterSerial.print(" V="); MasterSerial.print(state);
    MasterSerial.print(" T="); MasterSerial.print(magType);
    MasterSerial.print(" S=");
    for (uint8_t slot = 0; slot < 20; ++slot) {
        MasterSerial.print(state == 1 && slots[slot] != 0 ? '1' : '0');
    }
    MasterSerial.print(" C=");
    MasterSerial.println(irModule.getCalibrationValidMask(), HEX);
}

static void serviceMagazineSlideUpdates() {
    static uint32_t handledSequence[2] = {0, 0};
    for (int index = 0; index < 2; ++index) {
        MagazineIrSensorModule& irModule = index == 0 ? Magazine1Ir : Magazine2Ir;
        const uint32_t sequence = irModule.getSlideStatusSequence();
        if (sequence == 0 || sequence == handledSequence[index]) continue;
        handledSequence[index] = sequence;

        const uint8_t calMask = irModule.getCalibrationValidMask();
        const bool calibrationLost = magazineCalMaskKnown[index] &&
            (calMask & lastMagazineCalMask[index]) != lastMagazineCalMask[index];
        if (!magazineCalMaskKnown[index] || calibrationLost) {
            requestAllCalFromEsp(index + 1);
        }
        magazineCalMaskKnown[index] = true;
        lastMagazineCalMask[index] = calMask;

        serviceMagazineInsertionAutomation();
        emitMagazineTelemetry(index + 1, irModule);

        // GETCAL/SETCAL is asynchronous. Retry while the holder says its
        // type-specific slot thresholds have not arrived yet.
        if (irModule.getSlideStatusState() == 2) {
            const int magType = irModule.getSlideMagazineType();
            if (magType >= 1 && magType <= MAX_MAG_TYPES &&
                millis() - lastMagazineCalRequestMs[index] >= MAGAZINE_CAL_RETRY_MS) {
                requestCalFromEsp(index + 1, magType);
            }
        }
    }
}

void serviceMagazineBackgroundPolling() {
    if (!startMagzinechecks || magazine2PulseMonitorActive) {
        return;
    }

    // ---- recover from stuck pending requests ----
    // If a holder drops a response packet, isAnyRequestPending()
    // can stay true forever and polling stalls. After 2s of continuous "busy" we
    // force-clear the bus state so polling resumes.
    static unsigned long busySinceMs = 0;
    const bool busy = i2c1.hasNewData()
                   || i2c2.hasNewData()
                   || Magazine1Ir.isAnyRequestPending()
                   || Magazine2Ir.isAnyRequestPending();
    if (busy) {
        if (busySinceMs == 0) {
            busySinceMs = millis();
        } else if (millis() - busySinceMs > 2000) {
            clearMagazineBusState();
            busySinceMs = 0;
        }
        return;
    }
    busySinceMs = 0;

    static unsigned long nextPollMs = 0;
    if ((long)(millis() - nextPollMs) < 0) return;
    nextPollMs = millis() + 250UL;

    switch (magazineAutomationPollStep) {
        case MagazineAutomationPollStep::Mag1Slides:
            queueSilentMagazineSlideRequest(Magazine1Ir);
            magazineAutomationPollStep = MagazineAutomationPollStep::Mag2Slides;
            break;
        case MagazineAutomationPollStep::Mag2Slides:
            queueSilentMagazineSlideRequest(Magazine2Ir);
            magazineAutomationPollStep = MagazineAutomationPollStep::Mag1Slides;
            break;
    }
}

void serviceMagazineAutomation() {
    if (magazine2PulseMonitorActive) {
        return;
    }

    int holder = 0;
    if (magazine1AlertPending) {
        magazine1AlertPending = false;
        holder = 1;
    } else if (magazine2AlertPending) {
        magazine2AlertPending = false;
        holder = 2;
    } else {
        return;
    }

    MagazineIrSensorModule& irModule = getMagazineIrModule(holder);
    const uint8_t* eventData = requestMagazineDataBlocking(
        getMagazinePort(holder), irModule.getPcbID(), CMD_MAGAZINE_EVENT, 3, 1000);
    if (!eventData) {
        if (holder == 1) magazine1AlertPending = true;
        else magazine2AlertPending = true;
        return;
    }

    const uint8_t event = eventData[0];
    const uint8_t newType = eventData[1];
    const uint8_t previousType = eventData[2];
    if (event == MAG_EVENT_NONE) return;

    Serial.print("MAG EVENT: holder "); Serial.print(holder);
    Serial.print(" event "); Serial.print(event);
    Serial.print(" type "); Serial.print(previousType);
    Serial.print(" -> "); Serial.println(newType);

    if (event == MAG_EVENT_REMOVED || newType == 0) {
        if (activeMagazineForLoad == holder) activeMagazineForLoad = 0;
        MasterSerial.print("MAG removed: holder ");
        MasterSerial.println(holder);
        return;
    }

    activeMagazineForLoad = holder;
    MasterSerial.print(event == MAG_EVENT_SWAPPED ? "MAG swap detected: holder " : "MAG inserted: holder ");
    MasterSerial.print(holder);
    MasterSerial.print(" type ");
    MasterSerial.println(newType);
    requestMissingCalFromEsp(irModule, holder, newType);
}
}
#endif
#endif

#ifdef Master
#ifdef Nozzle_Mount_PCB

// UART to Master PCB (PC10=TX, PC11=RX — UART4) — non-static so other TUs can extern it
LivoHardwareSerial MasterSerial(MS_RX, MS_TX);
MirrorStream mirroredSerial(Serial, MasterSerial);
// Local mirror redirect for this file's Serial.print(...) calls.
#define Serial mirroredSerial

// =============================================================================
// HOMING TIMEOUT (ms) — shared by all individual home blocks
// =============================================================================
static const unsigned long NZ_HOME_TIMEOUT_MS = 30000UL;
static const long          NZ_WX_INIT_SPEED   = 12800L;  // ms=16 WX init speed (~240 RPM)
static const long          NZ_WX_INIT_POSITION = 0L;       // WX home / parked position (abs steps)

#if NZ_USE_HALL_HOME_SENSORS
static bool nzHallHomeActive(uint8_t hallPin, int reading) {
    if (hallPin == NZ_SY_HOME_HALL_PIN) return reading >= NZ_SY_HOME_HALL_THRESHOLD;
    if (hallPin == NZ_BX_HOME_HALL_PIN) return reading >= NZ_BX_HOME_HALL_THRESHOLD;
    if (hallPin == NZ_SX_HOME_HALL_PIN) return reading >= NZ_SX_HOME_HALL_THRESHOLD;
    if (hallPin == NZ_WY_HOME_HALL_PIN) return reading >= NZ_WY_HOME_HALL_THRESHOLD;
    if (hallPin == NZ_WX_HOME_HALL_PIN) return reading >= NZ_WX_HOME_HALL_THRESHOLD;
    return false;
}

static void nzSetupHallHomeInputs() {
    const uint8_t hallPins[] = {
        NZ_SY_HOME_HALL_PIN,
        NZ_BX_HOME_HALL_PIN,
        NZ_SX_HOME_HALL_PIN,
        NZ_WY_HOME_HALL_PIN,
        NZ_WX_HOME_HALL_PIN
    };
    analogReadResolution(12);
    for (uint8_t pin : hallPins) {
        // Disable the pull-up used by the legacy digital limit-switch inputs.
        pinMode(pin, INPUT);
        (void)analogRead(pin);
    }
    Serial.println("NZHO: AH49E HALL HOMING ENABLED");
}

#endif

// Both sensor types use the gantry sequence: fast contact, full fixed backoff,
// verified release, slow contact. Only the final contact establishes zero.
static bool nzReadHomeSample(uint8_t pin, bool hall, const char* tag,
                             int& raw, bool& active) {
#if NZ_USE_HALL_HOME_SENSORS
    if (hall) {
        raw = analogRead(pin);
        if (!SensorFaultPolicy::adcValid(raw)) {
            SensorFault::report("LIVO-SEN-021", tag, raw, "home_adc_invalid");
            return false;
        }
        active = nzHallHomeActive(pin, raw);
        return true;
    }
#else
    (void)hall;
    (void)tag;
#endif
    raw = digitalRead(pin);
    active = raw == LOW;
    return true;
}

static bool nzSeekHome(TMCModule& motor, const char* tag, uint8_t pin,
                        bool hall, long direction, bool slow, int& raw) {
    Serial.print(tag); Serial.println(slow ? ": slow approach" : ": fast approach");
    motor.setMaxSpeed(slow ? NZ_HOME_SLOW_SPEED : NZ_HOME_SPEED);
    const unsigned long started = millis();
    uint8_t stable = 0;
    while (true) {
        bool active = false;
        if (!nzReadHomeSample(pin, hall, tag, raw, active)) return false;
        if (active) {
            if (++stable >= NZ_HOME_STABLE_SAMPLES) {
                // Cancel immediately, as for GX/GZ; do not decelerate past home.
                motor.setCurrentPosition(motor.getCurrentPosition());
                Serial.print(tag);
                Serial.print(slow ? ": slow triggered=" : ": fast triggered=");
                Serial.println(raw);
                return true;
            }
        } else stable = 0;
        if (millis() - started >= NZ_HOME_TIMEOUT_MS) {
            SensorFault::report(hall ? "LIVO-NOZ-007" : "LIVO-SEN-023", tag, raw,
                                slow ? "slow_home_timeout" : "fast_home_timeout");
            return false;
        }
        if (!motor.isMoving()) {
            motor.move(direction * (slow ? NZ_HOME_SLOW_STEP_CHUNK : 2000000000L));
        }
        motor.loop();
        limitSwitch.loop();
    }
}

static bool nzHomeCycle(TMCModule& motor, const char* tag, uint8_t pin,
                         bool hall, long steps) {
    // Every return cancels pending movement and restores the caller's speed.
    struct Restore {
        TMCModule& motor; long speed;
        ~Restore() {
            motor.setCurrentPosition(motor.getCurrentPosition());
            motor.setMaxSpeed(speed);
        }
    } restore{motor, motor.getMaxSpeed()};
    const long direction = steps > 0 ? 1L : -1L;
    int raw = 0;
    bool active = false;
    if (!nzReadHomeSample(pin, hall, tag, raw, active)) return false;
    Serial.print(tag); Serial.print(hall ? ": hall initial=" : ": switch initial="); Serial.println(raw);
    // An already active input is first contact; never drive further into it.
    if (!active && !nzSeekHome(motor, tag, pin, hall, direction, false, raw)) return false;
    if (active) { Serial.print(tag); Serial.println(": initially active; backing off"); }

    motor.setMaxSpeed(NZ_HOME_SPEED);
    Serial.print(tag); Serial.print(": backoff steps="); Serial.println(NZ_HOME_BACKOFF_STEPS);
    motor.move(-direction * NZ_HOME_BACKOFF_STEPS);
    const unsigned long started = millis();
    while (motor.isMoving()) {
        if (!nzReadHomeSample(pin, hall, tag, raw, active)) return false;
        if (millis() - started >= NZ_HOME_BACKOFF_TIMEOUT_MS) {
            SensorFault::report("LIVO-NOZ-007", tag, raw, "home_backoff_timeout");
            return false;
        }
        motor.loop();
        limitSwitch.loop();
    }
    // Check after ALL backoff steps, not at the first moment the input clears.
    for (uint8_t sample = 0; sample < NZ_HOME_STABLE_SAMPLES; ++sample) {
        if (!nzReadHomeSample(pin, hall, tag, raw, active)) return false;
        if (active) {
            SensorFault::report("LIVO-SEN-022", tag, raw, "home_release_not_confirmed_sensor_or_motion_unknown");
            return false;
        }
        delay(1);
    }
    Serial.print(tag); Serial.print(": backoff clear="); Serial.println(raw);
    if (!nzSeekHome(motor, tag, pin, hall, direction, true, raw)) return false;
    motor.setCurrentPosition(0);
    return true;
}

#if NZ_USE_HALL_HOME_SENSORS
static bool nzRunToHall(TMCModule& motor, const char* tag, uint8_t hallPin, long steps) {
    pinMode(hallPin, INPUT);
    return nzHomeCycle(motor, tag, hallPin, true, steps);
}
#endif

static bool nzRunToSwitch(TMCModule& motor, const char* tag, uint8_t limPin, long steps) {
    return nzHomeCycle(motor, tag, limPin, false, steps);
}

static bool nzRunToHomeSensor(TMCModule& motor, const char* tag,
                              uint8_t legacyLimitPin, uint8_t hallPin,
                              long hallSteps, long legacySteps) {
#if NZ_USE_HALL_HOME_SENSORS
    (void)legacyLimitPin;
    (void)legacySteps;
    return nzRunToHall(motor, tag, hallPin, hallSteps);
#else
    (void)hallPin;
    (void)hallSteps;
    return nzRunToSwitch(motor, tag, legacyLimitPin, legacySteps);
#endif
}

// ---- Individual homing blocks ----

static bool homeSY() {
    // SY: negative direction → IR3 Hall / Lim2 fallback
    Serial.println("SYHO: homing");
    long savedSpeed = SYMotor.getMaxSpeed();
    SYMotor.setMaxSpeed(NZ_HOME_SPEED);
    if (!nzRunToHomeSensor(SYMotor, "SYHO", Lim2, NZ_SY_HOME_HALL_PIN,
                           -2000000000L, -2000000000L)) { SYMotor.setMaxSpeed(savedSpeed); return false; }
    SYMotor.setMaxSpeed(savedSpeed);
    Serial.println("SYHO: done");
    return true;
}

static bool homeBX() {
    // BX: positive direction → IR11 Hall / Lim4 fallback
    Serial.println("BXHO: homing");
    long savedSpeed = BXMotor.getMaxSpeed();
    BXMotor.setMaxSpeed(NZ_HOME_SPEED);
    if (!nzRunToHomeSensor(BXMotor, "BXHO", Lim4, NZ_BX_HOME_HALL_PIN,
                           2000000000L, 2000000000L)) { BXMotor.setMaxSpeed(savedSpeed); return false; }
    BXMotor.setMaxSpeed(savedSpeed);
    Serial.println("BXHO: done");
    return true;
}

static bool homeSX() {
    // SX: positive direction → IR13 Hall / Lim5 fallback
    Serial.println("SXHO: homing");
    long savedSpeed = SXMotor.getMaxSpeed();
    SXMotor.setMaxSpeed(NZ_HOME_SPEED);
    if (!nzRunToHomeSensor(SXMotor, "SXHO", Lim5, NZ_SX_HOME_HALL_PIN,
                           2000000000L, 2000000000L)) { SXMotor.setMaxSpeed(savedSpeed); return false; }
    SXMotor.setMaxSpeed(savedSpeed);
    Serial.println("SXHO: done");
    return true;
}

static bool homeWY() {
    // WY: IR7 Hall homes negative; retained Lim6 fallback homes positive.
    Serial.println("WYHO: homing");
    long savedSpeed = WYMotor.getMaxSpeed();
    WYMotor.setMaxSpeed(NZ_HOME_SPEED);
    if (!nzRunToHomeSensor(WYMotor, "WYHO", Lim6, NZ_WY_HOME_HALL_PIN,
                           -2000000000L, 2000000000L)) { WYMotor.setMaxSpeed(savedSpeed); return false; }
    WYMotor.setMaxSpeed(savedSpeed);
    Serial.println("WYHO: done; staying at home (0)");
    return true;
}

static bool homeWX() {
    // WX: positive direction → IR10 Hall / Lim3 fallback
    Serial.println("WXHO: homing");
    long savedSpeed = WXMotor.getMaxSpeed();
    WXMotor.setMaxSpeed(NZ_HOME_SPEED);
    if (!nzRunToHomeSensor(WXMotor, "WXHO", Lim3, NZ_WX_HOME_HALL_PIN,
                           2000000000L, 2000000000L)) { WXMotor.setMaxSpeed(savedSpeed); return false; }
    Serial.println("WXHO: done");
    // Move WX to initial position
    Serial.print("WXHO: moving to initial position (");
    Serial.print(NZ_WX_INIT_POSITION);
    Serial.println(")");
    WXMotor.setMaxSpeed(NZ_WX_INIT_SPEED);
    WXMotor.moveTo(NZ_WX_INIT_POSITION);
    unsigned long t = millis();
    while (WXMotor.isMoving()) {
        WXMotor.loop();
        if (millis() - t > NZ_HOME_TIMEOUT_MS) {
            Serial.println("WXHO: initial move timeout");
            WXMotor.setMaxSpeed(savedSpeed);
            return false;
        }
    }
    WXMotor.setMaxSpeed(savedSpeed);
    Serial.println("WXHO: at initial position");
    return true;
}



// =============================================================================
// STAIN PROFILE — nozzle-side runtime state
// Master pushes "PROFILE <code>" over MasterSerial on every change and at
// boot (once Nozzle: Ready is acknowledged). Until that arrives, the default
// is PROFILE_RP — same as master's boot default.
// Declared here (before runNzhoBlocking) so the NZHO BX park step can read
// nzProfile().bx_position.
// =============================================================================
static ProfileId nzActiveProfileId = PROFILE_RP;
static bool nzHomingComplete = false;
static inline const StainProfile& nzProfile() { return PROFILES[nzActiveProfileId]; }
static inline long nzSXParkPositionForProfile(ProfileId id) {
    if (id == PROFILE_WG) return NZ_SX_WG_PARK_POSITION;
    if (id == PROFILE_MG) return NZ_SX_MG_PARK_POSITION;
    return NZ_SX_DEFAULT_PARK_POSITION;
}

static inline bool nzSXIsFixedForProfile(ProfileId id) {
    return id == PROFILE_MG || id == PROFILE_WG;
}

// MG/WG use a fixed drying head: WX remains parked at its home position while
// DCF3 runs for the duration that the normal WX forward stroke would take.
static inline bool nzWXIsFixedForProfile(ProfileId id) {
    return id == PROFILE_MG || id == PROFILE_WG;
}

static inline long nzWYStrokeForProfile(ProfileId id) {
    return (id == PROFILE_MG || id == PROFILE_WG)
        ? NZ_WY_MG_WG_STROKE
        : NZ_WY_DEFAULT_STROKE;
}

// Apply a new profile by ID. Logs and updates the runtime variable.
static void nzApplyProfile(ProfileId newId) {
    if (newId >= PROFILE_COUNT) {
        Serial.print("[nz PROFILE] reject: invalid ID "); Serial.println((int)newId);
        return;
    }
    if (newId == nzActiveProfileId) return;
    nzActiveProfileId = newId;
    const StainProfile& p = nzProfile();
    Serial.print("[nz PROFILE] active="); Serial.print(p.code);
    Serial.print(" sph=");                Serial.print(currentSph);
    Serial.print(" bed-steps: M14="); Serial.print(timingStepsFor(newId, TK_M14_AT));
    Serial.print(" S1=");                  Serial.print(timingStepsFor(newId, TK_S1_AT));
    Serial.print(" B1=");                  Serial.print(timingStepsFor(newId, TK_B1_AT));
    Serial.print(" S2MIX=");               Serial.println(timingStepsFor(newId, TK_S2MIX_AT));

    // Startup profile synchronization must not move unhomed axes.
    if (!nzHomingComplete) {
        Serial.println("[nz PROFILE] park deferred until manual NZHO");
        return;
    }

    // Re-park BX to the new profile's target. Master rejects PROFILE changes
    // while slides are in flight, so when a push arrives the BX axis is safe
    // to move. Non-blocking: BXMotor.loop() in the main loop services the move.
    //
    // Manual NZHO establishes the origin and applies the current profile's
    // park positions. Subsequent profile changes can re-park the axes here.
    const long bxTarget  = p.bx_position;
    const long bxCurrent = BXMotor.getCurrentPosition();
    // Unconditional log so the operator can confirm the new code path is in
    // the flashed binary, even when the move is a no-op (current == target).
    Serial.print("[nz PROFILE] BX check: current="); Serial.print(bxCurrent);
    Serial.print(" target=");                        Serial.print(bxTarget);
    Serial.print(" moving=");                        Serial.println(BXMotor.isMoving() ? "yes" : "no");
    if (bxCurrent != bxTarget) {
        if (BXMotor.isMoving()) {
            Serial.print("[nz PROFILE] BX re-park deferred — motor already moving. wanted=");
            Serial.println(bxTarget);
        } else {
            Serial.print("[nz PROFILE] BX re-park "); Serial.print(bxCurrent);
            Serial.print(" → ");                     Serial.println(bxTarget);
            BXMotor.moveTo(bxTarget);
        }
    } else {
        Serial.println("[nz PROFILE] BX already at target — no move");
    }

    // MG/WG use profile-specific SX park positions and remain fixed there
    // throughout stain automation.
    const long sxTarget  = nzSXParkPositionForProfile(newId);
    const long sxCurrent = SXMotor.getCurrentPosition();
    Serial.print("[nz PROFILE] SX check: current="); Serial.print(sxCurrent);
    Serial.print(" target=");                        Serial.print(sxTarget);
    Serial.print(" moving=");                        Serial.println(SXMotor.isMoving() ? "yes" : "no");
    if (sxCurrent != sxTarget) {
        if (SXMotor.isMoving()) {
            Serial.print("[nz PROFILE] SX re-park deferred — motor already moving. wanted=");
            Serial.println(sxTarget);
        } else {
            Serial.print("[nz PROFILE] SX re-park "); Serial.print(sxCurrent);
            Serial.print(" → ");                     Serial.println(sxTarget);
            SXMotor.moveTo(sxTarget);
        }
    } else {
        Serial.println("[nz PROFILE] SX already at target — no move");
    }
}

// ---- NZHO: call all individual blocks in sequence ----
static bool runNzhoBlocking() {
    nzHomingComplete = false;
    Serial.println("NZHO: homing all nozzle mount axes");
    if (!homeSY())  { Serial.println("NZHO: SY failed");  return false; }
    if (!homeBX())  { Serial.println("NZHO: BX failed");  return false; }
    if (!homeSX())  { Serial.println("NZHO: SX failed");  return false; }
    if (!homeWY())  { Serial.println("NZHO: WY failed");  return false; }
	if (!homeWX())  { Serial.println("NZHO: WX failed");  return false; }

    // Move every profile-dependent station from its fresh home origin to the
    // selected recipe's park position. NZHO is not complete until both moves
    // finish, so Check Systems cannot report ready at generic home positions.
    const long bxTarget = nzProfile().bx_position;
    const long sxTarget = nzSXParkPositionForProfile(nzActiveProfileId);
    Serial.print("NZHO: profile park profile="); Serial.print(nzProfile().code);
    Serial.print(" BX="); Serial.print(bxTarget);
    Serial.print(" SX="); Serial.println(sxTarget);
    BXMotor.moveTo(bxTarget);
    SXMotor.moveTo(sxTarget);
    const unsigned long parkStartMs = millis();
    while (BXMotor.isMoving() || SXMotor.isMoving()) {
        BXMotor.loop();
        SXMotor.loop();
        if (millis() - parkStartMs > NZ_HOME_TIMEOUT_MS) {
            BXMotor.stop();
            SXMotor.stop();
            while (BXMotor.isMoving() || SXMotor.isMoving()) {
                BXMotor.loop();
                SXMotor.loop();
            }
            Serial.println("NZHO: profile park timeout");
            return false;
        }
    }
    Serial.println("NZHO: profile park reached");

    nzHomingComplete = true;
    Serial.println("NZHO: ready");
    return true;
}

// =============================================================================
// NOZZLE AUTOMATION — ALL CONFIGURABLE VALUES, IN EXECUTION ORDER
// Edit values here. Do not touch the logic below.
// Master-side cascade constants (DCM1/DCM2, DCW1/DCS1, S2/X, B2/Y, DCF1,
// DCW2/DCS2) live in the Stainer_Master_PCB section of this file.
// =============================================================================

// -----------------------------------------------------------------------------
// IR DETECTION THRESHOLDS  (IR value BELOW this = slide present)
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Per-SPH ratios — calibrated at 180 sph: ratio = value_at_180 / 180
// Formula: actual_value = ratio * current_sph (updated by nzUpdateDelaysForFeedSpeed)
// -----------------------------------------------------------------------------

// =============================================================================
// Recipe row chooses spatial coordinates and nozzle motor speeds.
unsigned long currentSph    = 180;
SpeedIdx      currentSphIdx = SPH_180;

// Updates both currentSph (numeric, for logs) and currentSphIdx (snapped table
// row used by timingStepsFor() / speedFor()). Operator-supplied SPH values that
// don't match a discrete row exactly are snapped to the nearest row.
static void nzUpdateDelaysForFeedSpeed(int sph) {
    if (sph <= 0) return;
    currentSph    = (unsigned long)sph;
    const SpeedIdx newIdx = speedIdxForSph(currentSph);
    if (newIdx != currentSphIdx) {
        Serial.print("[nz FEED] sph=");      Serial.print(currentSph);
        Serial.print(" snapped to row=");    Serial.print(SPEED_VALUES[newIdx]);
        Serial.print(" (idx=");              Serial.print((int)newIdx); Serial.println(")");
        currentSphIdx = newIdx;
    }
}

// =============================================================================
// STATE MACHINES
// =============================================================================

// Each IR4 detection owns its pump and fan deadlines until completion.
static SlideDetectionFilter nzBedFilters[3];
static int nzBedDepth();
static bool nzFan4Running = false;

// Opt-in IR4 trace: sample at most every 10 ms, summarize every 100 ms.
// No motion, threshold, filter, or scheduling changes. PIR 4 starts a bounded
// ten-minute capture; PIR 0 stops it. Printing every loop would stall motors.
static bool nzIR4TraceEnabled = false;
static uint32_t nzIR4TraceStarted = 0, nzIR4TraceSampleAt = 0, nzIR4TraceReportAt = 0;
static uint32_t nzIR4TraceMaxGap = 0, nzIR4TraceRearms = 0;
static bool nzIR4TraceWasArmed = false;
static int nzIR4RawMin = 4096, nzIR4RawMax = -1;
static int nzIR4FilteredMin = 4096, nzIR4FilteredMax = -1;
static uint16_t nzIR4TraceSamples = 0;

static void nzSetIR4Trace(bool enabled) {
    nzIR4TraceEnabled = enabled;
    nzIR4TraceStarted = nzIR4TraceSampleAt = nzIR4TraceReportAt = millis();
    nzIR4TraceMaxGap = nzIR4TraceRearms = 0;
    nzIR4TraceWasArmed = nzBedFilters[0].isArmed();
    nzIR4RawMin = nzIR4FilteredMin = 4096;
    nzIR4RawMax = nzIR4FilteredMax = -1;
    nzIR4TraceSamples = 0;
    Serial.println(enabled ? "[IR4TRACE] ON 600000ms raw/filtered ranges; N; PIR 0 stops" : "[IR4TRACE] OFF");
}

static void nzServiceIR4Trace(int filtered) {
    if (!nzIR4TraceEnabled) return;
    const uint32_t now = millis();
    if (now - nzIR4TraceStarted >= 600000UL) { nzSetIR4Trace(false); return; }
    const bool armed = nzBedFilters[0].isArmed();
    if (armed && !nzIR4TraceWasArmed) ++nzIR4TraceRearms;
    nzIR4TraceWasArmed = armed;
    const uint32_t gap = now - nzIR4TraceSampleAt;
    if (gap < 10UL) return;
    nzIR4TraceSampleAt = now;
    if (gap > nzIR4TraceMaxGap) nzIR4TraceMaxGap = gap;
    const int raw = analogRead(IR4);
    if (raw < nzIR4RawMin) nzIR4RawMin = raw;
    if (raw > nzIR4RawMax) nzIR4RawMax = raw;
    if (filtered < nzIR4FilteredMin) nzIR4FilteredMin = filtered;
    if (filtered > nzIR4FilteredMax) nzIR4FilteredMax = filtered;
    ++nzIR4TraceSamples;
    if (now - nzIR4TraceReportAt < 100UL) return;
    Serial.print("[IR4TRACE] t="); Serial.print(now);
    Serial.print(" raw="); Serial.print(nzIR4RawMin); Serial.print('/'); Serial.print(nzIR4RawMax);
    Serial.print(" filtered="); Serial.print(nzIR4FilteredMin); Serial.print('/'); Serial.print(nzIR4FilteredMax);
    Serial.print(" armed="); Serial.print(armed ? 1 : 0);
    Serial.print(" rearms="); Serial.print(nzIR4TraceRearms);
    Serial.print(" q="); Serial.print(nzBedDepth());
    Serial.print(" n="); Serial.print(nzIR4TraceSamples);
    Serial.print(" gap="); Serial.println(nzIR4TraceMaxGap);
    nzIR4TraceReportAt = now;
    nzIR4RawMin = nzIR4FilteredMin = 4096;
    nzIR4RawMax = nzIR4FilteredMax = -1;
    nzIR4TraceSamples = 0; nzIR4TraceMaxGap = 0;
}


// S1/B1/SXCAS/WYCAS state lives in queues (nzS1Queue, nzS2Queue, nzSxCasQueue, nzWyCasQueue)
// declared further below; their delays and volumes are in the constants block above.

// =============================================================================
// BEDTEST — single-slide step-based stain test
// =============================================================================
// Runs the full RP-profile stain timeline for ONE slide based on T-motor step
// position rather than nested millis()-anchored delays. On IR4 the slide is
// anchored (master resets T counter). Each event fires when stepsSinceIR4
// crosses its absolute threshold (the threshold is computed from the universal
// step values in PROFILE_TIMINGS + the measured BED_STEPS_IR4_TO_IR6/IR12
// constants). Slide-position events on master fire via REQ:* messages (same
// as production); nozzle motor strokes (SY/SX/WY/WX) are PRINTED only for the
// pilot — once verified, the real motor handlers will be hooked up.
//
// All other automation is disabled while BEDTEST is active (see early-return
// in serviceNozzleAutomation()).
// =============================================================================
static bool nzParaActive = false;
static bool nzParaAnchored = false;
static bool nzParaStainSeen = false, nzParaWashSeen = false;
static bool nzParaFanEnded = false, nzParaIdleSent = false;
static void nzParaEvent(const char* event) {
    if (!nzParaActive || !nzParaAnchored) return;
    MasterSerial.print("PCE "); MasterSerial.println(event);
}
static void nzParaMotorEvent(const char* motor, const char* phase) {
    if (!nzParaActive || !nzParaAnchored) return;
    char label[32];
    snprintf(label, sizeof(label), "%s_%s", motor, phase);
    nzParaEvent(label);
}
// Per-motor sub-states (run independently in parallel)
enum NzMotorOpState { MOP_IDLE, MOP_TO_INIT, MOP_STROKING, MOP_RETURNING };

struct NzMotorTraceState {
    unsigned long activeSequenceId;
    unsigned long detectedAtMs;
    unsigned long firedAtMs;
    long returnPosition;
};






// Opt-in IR12 trace: sample at most every 10 ms, summarize every 500 ms.
// No motion, threshold, filter, or scheduling changes. PIR 12 starts a bounded
// ten-minute capture; PIR 0 stops it. Printing every loop would stall motors.
static bool nzIR12TraceEnabled = false;
static uint32_t nzIR12TraceStarted = 0, nzIR12TraceSampleAt = 0, nzIR12TraceReportAt = 0;
static uint32_t nzIR12TraceMaxGap = 0, nzIR12TraceRearms = 0;
static bool nzIR12TraceWasArmed = false;
static int nzIR12RawMin = 4096, nzIR12RawMax = -1;
static int nzIR12FilteredMin = 4096, nzIR12FilteredMax = -1;
static uint16_t nzIR12TraceSamples = 0;

static void nzSetIR12Trace(bool enabled) {
    nzIR12TraceEnabled = enabled;
    nzIR12TraceStarted = nzIR12TraceSampleAt = nzIR12TraceReportAt = millis();
    nzIR12TraceMaxGap = nzIR12TraceRearms = 0;
    nzIR12TraceWasArmed = nzBedFilters[2].isArmed();
    nzIR12RawMin = nzIR12FilteredMin = 4096;
    nzIR12RawMax = nzIR12FilteredMax = -1;
    nzIR12TraceSamples = 0;
    Serial.println(enabled ? "[IR12TRACE] ON 600000ms raw/filtered ranges; N; PIR 0 stops" : "[IR12TRACE] OFF");
}

static void nzServiceIR12Trace(int filtered) {
    if (!nzIR12TraceEnabled) return;
    const uint32_t now = millis();
    if (now - nzIR12TraceStarted >= 600000UL) { nzSetIR12Trace(false); return; }
    const bool armed = nzBedFilters[2].isArmed();
    if (armed && !nzIR12TraceWasArmed) ++nzIR12TraceRearms;
    nzIR12TraceWasArmed = armed;
    const uint32_t gap = now - nzIR12TraceSampleAt;
    if (gap < 10UL) return;
    nzIR12TraceSampleAt = now;
    if (gap > nzIR12TraceMaxGap) nzIR12TraceMaxGap = gap;
    const int raw = analogRead(IR12);
    if (raw < nzIR12RawMin) nzIR12RawMin = raw;
    if (raw > nzIR12RawMax) nzIR12RawMax = raw;
    if (filtered < nzIR12FilteredMin) nzIR12FilteredMin = filtered;
    if (filtered > nzIR12FilteredMax) nzIR12FilteredMax = filtered;
    ++nzIR12TraceSamples;
    if (now - nzIR12TraceReportAt < 500UL) return;
    Serial.print("[IR12TRACE] t="); Serial.print(now);
    Serial.print(" raw="); Serial.print(nzIR12RawMin); Serial.print('/'); Serial.print(nzIR12RawMax);
    Serial.print(" filtered="); Serial.print(nzIR12FilteredMin); Serial.print('/'); Serial.print(nzIR12FilteredMax);
    Serial.print(" armed="); Serial.print(armed ? 1 : 0);
    Serial.print(" rearms="); Serial.print(nzIR12TraceRearms);
    Serial.print(" detectedAt="); Serial.print(nzBedFilters[2].detected);
    Serial.print(" n="); Serial.print(nzIR12TraceSamples);
    Serial.print(" gap="); Serial.println(nzIR12TraceMaxGap);
    nzIR12TraceReportAt = now;
    nzIR12RawMin = nzIR12FilteredMin = 4096;
    nzIR12RawMax = nzIR12FilteredMax = -1;
    nzIR12TraceSamples = 0; nzIR12TraceMaxGap = 0;
}






static NzMotorOpState nzSYOpState = MOP_IDLE;
static NzMotorOpState nzBXOpState = MOP_IDLE;
static NzMotorOpState nzSXOpState = MOP_IDLE;
static NzMotorOpState nzWYOpState = MOP_IDLE;
static NzMotorOpState nzWXOpState = MOP_IDLE;

// Saved pre-stroke speeds — restored after return-to-initial
static long nzSYSavedSpeed = 0L;
static long nzBXSavedSpeed = 0L;
static long nzSXSavedSpeed = 0L;
static long nzWYSavedSpeed = 0L;
static long nzWXSavedSpeed = 0L;

static int nzLastReportedPipelineDepth = -1;   // -1 = never reported; forces first send

static int nzBedDepth();
static int nzTotalPipelineDepth() {
    return nzBedDepth();
}

static void nzReportPipelineDepthIfChanged() {
    const int d = nzTotalPipelineDepth();
    if (d == nzLastReportedPipelineDepth) return;
    nzLastReportedPipelineDepth = d;
    MasterSerial.print("NZQ ");
    MasterSerial.println(d);
    if (d == 0) {
        Serial.println("[nz] pipeline drained (NZQ=0)");
    }
}

static NzMotorTraceState nzSYTrace = {};
static NzMotorTraceState nzBXTrace = {};
static NzMotorTraceState nzSXTrace = {};
static NzMotorTraceState nzWYTrace = {};
static NzMotorTraceState nzWXTrace = {};
static bool          nzWXParkedFanActive = false;


static void nzStartWXStrokeFan() {
    dcFan3.runHighSpeedFanPercent(NZ_WX_DCF3_SPEED_PCT);
    nzParaEvent("WX_FAN_ON");
    if (displayDebug) {
        Serial.print("IR12DBG FAN motor=WX action=ON pct=");
        Serial.print(NZ_WX_DCF3_SPEED_PCT);
        Serial.print(" now=");
        Serial.println(millis());
    }
}

static void nzStopWXStrokeFan() {
    dcFan3.stopHighSpeedFan();
    nzParaEvent("WX_FAN_OFF");
    if (nzParaActive) nzParaFanEnded = true;
    if (displayDebug) {
        Serial.print("IR12DBG FAN motor=WX action=OFF now=");
        Serial.println(millis());
    }
}

static const char* nzMotorStateName(NzMotorOpState state) {
    switch (state) {
        case MOP_IDLE:      return "IDLE";
        case MOP_TO_INIT:   return "TO_INIT";
        case MOP_STROKING:  return "STROKING";
        case MOP_RETURNING: return "RETURNING";
        default:            return "UNKNOWN";
    }
}

// Non-blocking service for one motor sub-state
// Sequence: go to ready position → stroke → return to ready position at returnSpeed
// → restore saved speed → idle.
static void nzServiceMotor(const char* debugPrefix, TMCModule& motor, const char* motorName, NzMotorOpState& state, long stroke,
                           long& savedSpeed, long returnSpeed, NzMotorTraceState& trace,
                           void (*onStrokeStart)() = nullptr, void (*onStrokeEnd)() = nullptr) {
    switch (state) {
        case MOP_TO_INIT:
            if (!motor.isMoving()) {
                if (displayDebug) {
                    Serial.print(debugPrefix); Serial.print(" STROKE motor="); Serial.print(motorName);
                    Serial.print(" seq="); Serial.print(trace.activeSequenceId);
                    Serial.print(" now="); Serial.print(millis());
                    Serial.print(" pos="); Serial.print(motor.getCurrentPosition());
                    Serial.print(" stroke="); Serial.println(stroke);
                }
                if (onStrokeStart != nullptr) {
                    onStrokeStart();
                }
                motor.move(stroke);
                nzParaMotorEvent(motorName, "STROKE_START");
                state = MOP_STROKING;
            }
            break;
        case MOP_STROKING:
            if (!motor.isMoving()) {
                if (displayDebug) {
                    Serial.print(debugPrefix); Serial.print(" RETURN motor="); Serial.print(motorName);
                    Serial.print(" seq="); Serial.print(trace.activeSequenceId);
                    Serial.print(" now="); Serial.print(millis());
                    Serial.print(" pos="); Serial.print(motor.getCurrentPosition());
                    Serial.print(" target="); Serial.println(trace.returnPosition);
                }
                if (onStrokeEnd != nullptr) {
                    onStrokeEnd();
                }
                motor.setMaxSpeed(returnSpeed);
                motor.moveTo(trace.returnPosition);
                nzParaMotorEvent(motorName, "RETURN_START");
                state = MOP_RETURNING;
            }
            break;
        case MOP_RETURNING:
            if (!motor.isMoving()) {
                if (displayDebug) {
                    Serial.print(debugPrefix); Serial.print(" DONE motor="); Serial.print(motorName);
                    Serial.print(" seq="); Serial.print(trace.activeSequenceId);
                    Serial.print(" now="); Serial.print(millis());
                    Serial.print(" total="); Serial.print(millis() - trace.detectedAtMs);
                    Serial.print(" fireToDone="); Serial.print(millis() - trace.firedAtMs);
                    Serial.print(" pos="); Serial.println(motor.getCurrentPosition());
                }
                motor.setMaxSpeed(savedSpeed);
                state = MOP_IDLE;
                nzParaMotorEvent(motorName, "RETURN_DONE");
                trace.activeSequenceId = 0UL;
                trace.detectedAtMs = 0UL;
                trace.firedAtMs = 0UL;
                trace.returnPosition = 0L;
            }
            break;
        default: break;
    }
}







// =============================================================================
#include "NozzleBedProduction.inc"

static void serviceNozzleAutomation() {
    serviceNozzleBedProduction();
    nzReportSlidePositions();
}
#endif
#endif

#ifdef Master
#ifdef Stainer_Master_PCB
// currentSph + currentSphIdx are declared extern in STAIN-PARAMETERS.h; defined
// here for the master MCU. Updated by the FEED command. timingStepsFor() consumes
// currentSph (live steps→ms conversion). speedFor() consumes currentSphIdx
// (still per-FEED motor speeds via 3D PROFILE_SPEEDS lookup).
unsigned long currentSph    = 180;
SpeedIdx      currentSphIdx = SPH_180;
static bool  tFeedActive  = false;
static uint32_t masterBedPosition=0;
static bool masterBedFault=false;
static void masterBedFail(uint32_t reason);
static bool masterBedStart();
static bool masterBedWindowsActive();
// Set by the ESP32 balance authority. When true, slide transport and all
// wash/suction/drain operations continue, but reagent dispensing is inhibited.
static bool reagentDispenseLocked = false;
// While FEED is active, auto-trigger DRAIN every FEED_DRAIN_INTERVAL_MS.
static float tFeedSpeed = 0;

// =============================================================================
// MASTER AUTOMATION — ALL CONFIGURABLE VALUES, IN EXECUTION ORDER
// Edit values here. Nozzle-side delays/strokes live in the Nozzle_Mount_PCB
// section of this file.
// =============================================================================

// -----------------------------------------------------------------------------
// PUMP FLOW RATES (mL per shaft revolution) — calibrate per tube
// Used by M14ML / ZML / M15ML / XML / YML commands and by cascade dispenses.
// -----------------------------------------------------------------------------

// E PUMP (M14 motor) — REQ:M14ML <µL>


// S1 PUMP (Z motor) — REQ:ZML <µL>


// B1 PUMP (M15 motor) — direct command: REQ:M15ML <µL>; cascade trigger: REQ:B1MIX


// S2 PUMP (X motor) — XML / used inside SXCAS


// B2 PUMP (Y motor) — YML / used inside SXCAS


// mL → step-count helpers (sign-positive; caller chooses direction)
static inline long m14StepsForMl(float ml) { return (long)((ml * (float)M14_STEPS_PER_REV) / M14_FLOW_ML_PER_REV + 0.5f); }
static inline long zStepsForMl(float ml)   { return (long)((ml * (float)Z_STEPS_PER_REV)   / Z_FLOW_ML_PER_REV   + 0.5f); }
static inline long m15StepsForMl(float ml) { return (long)((ml * (float)M15_STEPS_PER_REV) / M15_FLOW_ML_PER_REV + 0.5f); }

// -----------------------------------------------------------------------------
// DC MOTOR & FAN PWM DUTIES (0–255) — one constant per device
// Used by cascades and internal triggers. Direct DCxx commands still take
// their own command-supplied duty value.
// -----------------------------------------------------------------------------

// =============================================================================
// B1 pump completion state; air ON/OFF are independent IR4 positions.
// =============================================================================

// DCM1/DCM2 run together at air_on and stop at air_off.

// DUTIES: DCM1_DUTY / DCM2_DUTY (from DC duty block above)
// Dispense volume for the B1 cascade (M15 pump):


enum M15MixState {
    M15MIX_IDLE = 0,
    M15MIX_WAIT_FINISH,   // M15Motor moving; waiting for it to stop
    M15MIX_WAIT_PRE,      // legacy state, unused by production
    M15MIX_RUNNING,       // legacy state, unused by production
};
static M15MixState   m15MixState     = M15MIX_IDLE;
static unsigned long m15MixArmStep   = 0UL;

// =============================================================================
// Legacy SXCAS state retained for reset compatibility.
// Wash-1 and Suction-1 use independent absolute IR4 positions.
// =============================================================================

// Wash-1 is independently controlled by w1_on/w1_off.


// Suction-1 is independently controlled by suction1_on/suction1_off.


enum SxCasState { SXCAS_IDLE = 0, SXCAS_ACTIVE };
static SxCasState    sxCasState        = SXCAS_IDLE;
static unsigned long sxCasStartStep      = 0UL;
static bool          sxCasDcw1Started  = false;
static bool          sxCasDcw1Stopped  = false;
static bool          sxCasDcs1Started  = false;
static bool          sxCasDcs1Stopped  = false;

// =============================================================================
// S2/B2 preparation and M18 mixing start at s2mix_at.
// Final M18 dispatch starts independently at s2disp_at.
// =============================================================================

// S2 PUMP (X) — FLOW (delay = 0; fires immediately at S2MIX arm)


// B2 PUMP (Y) — FLOW (delay = 0)


// M18 oscillates until its absolute dispatch position, then runs its
// calibrated forward motor-step target. Pump completion cannot move that position.

enum S2MixState { S2MIX_IDLE = 0, S2MIX_ACTIVE };
static S2MixState    s2MixState        = S2MIX_IDLE;
static unsigned long s2MixStartStep      = 0UL;
static bool          s2MixPumpsEnded   = false;
static unsigned long s2MixPumpsEndStep   = 0UL;
static bool          s2MixComplete       = false;   // cascade complete

// M18 oscillation state machine — runs forward → reverse → forward → … from
// cascade arm until cascade ends. Direction flips when M18Motor.isMoving()
// returns false at the end of each half-cycle.
enum M18OscState : uint8_t { M18_OSC_IDLE = 0, M18_OSC_FORWARD, M18_OSC_REVERSE };
static M18OscState m18OscState = M18_OSC_IDLE;


// Dispatch phase — after mixing finishes, M18 runs continuous forward to push
// the mixed reagent through the ~100 mm pipe onto the slide.
static bool          s2MixDispatchStarted = false;
static unsigned long s2MixDispatchStartStep = 0UL;

// =============================================================================
// ─── CASCADE C: WYCAS  (WY start → DCW2, DCS2) ──────────────────────────────
// Triggered by nozzle's REQ:WYCAS.
// =============================================================================

// DCW2 — DELAY, DURATION   (duty digital ON, no PWM)


// DCS2 — DELAY, DURATION   (duty: DCS2_DUTY)


enum WyCasState { WYCAS_IDLE = 0, WYCAS_ACTIVE };
static WyCasState    wyCasState        = WYCAS_IDLE;
static unsigned long wyCasStartStep      = 0UL;
static bool          wyCasDcw2Started  = false;
static bool          wyCasDcw2Stopped  = false;
static bool          wyCasDcs2Started  = false;
static bool          wyCasDcs2Stopped  = false;

// =============================================================================
// STAIN PROFILE — runtime state + helpers
// (Struct, enum, and PROFILES[] table live in STAIN-PARAMETERS.h)
// =============================================================================
static ProfileId activeProfileId = PROFILE_RP;
static inline const StainProfile& profile() { return PROFILES[activeProfileId]; }

// Lookup by code string ("RP"/"LM"/"MG"/"WG"). Returns PROFILE_COUNT if not found.
static ProfileId lookupProfileByCode(const char* code) {
    for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
        if (strcmp(code, PROFILES[i].code) == 0) return (ProfileId)i;
    }
    return PROFILE_COUNT;
}

// True if any master cascade is mid-flight — used to reject profile changes mid-run.
static bool anyCascadeActive() {
    return masterBedWindowsActive() || m15MixState != M15MIX_IDLE
        || sxCasState  != SXCAS_IDLE
        || s2MixState  != S2MIX_IDLE
        || wyCasState  != WYCAS_IDLE;
}

static void applyReagentDispenseLock(bool locked) {
    if (reagentDispenseLocked == locked) return;
    reagentDispenseLocked = locked;
    if (locked) {
        M14Motor.stop();  // ethanol
        ZMotor.stop();    // stain 1
        M15Motor.stop();  // buffer 1
        XMotor.stop();    // stain 2
        YMotor.stop();    // buffer 2
        M18Motor.stop();  // reagent mixer/dispatch
        mixdc1.stopMotor();
        mixdc2.stopMotor();
        dcFan2.stopFan();
        m15MixState = M15MIX_IDLE;
        s2MixState = S2MIX_IDLE;
        m18OscState = M18_OSC_IDLE;
    }
    Serial.print("[REAGENTLOCK] reagent_dispense=");
    Serial.print(locked ? "BLOCKED" : "ENABLED");
    Serial.println(" transport=ENABLED wash=ENABLED suction=ENABLED drain=ENABLED");
}

// Forward decl — defined further down (PiSerial / NozzleSerial declared later).
static void sendSaveProfileToEsp32(const StainProfile& p);
static void pushProfileToNozzle(const StainProfile& p);

// Print a one-line summary of a profile's contents.
static void printProfileSummary(const StainProfile& p) {
    Serial.print("[PROFILE] active="); Serial.print(p.code);
    Serial.print(" ("); Serial.print(p.description); Serial.print(")");
    Serial.print(" E=");  if (p.e_enabled)  { Serial.print(p.e_uL);  Serial.print("uL"); } else Serial.print("off");
    Serial.print(" S1="); if (p.s1_enabled) { Serial.print(p.s1_uL); Serial.print("uL"); } else Serial.print("off");
    Serial.print(" B1="); if (p.b1_enabled) { Serial.print(p.b1_uL); Serial.print("uL"); } else Serial.print("off");
    Serial.print(" S2="); if (p.s2_enabled) { Serial.print(p.s2_uL); Serial.print("uL"); } else Serial.print("off");
    Serial.print(" B2="); if (p.b2_enabled) { Serial.print(p.b2_uL); Serial.print("uL"); } else Serial.print("off");
    Serial.println();
}

// Last pipeline depth reported by nozzle via "NZQ <count>". Used by
// applyProfile() to refuse profile changes while slides are still in flight
// on the nozzle (even when master cascades are idle and FEED is stopped).
// Updated by serviceNozzleSerial() in the NZQ parser branch.
static int lastNozzleQueueDepth = 0;
static bool nozzleProductionBusy = false;
static bool gantryProductionBusy = false;

// Apply a profile by ID. Returns true on success, false if rejected.
// Rejection cases:
//   - invalid profile ID
//   - FEED is currently active (slides on the belt being processed)
//   - any master cascade is mid-flight (B1MIX / S2MIX / SXCAS / WYCAS)
//   - any slide in the nozzle pipeline (NZQ > 0 last reported)
// Persists the new selection to ESP32 NVS on success (so it survives reboot)
// and pushes the new profile to the nozzle.
static bool applyProfile(ProfileId newId) {
    if (newId >= PROFILE_COUNT) {
        Serial.println("[PROFILE] reject: invalid ID");
        return false;
    }
    if (tFeedActive) {
        Serial.print("[PROFILE] reject: FEED is active — stop feed before switching profile. Current=");
        Serial.println(profile().code);
        return false;
    }
    if (lastNozzleQueueDepth > 0) {
        Serial.print("[PROFILE] reject: nozzle pipeline has ");
        Serial.print(lastNozzleQueueDepth);
        Serial.print(" slide(s) in flight — wait for NZQ=0. Current=");
        Serial.println(profile().code);
        return false;
    }
    if (anyCascadeActive()) {
        Serial.print("[PROFILE] reject: cascade active (mid-run change blocked). Current=");
        Serial.println(profile().code);
        return false;
    }
    if (newId == activeProfileId) {
        // Treat a repeated command as a synchronization request. The nozzle
        // can reboot independently, and Check Systems always needs its active
        // recipe confirmed before NZHO applies profile-specific park positions.
        Serial.print("[PROFILE] resync (already "); Serial.print(profile().code); Serial.println(")");
        pushProfileToNozzle(profile());
        return true;
    }
    activeProfileId = newId;
    printProfileSummary(profile());
    sendSaveProfileToEsp32(profile());
    pushProfileToNozzle(profile());
    return true;
}

// UART to Pi/ESP32 (PB10=TX, PB11=RX) — accept commands and mirror responses
LivoHardwareSerial PiSerial(Pi_RX, Pi_TX);
MirrorStream   mirroredSerial(Serial, PiSerial);
// Local mirror redirect — every Serial.print() in this file goes to both USB and PiSerial.
#define Serial mirroredSerial

// Persist active profile to ESP32 NVS. ESP32 firmware writes Preferences key
// on receiving this line; the call is fire-and-forget (no ACK awaited).
// On reboot, master sends GET_PROFILE\n and ESP32 replies with "PROFILE <code>"
// which the standard master command parser picks up.
static void sendSaveProfileToEsp32(const StainProfile& p) {
    PiSerial.print("SAVE_PROFILE ");
    PiSerial.println(p.code);
}

// UART to Nozzle Mount PCB (PC10=TX, PC11=RX)
static LivoHardwareSerial NozzleSerial(NM_RX, NM_TX);
static bool           nozzleReady   = false;   // set true when "Nozzle: Ready" received
static String         nozzleRxBuf   = "";      // line buffer for NozzleSerial

// Push the active profile to the nozzle so its per-recipe delays stay in sync.
// Sent over NozzleSerial as "PROFILE <code>" — same line the nozzle parser
// already recognises. Fire-and-forget; nozzle just updates its nzActiveProfileId.
static void pushProfileToNozzle(const StainProfile& p) {
    NozzleSerial.print("PROFILE ");
    NozzleSerial.println(p.code);
}

// UART to Gantry PCB (G_TX=PC6, G_RX=PC7)
static LivoHardwareSerial GantrySerial(G_RX, G_TX);
static bool           gantryReady   = false;   // set true when "Gantry: Ready" received
static String         gantryRxBuf   = "";      // line buffer for GantrySerial

// Forward decl — dispatcher is defined further down (after stmFlash).
static void dispatchMasterCommandLine(const String& rawLine, const char* sourceTag);
static ParaCheckTrace paraCheck;
static bool pcWaitingForNozzle = false;
static bool pcConfigurationPending = false;
static bool pcProfileSelected = false, pcFeedSelected = false;
static bool pcEthanolEnded = false, pcStain1Ended = false;
static void pcEvent(TimingKnob k, const char* state = "FIRED") {
    paraCheck.event(Serial, k, TMotor.getCurrentPosition(), state);
}
static void pcNote(const char* label) {
    paraCheck.note(Serial, label, TMotor.getCurrentPosition());
}
static void pcFinish(const char* reason) {
    if (pcConfigurationPending) {
        Serial.print("[PARA] END "); Serial.println(reason);
    }
    pcConfigurationPending = false;
    paraCheck.finish(Serial, reason);
    pcWaitingForNozzle = false;
    NozzleSerial.println("PARA STOP");
}
// PARA CHECK only opens configuration. Start after the operator has explicitly
// selected both PROFILE and FEED, in either order, then await the nozzle ACK.
static void pcTryStart() {
    if (!pcConfigurationPending || !pcProfileSelected || !pcFeedSelected) return;
    pcConfigurationPending = false;
    paraCheck.begin(activeProfileId, TMotor.getCurrentPosition());
    pcEthanolEnded = pcStain1Ended = false;
    pcWaitingForNozzle = true;
    // PROFILE selection already synchronized the nozzle. Re-sending it here
    // can re-park axes during an existing slide's motion.
    NozzleSerial.print("FEED "); NozzleSerial.println(currentSph);
    NozzleSerial.println("PARA CHECK");
    Serial.print("[PARA] ARMED PROFILE:"); Serial.print(profile().code);
    Serial.print(" FEED:"); Serial.println(currentSph);
    Serial.println("[PARA] One slide; counter starts at IR4 UART receipt; event lines print only when fired");
}
static void pcNozzleEvent(const String& event) {
    if (!paraCheck.active) return;
    const long pos = TMotor.getCurrentPosition();
    if (event == "ARMED" && pcWaitingForNozzle) {
        pcWaitingForNozzle = false;
        tFeedSpeed = (currentSph * T_MOTOR_STEPS_PER_REV) / 3600L;
        if(!masterBedStart()){pcFinish("INVALID_RECIPE_OR_BED_NOT_READY");return;}
        TMotor.motor.setMaxSpeed(tFeedSpeed);
        if (!tFeedActive) TMotor.move(-2000000000L);
        tFeedActive = true;
        Serial.println("[PARA] nozzle armed; bed running; waiting for next IR4 clear then trigger");
    } else if (event == "REJECT_BUSY") {
        pcFinish("NOZZLE_BUSY_BED_NOT_STARTED");
    } else if (event == "IR4") {
        if (paraCheck.anchored) { pcNote("DUPLICATE_IR4"); return; }
        paraCheck.origin = pos;
        paraCheck.anchored = true;
        Serial.println("[PARA] IR4 CURRENT:0 PARAMETER:0 SOURCE:UART_RECEIPT");
    } else if (event == "ETH_ON") pcEvent(TK_ETH_ON, "NOZZLE_UART_RECEIPT");
    else if (event == "ETH_OFF") pcEvent(TK_ETH_OFF, "NOZZLE_UART_RECEIPT");
    else if (event == "SY") pcEvent(TK_SY_AT, "NOZZLE_MOVE_COMMAND_UART_RECEIPT");
    else if (event == "SX") pcEvent(TK_SX_AT, "NOZZLE_MOVE_COMMAND_UART_RECEIPT");
    else if (event == "WY") pcEvent(TK_WY_AT, "NOZZLE_MOVE_COMMAND_UART_RECEIPT");
    else if (event == "WX") pcEvent(TK_WX_AT, "NOZZLE_MOVE_COMMAND_UART_RECEIPT");
    else if (event == "WX_FAN_ON") {
        pcEvent(TK_DRY_ON, "NOZZLE_UART_RECEIPT");
    } else if (event == "WX_FAN_OFF") {
        pcEvent(TK_DRY_OFF, "NOZZLE_UART_RECEIPT");
    } else if (event == "FLOW_IDLE") {
        pcNote("NOZZLE_FLOW_IDLE"); paraCheck.fanDone = true;
    } else pcNote(event.c_str());
}

static void serviceParaCheck() {
    if (!paraCheck.active) return;
    if (pcWaitingForNozzle) {
        if (millis() - paraCheck.startedMs > 5000UL) pcFinish("NO_NOZZLE_ACK_EXISTING_BED_MOTION_UNCHANGED");
        return;
    }
    if (paraCheck.anchored && paraCheck.actual[TK_M14_AT] >= 0 &&
        !pcEthanolEnded && !M14Motor.isMoving()) {
        pcEthanolEnded = true; pcNote("M14_DISPENSE_END");
    }
    if (paraCheck.anchored && paraCheck.actual[TK_S1_AT] >= 0 &&
        !pcStain1Ended && !ZMotor.isMoving()) {
        pcStain1Ended = true; pcNote("S1_DISPENSE_END");
    }
    if (paraCheck.anchored && paraCheck.actual[TK_B1_AT] >= 0 &&
        paraCheck.b1End < 0 && !M15Motor.isMoving()) {
        paraCheck.b1End = paraCheck.current(TMotor.getCurrentPosition());
        pcNote("B1_DISPENSE_END");
    }
    if (!tFeedActive) { pcFinish("FEED_STOPPED"); return; }
    if (paraCheck.fanDone && lastNozzleQueueDepth == 0 && !anyCascadeActive() &&
        !M14Motor.isMoving() && !M15Motor.isMoving() && !XMotor.isMoving() &&
        !YMotor.isMoving() && !ZMotor.isMoving() && !M18Motor.isMoving()) {
        pcFinish("FLOW_COMPLETE_BED_CONTINUES_USE_FEEDSTOP");
    } else if (millis() - paraCheck.startedMs > 1800000UL) {
        pcFinish("TIMEOUT_INCOMPLETE_BED_CONTINUES_USE_FEEDSTOP");
    }
}

static bool handleReagentLockControl(const String& rawCommand, const char* sourceTag) {
    String command = rawCommand;
    command.trim();
    command.toUpperCase();
    if (!command.startsWith("REAGENTLOCK")) return false;
    // Only the ESP32 balance authority or the physical USB service port may
    // change this interlock. Requests originating from subordinate boards fail closed.
    const bool authorizedSource = sourceTag && (sourceTag[0] == 'P' || sourceTag[0] == 'U');
    if (!authorizedSource) {
        Serial.println("[REAGENTLOCK] rejected unauthorized source");
        return true;
    }
    if (command == "REAGENTLOCK 1") applyReagentDispenseLock(true);
    else if (command == "REAGENTLOCK 0") applyReagentDispenseLock(false);
    else Serial.println("[REAGENTLOCK] usage: REAGENTLOCK 0|1");
    return true;
}

// Arm the SX cascade at the current acknowledged bed coordinate.
static void armSxCascade() {
    Serial.println("Cascade command retired: use independent IR4-relative ON/OFF positions");
}

// Service the SX cascade — called from the master loop each cycle.
// Each event fires exactly once per cascade. Now handles only DCW1 and DCS1;
// the pump+fan phase is in S2MIX, scheduled separately by the nozzle.


// Start B1 dispense. Air mixing has independent absolute IR4 ON/OFF events.
static void armB1Mixcade() {
    if (m15MixState != M15MIX_IDLE) {
        SensorFault::report("LIVO-SEQ-002", "B1MIX", 0, "cascade_busy_existing_timing_preserved");
        return;
    }
    if (reagentDispenseLocked) {
        Serial.println("[B1MIX] BLOCKED by REAGENTLOCK; transport continues");
        return;
    }
    if (!profile().b1_enabled) {
        Serial.print("[B1MIX] skipped — B1 disabled in profile "); Serial.println(profile().code);
        return;
    }
    const long  uL    = profile().b1_uL;
    const float ml    = (float)uL / 1000.0f;
    const long  steps = (long)((ml * (float)M15_STEPS_PER_REV) / M15_FLOW_ML_PER_REV
                               + (ml >= 0 ? 0.5f : -0.5f));
    M15Motor.setMaxSpeed(M15_DISPENSE_SPEED);
    M15Motor.setAcceleration(M15_DISPENSE_ACCEL);
    M15Motor.move(steps);
    pcEvent(TK_B1_AT);
    ackManager.requestMovementAck(&M15Motor, "M15", micros());
    m15MixArmStep = masterBedPosition;
    m15MixState = M15MIX_WAIT_FINISH;   // serviceM15MixCascade picks it up
    Serial.print("[B1MIX] profile="); Serial.print(profile().code);
    Serial.print(" armed  M15="); Serial.print(steps); Serial.print(" steps  vol=");
    Serial.print(ml, 3); Serial.println(" mL");
}

static void setPWMDuty(unsigned long pwmFrequencyHz) {
    analogWriteResolution(8);
    analogWriteFrequency(pwmFrequencyHz);  // ensure 8-bit resolution and correct frequency for MIXD/MIXR
}

// Start S2/B2 pumps and M18 oscillation at s2mix_at.
// MasterBedProduction dispatches at the independent s2disp_at position.
static void armS2Mixcade() {
    if (s2MixState != S2MIX_IDLE) {
        SensorFault::report("LIVO-SEQ-002", "S2MIX", 0, "cascade_busy_existing_timing_preserved");
        return;
    }
    if (reagentDispenseLocked) {
        Serial.println("[S2MIX] BLOCKED by REAGENTLOCK; transport continues");
        return;
    }
    // Profile-gated skip: if neither S2 nor B2 is enabled, skip the whole cascade.
    if (!profile().s2_enabled && !profile().b2_enabled) {
        Serial.print("[S2MIX] skipped — S2 and B2 both disabled in profile ");
        Serial.println(profile().code);
        return;
    }

    Serial.println("[BUILD] 2026-09-07-s2b2-mg1to8-wg1to9");  // MG=140:1120 uL; WG=126:1134 uL
    s2MixState           = S2MIX_ACTIVE;
    pcEvent(TK_S2MIX_AT);
    s2MixStartStep         = masterBedPosition;
    s2MixPumpsEnded      = false;
    s2MixPumpsEndStep      = 0UL;
    s2MixComplete        = false;
    s2MixDispatchStarted = false;
    s2MixDispatchStartStep = 0UL;

    // X (S2) dispense — fires immediately
    long xSteps = 0;
    if (profile().s2_enabled) {
        const long  uL    = profile().s2_uL;
        const float xMl   = (float)uL / 1000.0f;
        xSteps = (long)((xMl * (float)X_STEPS_PER_REV) / X_FLOW_ML_PER_REV
                        + (xMl >= 0 ? 0.5f : -0.5f));
        XMotor.setMaxSpeed(X_DISPENSE_SPEED);
        XMotor.setAcceleration(X_DISPENSE_ACCEL);
        XMotor.move(xSteps);
        ackManager.requestMovementAck(&XMotor, "X", micros());
    }

    // Y (B2) dispense — fires immediately
    long ySteps = 0;
    if (profile().b2_enabled) {
        const long  uL    = profile().b2_uL;
        const float yMl   = (float)uL / 1000.0f;
        ySteps = (long)((yMl * (float)Y_STEPS_PER_REV) / Y_FLOW_ML_PER_REV
                        + (yMl >= 0 ? 0.5f : -0.5f));
        YMotor.setMaxSpeed(Y_DISPENSE_SPEED);
        YMotor.setAcceleration(Y_DISPENSE_ACCEL);
        YMotor.move(ySteps);
        ackManager.requestMovementAck(&YMotor, "Y", micros());
    }

    // M18 stepper: OSCILLATION starts at t=0 alongside X+Y pumps.
    M18Motor.setRMSCurrentIRUN(MIX_RMS_CURRENT);
    M18Motor.setAcceleration(MIX_ACCEL);
    M18Motor.setMaxSpeed(MIX_OSC_SPEED);
    M18Motor.move(MIX_OSC_AMPLITUDE_STEPS);
    m18OscState = M18_OSC_FORWARD;
    // Acknowledge only final dispatch completion, not the first mix half-cycle.

    Serial.print("[S2MIX] profile="); Serial.print(profile().code);
    Serial.print(" armed  X=");        Serial.print(xSteps);
    Serial.print(" steps  Y=");        Serial.print(ySteps);
    Serial.print(" steps  M18 osc=±"); Serial.print(MIX_OSC_AMPLITUDE_STEPS);
    Serial.print("steps @ ");          Serial.print(MIX_OSC_SPEED);
    Serial.print(" sps Irun=");        Serial.print(MIX_RMS_CURRENT);
    Serial.println("mA");
}




// Service motor completion only; this function never schedules an operation.
static void serviceS2Mixcade() {
    if(!tFeedActive || masterBedFault || s2MixState!=S2MIX_ACTIVE)return;
    if(m18OscState!=M18_OSC_IDLE && !M18Motor.isMoving()) {
        const bool forward=m18OscState==M18_OSC_REVERSE;
        m18OscState=forward?M18_OSC_FORWARD:M18_OSC_REVERSE;
        M18Motor.move(forward?MIX_OSC_AMPLITUDE_STEPS:-MIX_OSC_AMPLITUDE_STEPS);
    }
    // Pump completion is a readiness check, never a new scheduling anchor.
    if(!s2MixPumpsEnded && !XMotor.isMoving() && !YMotor.isMoving()) {
        s2MixPumpsEnded=true;pcNote("S2_PUMPS_FINISHED_WAITING_ABSOLUTE_DISPATCH");
    }
    if(s2MixDispatchStarted && !M18Motor.isMoving()) {
        s2MixComplete=true;s2MixState=S2MIX_IDLE;pcNote("S2_DISPATCH_COMPLETE");
    }
}

// =============================================================================
// MLOAD — prime each liquid line by running its motor until the corresponding
// IR sensor detects liquid (analog value drops below MLOAD_IR_THRESHOLD).
// IR pin → motor mapping (per StainerMasterPCBV1.h):
//   IR1 (PA4) → M14 (Ethanol pump E)
//   IR2 (PA5) → Z   (Stain pump S1)
//   IR3 (PA6) → M15 (Buffer pump B1)
//   IR4 (PA7) → X   (Stain pump S2)
//   IR5 (PC4) → Y   (Buffer pump B2)
//   IR6 (PC5) → DCW1 (Wash 1)
//   IR7 (PF3) → DCW2 (Wash 2)
// Blocking. All motors run in parallel; each stops the moment its own IR drops.
// =============================================================================
constexpr int           MLOAD_IR_THRESHOLD = 100;
constexpr unsigned long MLOAD_TIMEOUT_MS   = 60000UL;
constexpr long          MLOAD_STEPPER_TARGET = 2000000000L;   // effectively forever
constexpr uint8_t       MLOAD_DCW_DUTY     = 255;

static void runMLoadBlocking() {
    Serial.println("MLOAD: priming all liquid lines");

    // Start all steppers (+ve direction) and DC wash motors
    M14Motor.move(MLOAD_STEPPER_TARGET);
    ZMotor.move(MLOAD_STEPPER_TARGET);
    M15Motor.move(MLOAD_STEPPER_TARGET);
    XMotor.move(MLOAD_STEPPER_TARGET);
    YMotor.move(MLOAD_STEPPER_TARGET);
    washdc1.runMotor(MLOAD_DCW_DUTY);
    washdc2.runMotor(MLOAD_DCW_DUTY);

    bool m14Done = false, zDone = false, m15Done = false;
    bool xDone   = false, yDone = false;
    bool dcw1Done = false, dcw2Done = false;

    const unsigned long start = millis();
    while (!(m14Done && zDone && m15Done && xDone && yDone && dcw1Done && dcw2Done)) {
        if (!m14Done) M14Motor.loop();
        if (!zDone)   ZMotor.loop();
        if (!m15Done) M15Motor.loop();
        if (!xDone)   XMotor.loop();
        if (!yDone)   YMotor.loop();

        limitSwitch.loop();
        int* ir = limitSwitch.getSmoothedSensorValues();
        if (ir != nullptr) {
            if (!m14Done && SensorFaultPolicy::adcValid(ir[0]) && ir[0] < MLOAD_IR_THRESHOLD) {
                M14Motor.stop(); while (M14Motor.isMoving()) M14Motor.loop();
                m14Done = true; Serial.println("MLOAD: E (M14) primed");
            }
            if (!zDone && SensorFaultPolicy::adcValid(ir[1]) && ir[1] < MLOAD_IR_THRESHOLD) {
                ZMotor.stop(); while (ZMotor.isMoving()) ZMotor.loop();
                zDone = true; Serial.println("MLOAD: S1 (Z) primed");
            }
            if (!m15Done && SensorFaultPolicy::adcValid(ir[2]) && ir[2] < MLOAD_IR_THRESHOLD) {
                M15Motor.stop(); while (M15Motor.isMoving()) M15Motor.loop();
                m15Done = true; Serial.println("MLOAD: B1 (M15) primed");
            }
            if (!xDone && SensorFaultPolicy::adcValid(ir[3]) && ir[3] < MLOAD_IR_THRESHOLD) {
                XMotor.stop(); while (XMotor.isMoving()) XMotor.loop();
                xDone = true; Serial.println("MLOAD: S2 (X) primed");
            }
            if (!yDone && SensorFaultPolicy::adcValid(ir[4]) && ir[4] < MLOAD_IR_THRESHOLD) {
                YMotor.stop(); while (YMotor.isMoving()) YMotor.loop();
                yDone = true; Serial.println("MLOAD: B2 (Y) primed");
            }
            if (!dcw1Done && SensorFaultPolicy::adcValid(ir[5]) && ir[5] < MLOAD_IR_THRESHOLD) {
                washdc1.stopMotor();
                dcw1Done = true; Serial.println("MLOAD: W1 (DCW1) primed");
            }
            if (!dcw2Done && SensorFaultPolicy::adcValid(ir[6]) && ir[6] < MLOAD_IR_THRESHOLD) {
                washdc2.stopMotor();
                dcw2Done = true; Serial.println("MLOAD: W2 (DCW2) primed");
            }
        }

        if (millis() - start > MLOAD_TIMEOUT_MS) {
            if (!m14Done) { M14Motor.stop(); while (M14Motor.isMoving()) M14Motor.loop(); }
            if (!zDone)   { ZMotor.stop();   while (ZMotor.isMoving())   ZMotor.loop(); }
            if (!m15Done) { M15Motor.stop(); while (M15Motor.isMoving()) M15Motor.loop(); }
            if (!xDone)   { XMotor.stop();   while (XMotor.isMoving())   XMotor.loop(); }
            if (!yDone)   { YMotor.stop();   while (YMotor.isMoving())   YMotor.loop(); }
            if (!dcw1Done) washdc1.stopMotor();
            if (!dcw2Done) washdc2.stopMotor();
            const bool pending[]={!m14Done,!zDone,!m15Done,!xDone,!yDone,!dcw1Done,!dcw2Done};
            for(int channel=0;channel<7;++channel) if(pending[channel]) {
                char sensor[12]; snprintf(sensor,sizeof(sensor),"LIQ-IR%d",channel+1);
                SensorFault::report("LIVO-FLD-002",sensor,ir?ir[channel]:-1,"prime_not_confirmed_sensor_pump_or_liquid_unknown");
            }
            Serial.print("MLOAD: timeout — pending:");
            if (!m14Done)  Serial.print(" E");
            if (!zDone)    Serial.print(" S1");
            if (!m15Done)  Serial.print(" B1");
            if (!xDone)    Serial.print(" S2");
            if (!yDone)    Serial.print(" B2");
            if (!dcw1Done) Serial.print(" W1");
            if (!dcw2Done) Serial.print(" W2");
            Serial.println();
            return;
        }
    }
    Serial.println("MLOAD: all lines primed");
}

// =============================================================================
// DRAIN — start DCD at full duty, wait 3 s, then monitor IR8.
// Stop DCD as soon as IR8 RISES ABOVE DRAIN_IR8_THRESHOLD.
// Non-blocking. Safety timeout DRAIN_TIMEOUT_MS forces stop if IR8 never trips.
// =============================================================================
constexpr int           DRAIN_IR8_INDEX     = 7;           // irsensorPins[7] = IR8 (PF6)
constexpr int           DRAIN_IR8_THRESHOLD = 100;
constexpr unsigned long DRAIN_PRE_DELAY_MS  = 5000UL;
constexpr unsigned long DRAIN_TIMEOUT_MS    = 60000UL;

enum DrainState { DRAIN_IDLE = 0, DRAIN_PRE_DELAY, DRAIN_MONITORING };
static DrainState    drainState   = DRAIN_IDLE;
static unsigned long drainStartMs = 0UL;

static void armDrain() {
    digitalWrite(D_Motor, HIGH);              // DCD plain GPIO ON (PE11 must stay off TIM1)
    drainState   = DRAIN_PRE_DELAY;
    drainStartMs = millis();
    Serial.print("[DRAIN] DCD ON (digital), monitoring IR8 in ");
    Serial.print(DRAIN_PRE_DELAY_MS); Serial.println("ms");
}

static void serviceDrain() {
    if (drainState == DRAIN_IDLE) return;
    const unsigned long now     = millis();
    const unsigned long elapsed = now - drainStartMs;

    if (drainState == DRAIN_PRE_DELAY) {
        if (elapsed >= DRAIN_PRE_DELAY_MS) {
            drainState = DRAIN_MONITORING;
            Serial.print("[DRAIN] now monitoring IR8 (threshold="); Serial.print(DRAIN_IR8_THRESHOLD);
            Serial.println(")");
        }
        return;
    }

    // DRAIN_MONITORING — poll IR8 and stop when it rises above threshold
    limitSwitch.loop();
    int* ir = limitSwitch.getSmoothedSensorValues();
    if (ir != nullptr && ir[DRAIN_IR8_INDEX] > DRAIN_IR8_THRESHOLD) {
        digitalWrite(D_Motor, LOW);
        drainState = DRAIN_IDLE;
        Serial.print("[DRAIN] IR8="); Serial.print(ir[DRAIN_IR8_INDEX]);
        Serial.print(" > threshold, DCD OFF @ "); Serial.print(elapsed);
        Serial.println("ms");
        return;
    }

    if (elapsed > DRAIN_TIMEOUT_MS) {
        digitalWrite(D_Motor, LOW);
        drainState = DRAIN_IDLE;
        SensorFault::report("LIVO-FLD-003","DRAIN-IR8",ir?ir[DRAIN_IR8_INDEX]:-1,"drain_not_confirmed_cause_unconfirmed");
        Serial.print("[DRAIN] timeout @ "); Serial.print(elapsed);
        Serial.println("ms, DCD OFF");
    }
}

// =============================================================================
// MUNLOAD — reverse-purge all liquid lines, then drain.
//   t=0:        All pumps (M14, Z, M15, X, Y) move MUNLOAD_PUMP_STEPS (-ve).
//               DCW1 and DCW2 ON.
//   t=DCW_DUR:  DCW1 OFF, DCW2 OFF, DCD ON.
//   t=DCW_DUR+DCD_DUR: DCD OFF.
//   Command completes when DCD is off AND all stepper pumps have stopped moving.
// Non-blocking — driven by serviceMUnloadCascade() from the master loop.
// =============================================================================
constexpr long          MUNLOAD_PUMP_STEPS       = -1500000L;   // E (M14), B1 (M15), B2 (Y)
constexpr long          MUNLOAD_STAIN_STEPS      = -100000L;    // S1 (Z), S2 (X) — shorter purge
constexpr unsigned long MUNLOAD_DCW_DURATION_MS  = 5000UL;
constexpr unsigned long MUNLOAD_DCD_DURATION_MS  = 10000UL;

enum MUnloadState { MUNLOAD_IDLE = 0, MUNLOAD_ACTIVE };
static MUnloadState  mUnloadState      = MUNLOAD_IDLE;
static unsigned long mUnloadStartMs    = 0UL;
static bool          mUnloadDcwStopped = false;
static bool          mUnloadDcdStarted = false;
static bool          mUnloadDcdStopped = false;
static bool          mUnloadDone       = false;

static void armMUnload() {
    Serial.print("[MUNLOAD] start  E/B1/B2="); Serial.print(MUNLOAD_PUMP_STEPS);
    Serial.print(" S1/S2=");                   Serial.print(MUNLOAD_STAIN_STEPS);
    Serial.print(" DCW=");                     Serial.print(MUNLOAD_DCW_DURATION_MS);
    Serial.print("ms DCD=");                   Serial.print(MUNLOAD_DCD_DURATION_MS);
    Serial.println("ms");

    // Reset flags
    mUnloadState      = MUNLOAD_ACTIVE;
    mUnloadStartMs    = millis();
    mUnloadDcwStopped = false;
    mUnloadDcdStarted = false;
    mUnloadDcdStopped = false;
    mUnloadDone       = false;

    // Kick off all stepper pumps in reverse, plus DCW1 + DCW2
    M14Motor.move(MUNLOAD_PUMP_STEPS);   // E
    ZMotor.move(MUNLOAD_STAIN_STEPS);    // S1 — shorter
    M15Motor.move(MUNLOAD_PUMP_STEPS);   // B1
    XMotor.move(MUNLOAD_STAIN_STEPS);    // S2 — shorter
    YMotor.move(MUNLOAD_PUMP_STEPS);     // B2
    washdc1.runMotor(DCW1_DUTY);
    washdc2.runMotor(DCW2_DUTY);
}

static void serviceMUnloadCascade() {
    if (mUnloadState != MUNLOAD_ACTIVE) return;
    const unsigned long now     = millis();
    const unsigned long elapsed = now - mUnloadStartMs;

    // At DCW_DURATION: stop DCW1+DCW2, start DCD (digital — PE11 must stay off TIM1)
    if (!mUnloadDcwStopped && elapsed >= MUNLOAD_DCW_DURATION_MS) {
        washdc1.stopMotor();
        washdc2.stopMotor();
        digitalWrite(D_Motor, HIGH);
        mUnloadDcwStopped = true;
        mUnloadDcdStarted = true;
        Serial.print("[MUNLOAD] DCW1+DCW2 OFF, DCD ON @ ");
        Serial.print(elapsed); Serial.println("ms");
    }

    // At DCW_DURATION + DCD_DURATION: stop DCD
    if (!mUnloadDcdStopped && elapsed >= (MUNLOAD_DCW_DURATION_MS + MUNLOAD_DCD_DURATION_MS)) {
        digitalWrite(D_Motor, LOW);
        mUnloadDcdStopped = true;
        Serial.print("[MUNLOAD] DCD OFF @ "); Serial.print(elapsed); Serial.println("ms");
    }

    // Cascade complete only when DCD is off AND all pumps have stopped moving
    if (mUnloadDcdStopped && !mUnloadDone &&
        !M14Motor.isMoving() && !ZMotor.isMoving() && !M15Motor.isMoving() &&
        !XMotor.isMoving()   && !YMotor.isMoving()) {
        mUnloadDone  = true;
        mUnloadState = MUNLOAD_IDLE;
        Serial.print("[MUNLOAD] complete @ "); Serial.print(elapsed); Serial.println("ms");
    }
}

// Arm the WY cascade at the current acknowledged bed coordinate.
static void armWyCascade() {
    Serial.println("Cascade command retired: use independent IR4-relative ON/OFF positions");
}

// Service the WY cascade — DCW2 then DCS2, mirrors the SXCAS DCW1/DCS1 phases.
// DCW2 is PWM via TIM1_CH1 (PE9); DCS2 uses suctiondc2.runMotor (PG10 has no
// PWM timer so analogWrite falls back to digital anyway).


// Service the post-B1 mix cascade. Call from the master loop every cycle.
static void serviceM15MixCascade() {
    // B1 pump completion is independent of the air pump's ON/OFF positions.
    if(m15MixState!=M15MIX_IDLE && !M15Motor.isMoving())m15MixState=M15MIX_IDLE;
}

// Call from loop() — reads NozzleSerial, echoes lines to USB with [N] tag,
// tracks boot ack, and forwards "REQ:<command>" lines through the master command
// dispatcher so the nozzle can ask master to do things (e.g. profile-volume dispense).
#include "MasterBedProduction.inc"

static void serviceNozzleSerial() {
    static bool nozzleRxBufDiscarding = false;
    if (livoCommunication.getinputBytesAvailable()) return;
    unsigned byteBudget = 128;
    while (byteBudget-- && NozzleSerial.available()) {
        char c = (char)NozzleSerial.read();
        if (receiveUartLineByte(c, nozzleRxBuf, nozzleRxBufDiscarding)) {
            nozzleRxBuf.trim();
            if (nozzleRxBuf.length() > 0) {
                if(masterBedInput(nozzleRxBuf)){nozzleRxBuf="";continue;}
                if (nozzleRxBuf == "RUNBUSY 0" || nozzleRxBuf == "RUNBUSY 1") {
                    nozzleProductionBusy = nozzleRxBuf.endsWith("1");
                    nozzleRxBuf = ""; continue;
                }
                if (nozzleRxBuf.startsWith("PCE ")) {
                    pcNozzleEvent(nozzleRxBuf.substring(4));
                    nozzleRxBuf = "";
                    continue;
                }
                if (nozzleRxBuf.equals("Nozzle: Ready")) {
                    if (paraCheck.active) pcFinish("NOZZLE_RESTART_INCOMPLETE");
                    if(masterBedEstablished)masterBedFail(86);
                    nozzleReady = true;
                    // Sync nozzle to the active stain profile so its per-recipe
                    // delays match. Sent every time the nozzle reports ready
                    // (handles nozzle resets / late join after master is up).
                    pushProfileToNozzle(profile());
                } else if (nozzleRxBuf.equals("Nozzle: Failed")) {
                    nozzleReady = false;
                } else if (nozzleRxBuf.startsWith("NZQ ")) {
                    // Pipeline depth report from nozzle — gates PROFILE changes.
                    String tail = nozzleRxBuf.substring(4); tail.trim();
                    long d = tail.toInt();
                    if (d < 0) d = 0;
                    lastNozzleQueueDepth = (int)d;
                    Serial.print("[N] NZQ "); Serial.println(lastNozzleQueueDepth);
                    nozzleRxBuf = "";
                    continue;
                } else if (nozzleRxBuf.startsWith("REQ:")) {
                    String cmd = nozzleRxBuf.substring(4); cmd.trim();
                    Serial.print("[N] "); Serial.println(nozzleRxBuf);
                    dispatchMasterCommandLine(cmd, "N");
                    nozzleRxBuf = "";
                    if (livoCommunication.getinputBytesAvailable()) return;
                    continue;
                }
                if(nozzleRxBuf.startsWith("UITR ") || nozzleRxBuf.startsWith("DROW ") || nozzleRxBuf.startsWith("DDONE ") || nozzleRxBuf.startsWith("DLOCKED ")) {
                    PiSerial.println(nozzleRxBuf); // Machine telemetry bypasses slow USB mirroring.
                } else { Serial.print("[N] "); Serial.println(nozzleRxBuf); }
            }
            nozzleRxBuf = "";
        }
    }
}

// Call from loop() — reads GantrySerial, echoes lines to USB with [G] tag, tracks boot ack
static void serviceGantrySerial() {
    static bool gantryRxBufDiscarding = false;
    unsigned byteBudget = 128;
    while (byteBudget-- && GantrySerial.available()) {
        char c = (char)GantrySerial.read();
        if (receiveUartLineByte(c, gantryRxBuf, gantryRxBufDiscarding)) {
            gantryRxBuf.trim();
            if (gantryRxBuf.length() > 0) {
                if (gantryRxBuf == "RUNBUSY 0" || gantryRxBuf == "RUNBUSY 1") {
                    gantryProductionBusy = gantryRxBuf.endsWith("1");
                    gantryRxBuf = ""; continue;
                }
                if (gantryRxBuf.equals("Gantry: Ready")) {
                    gantryReady = true;
                } else if (gantryRxBuf.equals("Gantry: Failed")) {
                    gantryReady = false;
                }
                if (gantryRxBuf.startsWith("DROW ") || gantryRxBuf.startsWith("DDONE ") || gantryRxBuf.startsWith("DLOCKED ")) {
                    PiSerial.println(gantryRxBuf);
                } else if (gantryRxBuf.startsWith("MAGSTAT ")) {
                    // Routine live telemetry is forwarded to the ESP32 without
                    // mirroring it to the Master's USB diagnostic console.
                    PiSerial.print("[G] ");
                    PiSerial.println(gantryRxBuf);
                } else {
                    Serial.print("[G] "); Serial.println(gantryRxBuf);
                }
            }
            gantryRxBuf = "";
        }
    }
}
#endif
#endif

Execution2019Handler::Execution2019Handler()
{
	// Constructor
}

Execution2019Handler::~Execution2019Handler()
{
	// Destructor
}

void Execution2019Handler::beginControllerLink()
{
#if defined(Stainer_Master_PCB)
    // Re-establish application boot mode after every Master restart, not only
    // after flashing. Preload NRST HIGH before enabling open-drain output so
    // startup releases reset without introducing a low pulse.
    digitalWrite(NM_BootPin, LOW);
    pinMode(NM_BootPin, OUTPUT);
    digitalWrite(G_BootPin, LOW);
    pinMode(G_BootPin, OUTPUT);
    digitalWrite(NM_ResetPin, HIGH);
    pinMode(NM_ResetPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(G_ResetPin, HIGH);
    pinMode(G_ResetPin, OUTPUT_OPEN_DRAIN);
    PiSerial.begin(BAUD_RATE);
#elif defined(Stainer_Gantry_PCB) || defined(Nozzle_Mount_PCB)
    MasterSerial.begin(BAUD_RATE);
#endif
}

void Execution2019Handler::setup()
{
#ifdef Master
	#ifdef Stainer_Gantry_PCB
		// Gantry Motors
		GXMotor.setup();
		GZMotor.setup();
		GRMotor.setup();
		GYMotor.setup();
		// Extra Motor
		XMotor.setup();
		#if GANTRY_MAG_HALL_CAL_EXPERIMENT
		analogReadResolution(12);
		pinMode(GANTRY_MAG_HALL_PIN, INPUT);
		(void)analogRead(GANTRY_MAG_HALL_PIN);
		Serial.println("MAGCAL: Hall experiment enabled on IR5/PA7");
		#endif
		#if !MAGAZINE_UART_TEST
		pinMode(I2C3_INT1, INPUT_PULLUP);
		pinMode(I2C3_INT2, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(I2C3_INT1), magazine1AlertISR, FALLING);
		attachInterrupt(digitalPinToInterrupt(I2C3_INT2), magazine2AlertISR, FALLING);
		magazine1AlertPending = digitalRead(I2C3_INT1) == LOW;
		magazine2AlertPending = digitalRead(I2C3_INT2) == LOW;
		#else
		// The UART wiring has TX/RX/power/ground only. Insertion, removal,
		// type, and slide changes are detected by the existing status poll.
		magazine1AlertPending = false;
		magazine2AlertPending = false;
		#endif
	#endif

	#ifdef Nozzle_Mount_PCB
		// Master PCB communication UART
		// Controller UART is initialized before sensor probing in sketch setup.
		// Stain Motors
		SXMotor.setup();
		SYMotor.setup();
		// Wash Motors
		WXMotor.setup();
		WYMotor.setup();
		// Buffer Motors
		BXMotor.setup();
		BYMotor.setup();
		// DC Motor
		dcMotor.setup();
		// DC Fan
		dcFan1.setup();
		dcFan2.setup();
		dcFan2.runFan(255); // Nozzle DCF2 defaults to full duty on boot.
		dcFan3.setup();
		dcFan4.setup();
	#endif

	#ifdef Stainer_Master_PCB
		// Pi/ESP32 communication UART (Option A — listen on both USB and PiSerial)
		// Controller UART is initialized before sensor probing in sketch setup.
		// Ask ESP32 for last-used stain profile. ESP32 replies (when available)
		// with "PROFILE <code>" which the master command parser handles normally.
		// Until/unless that reply arrives, activeProfileId stays at PROFILE_RP.
		PiSerial.println("GET_PROFILE");
		printProfileSummary(profile());
		// Nozzle Mount communication UART
		NozzleSerial.begin(BAUD_RATE);
		Serial.println("NOZZLE: waiting for nozzle mount ready...");
		// Gantry communication UART
		GantrySerial.begin(BAUD_RATE);
		Serial.println("GANTRY: waiting for gantry ready...");
		// X, Y, Z, T Motors
		XMotor.setup();
		YMotor.setup();
		ZMotor.setup();
		TMotor.setup();
		TMotor.setStepGapTracked(false); // bed barrier latency is reported separately
		// The bed steps exactly once per barrier ACK; it must stay polled.
		TMotor.motor.disableInterruptStepping();
		// Spare TMC motors M14, M15, M17, M18 — must call setup() so step pins,
		// TMC driver, and AccelStepper defaults (speed/accel) are initialized.
		M14Motor.setup();
		M15Motor.setup();
		M17Motor.setup();
		M18Motor.setup();

		// DC Motors (Mix, Drain, Wash, Suction)
		mixdc1.setup();
		mixdc2.setup();
		draindc.setup();
		washdc1.setup();
		washdc2.setup();
		suctiondc1.setup();
		suctiondc2.setup();
		// DC Fans:
		//   MIXR (PB15 = TIM1_CH3N) — keep on timer for PWM. PE13 (TIM1_CH3) is
		//   unused so no cross-talk. Used by SXCAS for the cascade fan.
		dcFan2.setup();
		//   MIXD (PB14 = TIM1_CH2N) — driven as plain GPIO. Cannot use the timer
		//   here because DCW1 (PE11 = TIM1_CH2) is now PWM, and routing PB14 to
		//   TIM1_CH2N would mirror DCW1's PWM onto the fan. Don't call dcFan1.setup().
		pinMode(MIXD, OUTPUT); digitalWrite(MIXD, LOW);
		// Post-swap pin map:
		//   DCD  = PE11 (was DCW1), forced digital so TIM1_CH2 stays clear for MIXD on PB14.
		//   DCW1 = PD15 (TIM4_CH4) — real PWM via washdc1.runMotor() / analogWrite.
		//   DCM2 = PG9 (no PWM timer) — threshold-digital via mixdc2.runMotor().
		pinMode(D_Motor, OUTPUT); digitalWrite(D_Motor, LOW);
		// DCW2 (PE9 = TIM1_CH1) — PWM via washdc2.runMotor(). PE8 (CH1N) unused.
	#endif

	limitSwitch.setup();
	#ifdef Nozzle_Mount_PCB
		#if NZ_USE_HALL_HOME_SENSORS
			nzSetupHallHomeInputs();
		#else
			Serial.println("NZHO: DIN LIMIT-SWITCH HOMING ENABLED");
		#endif
	#endif
	#ifdef Stainer_Gantry_PCB
	// Master PCB communication UART
	// Controller UART is initialized before sensor probing in sketch setup.
	#endif
	#ifdef Nozzle_Mount_PCB
		Serial.println("NZHO: boot homing disabled; use NZHO to home manually");
		// Advertise command availability; axes have not been homed.
		MasterSerial.println("Nozzle: Ready");
	#endif
	#ifdef Stainer_Gantry_PCB
		// TEMP DISABLED for Magazine OTA testing.
		// Re-enable this boot GHOME/LOAD block after magazine firmware update validation.
		/*
		Serial.println("GBOOT: GHOME started");
		if (runGhomeBlocking()) {
			Serial.println("GBOOT: GHOME complete");
			Serial.println("GBOOT: detecting magazine...");
			const int detectedMag = pickAvailableMagazine(0);
			if (detectedMag != 0) {
				Serial.print("GBOOT: MAG"); Serial.print(detectedMag); Serial.println(" detected");
				Serial.println("GBOOT: LOAD activation started");
				runBootLoadActivationBlocking();
				Serial.println("GBOOT: LOAD activation complete");
			} else {
				Serial.println("GBOOT: no magazine detected, skipping LOAD activation");
			}
			MasterSerial.println("Gantry: Ready");
		} else {
			Serial.println("GBOOT: GHOME failed");
			MasterSerial.println("Gantry: Failed");
		}
		*/
		Serial.println("GBOOT: startup GHOME disabled for Magazine OTA testing");
		MasterSerial.println("Gantry: Ready");
	#endif
#endif
}

// =============================================================================
// STM32 firmware flasher via the ROM system bootloader (AN3155 over UART).
// Generalized over target (Gantry / Nozzle) and binary source (USB or ESP32 relay).
//
// Two entry points from dispatchMasterCommandLine():
//   • GFLASH <size>        (USB dev) — flash Gantry, raw .bin piped over USB Serial.
//   • FLASH G|N <size>     (ESP32)   — flash Gantry/Nozzle, raw .bin streamed over
//                                      PiSerial with a FLASHRDY / ACK flow-control
//                                      handshake (the raw UART has no host buffering,
//                                      so we gate one 256-byte chunk per ACK).
//
// For each session the target's BOOT0 is pulled high, NRST is pulsed, and AN3155 is
// spoken over the target UART. AN3155 needs 8E1 framing, so the target UART is
// reopened SERIAL_8E1 for the session and restored to SERIAL_8N1 (BAUD_RATE) after.
// =============================================================================
#if defined(Master) && defined(Stainer_Master_PCB)

// One AN3155 session targets a specific STM32 over a specific UART + BOOT0/NRST.
struct StmFlashTarget {
    LivoHardwareSerial* ser;
    uint8_t         bootPin;
    uint8_t         resetPin;
    const char*     name;
};

static bool stmWaitForByte(LivoHardwareSerial& s, uint8_t expected, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (s.available()) {
            int b = s.read();
            return (b == expected);
        }
    }
    return false;
}

static bool stmSendCommand(LivoHardwareSerial& s, uint8_t cmd) {
    s.write(cmd);
    s.write((uint8_t)(cmd ^ 0xFF));
    s.flush();
    return stmWaitForByte(s, 0x79, 1000);
}

static bool stmSendAddress(LivoHardwareSerial& s, uint32_t addr) {
    uint8_t b[5];
    b[0] = (addr >> 24) & 0xFF;
    b[1] = (addr >> 16) & 0xFF;
    b[2] = (addr >> 8)  & 0xFF;
    b[3] = addr         & 0xFF;
    b[4] = b[0] ^ b[1] ^ b[2] ^ b[3];
    s.write(b, 5);
    s.flush();
    return stmWaitForByte(s, 0x79, 1000);
}

static uint16_t stmDrainRxBounded(LivoHardwareSerial& s, unsigned long maxMs) {
    const unsigned long start = millis();
    uint16_t drained = 0;
    while (millis() - start < maxMs) {
        if (s.available()) {
            s.read();
            if (drained < UINT16_MAX) drained++;
        } else {
            break;
        }
    }
    return drained;
}

static bool stmEnterBootloader(LivoHardwareSerial& s, uint8_t bootPin, uint8_t resetPin) {
    pinMode(bootPin, OUTPUT);
    // NRST is active-low and must be released, not actively driven high.
    pinMode(resetPin, OUTPUT_OPEN_DRAIN);
    // Gantry may be producing continuous application logs. Never let RX cleanup
    // prevent the reset sequence from running.
    const uint16_t preResetDrained = stmDrainRxBounded(s, 20);
    if (preResetDrained) {
        Serial.print("[flash] drained pre-reset UART bytes: ");
        Serial.println(preResetDrained);
    }
    digitalWrite(bootPin, HIGH);    // BOOT0 = 1 → system bootloader
    digitalWrite(resetPin, LOW);
    delay(50);
    digitalWrite(resetPin, HIGH);   // release reset
    Serial.println("[flash] BOOT0=HIGH, NRST pulse complete");
    delay(300);                     // bootloader startup
    const uint16_t postResetDrained = stmDrainRxBounded(s, 20);
    if (postResetDrained) {
        Serial.print("[flash] drained post-reset UART bytes: ");
        Serial.println(postResetDrained);
    }
    Serial.println("[flash] sending 0x7F sync (5 attempts)...");
    // Autobaud sync. Bootloader replies 0x79 (ACK) or 0x1F (already initialised).
    // Retry a few times, and log what (if anything) came back so failures diagnose:
    //   silence      → no reset / wiring (BOOT0/NRST/UART) / target unpowered.
    //   wrong byte(s)→ target booted into APP (BOOT0 not high) or on another iface.
    int heard = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        s.write((uint8_t)0x7F);
        s.flush();
        unsigned long start = millis();
        while (millis() - start < 1000) {
            if (s.available()) {
                uint8_t b = (uint8_t)s.read();
                if (b == 0x79 || b == 0x1F) return true;
                heard++;
                Serial.print("[flash] unexpected sync byte 0x"); Serial.println(b, HEX);
            }
        }
    }
    Serial.println(heard == 0 ? "[flash] SILENCE to 0x7F - no reset/wiring or target unpowered"
                              : "[flash] bytes but no ACK - target in APP (BOOT0 low?) or wrong iface");
    return false;
}

static void stmExitBootloader(LivoHardwareSerial& s, uint8_t bootPin, uint8_t resetPin) {
    (void)s;
    digitalWrite(bootPin, LOW);     // BOOT0 = 0 → run application
    delay(10);
    digitalWrite(resetPin, LOW);
    delay(50);
    digitalWrite(resetPin, HIGH);
    delay(100);
}

static bool stmEraseAll(LivoHardwareSerial& s) {
    if (!stmSendCommand(s, 0x44)) return false; // Extended Erase
    // Mass erase code = 0xFFFF, checksum = 0xFF ^ 0xFF = 0x00
    s.write((uint8_t)0xFF);
    s.write((uint8_t)0xFF);
    s.write((uint8_t)0x00);
    s.flush();
    return stmWaitForByte(s, 0x79, 60000); // mass erase ~30s on F407
}

static bool stmWriteMemory(LivoHardwareSerial& s, uint32_t addr, const uint8_t* data, uint16_t len) {
    if (len == 0 || len > 256 || (len % 4) != 0) return false;
    if (!stmSendCommand(s, 0x31)) return false; // Write Memory
    if (!stmSendAddress(s, addr)) return false;
    uint8_t n = (uint8_t)(len - 1);
    s.write(n);
    uint8_t checksum = n;
    for (uint16_t i = 0; i < len; i++) {
        s.write(data[i]);
        checksum ^= data[i];
    }
    s.write(checksum);
    s.flush();
    return stmWaitForByte(s, 0x79, 5000);
}

static bool stmReadExact(LivoHardwareSerial& s, uint8_t* data, uint16_t len, unsigned long timeoutMs) {
    uint16_t got = 0;
    unsigned long lastByteMs = millis();
    while (got < len) {
        if (s.available()) {
            data[got++] = (uint8_t)s.read();
            lastByteMs = millis();
        } else if (millis() - lastByteMs > timeoutMs) {
            return false;
        }
    }
    return true;
}

static bool stmReadMemory(LivoHardwareSerial& s, uint32_t addr, uint8_t* data, uint16_t len) {
    if (len == 0 || len > 256) return false;
    if (!stmSendCommand(s, 0x11)) return false; // Read Memory
    if (!stmSendAddress(s, addr)) return false;
    uint8_t n = (uint8_t)(len - 1);
    s.write(n);
    s.write((uint8_t)(n ^ 0xFF));
    s.flush();
    if (!stmWaitForByte(s, 0x79, 1000)) return false;
    return stmReadExact(s, data, len, 1000);
}

static bool stmVerifyMemory(LivoHardwareSerial& s, uint32_t addr, const uint8_t* expected, uint16_t len) {
    uint8_t actual[256];
    if (!stmReadMemory(s, addr, actual, len)) return false;
    for (uint16_t i = 0; i < len; i++) {
        if (actual[i] != expected[i]) {
            Serial.print("[flash] verify mismatch @0x");
            Serial.print(addr + i, HEX);
            Serial.print(" expected 0x");
            Serial.print(expected[i], HEX);
            Serial.print(" got 0x");
            Serial.println(actual[i], HEX);
            return false;
        }
    }
    return true;
}

static void stmPrintHexBytes(const char* prefix, const uint8_t* data, uint8_t len) {
    Serial.print(prefix);
    for (uint8_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
}

// Flash `totalBytes` from `binSrc` into target `t` via AN3155.
// The target UART is reopened 8E1 for the AN3155 session and restored to 8N1 after.
// Progress/handshake tokens (FLASHRDY, ACK <n>) are printed on the master console
// (mirrored to USB + PiSerial): the ESP32 relay waits for FLASHRDY before streaming
// and for one ACK per 256-byte chunk; on USB the tokens are harmless.
static bool stmFlash(Stream& binSrc, const StmFlashTarget& t, uint32_t totalBytes, bool drainBeforeReady = false) {
    LivoHardwareSerial& s = *t.ser;
    s.end();
    s.begin(BAUD_RATE, SERIAL_8E1);   // AN3155 requires even parity
    delay(100);

    Serial.print("[flash] "); Serial.print(t.name); Serial.println(": entering bootloader...");
    if (!stmEnterBootloader(s, t.bootPin, t.resetPin)) {
        Serial.println("[flash] sync FAILED — check wiring/BOOT0/NRST");
        s.end(); s.begin(BAUD_RATE);
        return false;
    }
    Serial.println("[flash] sync OK");

    Serial.println("[flash] mass erasing flash (~30s)...");
    if (!stmEraseAll(s)) {
        Serial.println("[flash] erase FAILED");
        stmExitBootloader(s, t.bootPin, t.resetPin);
        s.end(); s.begin(BAUD_RATE);
        return false;
    }
    Serial.println("[flash] erase OK");

    // Erase done — signal the sender it may begin streaming the payload.
    if (drainBeforeReady) {
        uint16_t drained = 0;
        while (binSrc.available()) {
            binSrc.read();
            drained++;
        }
        if (drained) {
            Serial.print("[flash] drained ");
            Serial.print(drained);
            Serial.println(" stale byte(s) before binary stream");
        }
    }
    Serial.println("FLASHRDY");

    const uint32_t BASE_ADDR = 0x08000000;
    uint32_t written = 0;
    uint8_t  buf[256];
    bool     ok = true;

    while (written < totalBytes) {
        uint32_t remaining = totalBytes - written;
        uint16_t chunk = (remaining > 256) ? 256 : (uint16_t)remaining;
        uint16_t got = 0;
        unsigned long lastByteMs = millis();
        while (got < chunk) {
            if (binSrc.available()) {
                buf[got++] = (uint8_t)binSrc.read();
                lastByteMs = millis();
            } else if (millis() - lastByteMs > 15000) {
                Serial.println("[flash] receive TIMEOUT");
                ok = false;
                break;
            }
        }
        if (!ok) break;
        if (written == 0) {
            stmPrintHexBytes("[flash] first 16 received: ", buf, (got < 16) ? got : 16);
        }
        // Pad final chunk to 4-byte alignment with 0xFF (erased state)
        while ((chunk % 4) != 0) buf[chunk++] = 0xFF;
        uint32_t writeAddr = BASE_ADDR + written;
        if (!stmWriteMemory(s, writeAddr, buf, chunk)) {
            Serial.print("[flash] write FAILED at offset 0x");
            Serial.println(written, HEX);
            ok = false;
            break;
        }
        if (!stmVerifyMemory(s, writeAddr, buf, chunk)) {
            Serial.print("[flash] verify FAILED at offset 0x");
            Serial.println(written, HEX);
            ok = false;
            break;
        }
        written += chunk;
        Serial.print("ACK ");            // per-chunk flow-control ack (also progress)
        Serial.println(written);
    }

    stmExitBootloader(s, t.bootPin, t.resetPin);
    s.end();
    s.begin(BAUD_RATE);                  // restore normal 8N1 comms with the target

    if (ok) Serial.println("[flash] write complete, target reset into new firmware");
    return ok;
}

// Concrete targets (BOOT0/NRST pins defined in StainerMasterPCBV1.h).
static const StmFlashTarget FLASH_TGT_GANTRY = { &GantrySerial, G_BootPin,  G_ResetPin,  "gantry" };
static const StmFlashTarget FLASH_TGT_NOZZLE = { &NozzleSerial, NM_BootPin, NM_ResetPin, "nozzle" };

// Diagnostics — isolate a target's boot/reset wiring without needing a .bin.
// GRESET/NRESET pulse NRST only (BOOT0 low) → the target should reboot into its
// app; if wired you'll see its boot chatter. GBOOT/NBOOT do the full ROM-bootloader
// entry + autobaud sync (BOOT0 high + reset @8E1) and report the result.
// GBOOTHI/NBOOTHI hold BOOT0 high without reset so the node can be probed.
static void stmDiagReset(const StmFlashTarget& t) {
    Serial.print("[diag] "); Serial.print(t.name);
    Serial.println(": BOOT0=LOW, pulsing NRST — target should reboot into its app");
    pinMode(t.bootPin, OUTPUT);
    pinMode(t.resetPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(t.bootPin, LOW);
    digitalWrite(t.resetPin, LOW); delay(50); digitalWrite(t.resetPin, HIGH);
    Serial.println("[diag] NRST released (confirms NRST wire if the target reboots)");
}

static void stmDiagBootHigh(const StmFlashTarget& t) {
    Serial.print("[diag] "); Serial.print(t.name);
    Serial.println(": holding BOOT0 HIGH for 8s (no reset) - probe BOOT0 node now");
    pinMode(t.bootPin, OUTPUT);
    digitalWrite(t.bootPin, HIGH);
    delay(8000);
    digitalWrite(t.bootPin, LOW);
    Serial.print("[diag] "); Serial.print(t.name);
    Serial.println(": BOOT0 released LOW");
}

static void stmDiagBoot(const StmFlashTarget& t) {
    LivoHardwareSerial& s = *t.ser;
    Serial.print("[diag] "); Serial.print(t.name);
    Serial.println(": ROM bootloader entry test @8E1 (no .bin)...");
    s.end();
    s.begin(BAUD_RATE, SERIAL_8E1);
    bool ok = stmEnterBootloader(s, t.bootPin, t.resetPin);
    Serial.println(ok ? "[diag] ACK received (0x79/0x1F) — AN3155 path is GOOD"
                      : "[diag] no ACK (reason logged above)");
    stmExitBootloader(s, t.bootPin, t.resetPin);
    s.end();
    s.begin(BAUD_RATE);
}

static bool waitGantryLineToken(const char* token, unsigned long timeoutMs) {
    String line;
    const unsigned long started = millis();
    while (millis() - started < timeoutMs) {
        while (GantrySerial.available()) {
            const char c = (char)GantrySerial.read();
            if (c == '\n' || c == '\r') {
                line.trim();
                if (line.length()) {
                    Serial.print("[G] "); Serial.println(line);
                    if (line.startsWith(token)) return true;
                    if (line.startsWith("MAGFAIL")) return false;
                }
                line = "";
            } else {
                line += c;
                if (line.length() > 160) line = "";
            }
        }
    }
    Serial.print("[G] TIMEOUT waiting for "); Serial.println(token);
    return false;
}

static void printHexByteToGantry(uint8_t value) {
    static const char hex[] = "0123456789ABCDEF";
    GantrySerial.write(hex[(value >> 4) & 0x0F]);
    GantrySerial.write(hex[value & 0x0F]);
}

static bool sendMagazineDataLineToGantry(uint32_t offset, const uint8_t* data, uint16_t len) {
    GantrySerial.print("MAGDATA "); GantrySerial.print(offset);
    GantrySerial.print(' '); GantrySerial.print(len); GantrySerial.print(' ');
    for (uint16_t i = 0; i < len; ++i) printHexByteToGantry(data[i]);
    GantrySerial.println();
    GantrySerial.flush();
    return waitGantryLineToken("MAGACK", 10000);
}

static bool relayMagazineFlash(Stream& source, int holder, uint32_t totalBytes, uint32_t crc) {
    if (holder < 1 || holder > 2 || totalBytes == 0 || crc == 0) return false;
    while (GantrySerial.available()) (void)GantrySerial.read();
    GantrySerial.println();
    GantrySerial.print("MAGFLASH "); GantrySerial.print(holder);
    GantrySerial.print(' '); GantrySerial.print(totalBytes);
    GantrySerial.print(' '); GantrySerial.println(crc);
    GantrySerial.flush();
    if (!waitGantryLineToken("MAGRDY", 90000)) return false;

    Serial.println("FLASHRDY");
    uint32_t written = 0;
    uint8_t outer[256];
    while (written < totalBytes) {
        const uint16_t wanted = (totalBytes - written > sizeof(outer))
            ? sizeof(outer) : (uint16_t)(totalBytes - written);
        uint16_t received = 0;
        unsigned long lastByteMs = millis();
        while (received < wanted) {
            if (source.available()) {
                outer[received++] = (uint8_t)source.read();
                lastByteMs = millis();
            } else if (millis() - lastByteMs > 15000UL) {
                Serial.println("[flash] magazine receive TIMEOUT");
                return false;
            }
        }
        for (uint16_t pos = 0; pos < wanted;) {
            const uint16_t count = min((uint16_t)20, (uint16_t)(wanted - pos));
            if (!sendMagazineDataLineToGantry(written + pos, outer + pos, count)) return false;
            pos += count;
        }
        written += wanted;
        Serial.print("ACK "); Serial.println(written);
    }
    GantrySerial.println("MAGCOMMIT");
    GantrySerial.flush();
    return waitGantryLineToken("MAGDONE", 30000);
}

#endif // Stainer_Master_PCB

// Master-side prefix router: read from USB Serial, dispatch by leading token "G;" / "N;" / "M;".
// "G;<cmd>" -> forward over GantrySerial. "N;<cmd>" -> forward over NozzleSerial.
// "M;<cmd>" or no prefix -> execute locally via livoCommunication.process2019Byte().
// Whitespace tolerant: "G; cmd", "G;cmd", " G ; cmd " all parse the same.
// Special: "GFLASH <size>" triggers the firmware flasher above.
#if defined(Master) && defined(Stainer_Master_PCB)
// Shared dispatcher — same line-handling logic regardless of source (USB or PiSerial)
static void dispatchMasterCommandLine(const String& rawLine, const char* sourceTag) {
    String line = rawLine; line.trim();
    if (line.length() == 0) return;

    // Firmware flashers over the STM32 ROM bootloader (AN3155).
    String upper = line; upper.toUpperCase();
    const bool fromPi = (sourceTag && sourceTag[0] == 'P');

    String commandBody = upper;
    const int commandSeparator = commandBody.indexOf(';');
    if (commandSeparator >= 0) commandBody = commandBody.substring(commandSeparator + 1);
    commandBody.trim();
    if(commandBody=="BEDRESET") {
        if(tFeedActive){Serial.println("BEDRESET rejected: FEEDSTOP first");return;}
        masterBedReset();return;
    }
    if(masterBedEstablished && (tFeedActive || lastNozzleQueueDepth || anyCascadeActive())) {
        const bool routedToGantry=commandSeparator>=0 && upper.substring(0,commandSeparator)=="G";
        const bool allowed=routedToGantry || commandBody=="FEEDSTOP" || commandBody.startsWith("FEED ") ||
            commandBody.startsWith("REAGENTLOCK ") || commandBody=="ID" || commandBody=="COMMSTAT" ||
            commandBody=="LOOPSTAT" ||
            commandBody=="PROFILE" || commandBody=="PARA STOP";
        if(!allowed){Serial.println("[BED] command rejected while spatial run is active");return;}
    }
    if (serviceProductionBusy()) {
        // Also covers console-started runs not tracked by the ESP UI.
        if (serviceBackgroundCommand(commandBody)) return;
        if (commandBody == "DLOCK 1" || commandBody.startsWith("DTEST ")) {
            Serial.println("DBUSY PRODUCTION_ACTIVE"); return;
        }
    }

    if (ServiceDiagnostics::locked() && !fromPi) return;
    if (ServiceDiagnostics::locked() && line.indexOf(';') < 0 && !line.startsWith("D") && line != "ID") return;
    if (handleReagentLockControl(line, sourceTag)) return;

    // Flasher diagnostics (USB or ESP32): isolate a target's boot/reset wiring.
    if (upper == "GRESET") { stmDiagReset(FLASH_TGT_GANTRY); return; }
    if (upper == "NRESET") { stmDiagReset(FLASH_TGT_NOZZLE); return; }
    if (upper == "GBOOT")  { stmDiagBoot(FLASH_TGT_GANTRY);  return; }
    if (upper == "NBOOT")  { stmDiagBoot(FLASH_TGT_NOZZLE);  return; }
    if (upper == "GBOOTHI") { stmDiagBootHigh(FLASH_TGT_GANTRY); return; }
    if (upper == "NBOOTHI") { stmDiagBootHigh(FLASH_TGT_NOZZLE); return; }

    // Legacy USB dev command: GFLASH <size> → flash Gantry, .bin piped over USB Serial.
    if (upper.startsWith("GFLASH ")) {
        long size = upper.substring(7).toInt();
        if (size > 0) stmFlash(Serial, FLASH_TGT_GANTRY, (uint32_t)size);
        else Serial.println("[flash] usage: GFLASH <size_in_bytes>");
        return;
    }
    // ESP32 relay: FLASH G|N <size> → .bin streamed over PiSerial with FLASHRDY/ACK handshake.
    if (fromPi && upper.startsWith("FLASH ")) {
        String args = upper.substring(6);
        args.trim();
        int sp = args.indexOf(' ');
        String tgtToken = (sp >= 0) ? args.substring(0, sp) : args;
        String rest = (sp >= 0) ? args.substring(sp + 1) : "";
        rest.trim();
        int sp2 = rest.indexOf(' ');
        const long size = (sp2 >= 0) ? rest.substring(0, sp2).toInt() : rest.toInt();
        uint32_t crc = 0;
        if (sp2 >= 0) {
            String crcText = rest.substring(sp2 + 1);
            crcText.trim();
            crc = (uint32_t)strtoul(crcText.c_str(), nullptr, 0);
        }
        bool ok = false;
        if (size > 0 && tgtToken == "G")       ok = stmFlash(PiSerial, FLASH_TGT_GANTRY, (uint32_t)size, true);
        else if (size > 0 && tgtToken == "N")  ok = stmFlash(PiSerial, FLASH_TGT_NOZZLE, (uint32_t)size, true);
        else if (size > 0 && crc != 0 && (tgtToken == "M1" || tgtToken == "M2")) {
            ok = relayMagazineFlash(PiSerial, tgtToken == "M1" ? 1 : 2, (uint32_t)size, crc);
        }
        else { Serial.println("FLASHFAIL bad-args"); return; }
        Serial.println(ok ? "FLASHDONE" : "FLASHFAIL");
        return;
    }

    int sepPos = line.indexOf(';');
    if (sepPos > 0) {
        String prefix = line.substring(0, sepPos); prefix.trim();
        String body   = line.substring(sepPos + 1); body.trim();
        if (prefix.length() == 1) {
            char p = prefix.charAt(0);
            if (p == 'G' || p == 'g') {
                GantrySerial.println(body);
                Serial.print("[->G "); Serial.print(sourceTag); Serial.print("] "); Serial.println(body);
                return;
            }
            if (p == 'N' || p == 'n') {
                NozzleSerial.println(body);
                Serial.print("[->N "); Serial.print(sourceTag); Serial.print("] "); Serial.println(body);
                return;
            }
            if (p == 'M' || p == 'm') {
                if (handleReagentLockControl(body, sourceTag)) return;
                for (size_t i = 0; i < body.length(); i++) {
                    livoCommunication.process2019Byte((byte)body.charAt(i));
                }
                livoCommunication.process2019Byte((byte)'\n');
                return;
            }
        }
    }
    // No recognized prefix — execute locally
    for (size_t i = 0; i < line.length(); i++) {
        livoCommunication.process2019Byte((byte)line.charAt(i));
    }
    livoCommunication.process2019Byte((byte)'\n');
}
#endif

void Execution2019Handler::serviceMasterCommandRouter() {
    static bool piBufDiscarding = false;
    static bool usbBufDiscarding = false;
#if defined(Master) && defined(Stainer_Master_PCB)
    // Execute each local command before routing the next line. Check Systems
    // relies on FEEDSTOP -> PROFILE -> nozzle NZHO -> MLOAD ordering.
    if (livoCommunication.getinputBytesAvailable()) return;
    // ---- USB Serial (developer / debug interface) ----
    static String usbBuf = "";
    unsigned usbBudget = 64;
    while (Serial.available()) {                  // ↳ via mirroredSerial.read() → real USB
        if (!usbBudget--) break;
        char c = (char)Serial.read();
        if (receiveUartLineByte(c, usbBuf, usbBufDiscarding)) {
            if (usbBuf.length() > 0) {
                dispatchMasterCommandLine(usbBuf, "U");
                usbBuf = "";
                if (livoCommunication.getinputBytesAvailable()) return;
                break;
            }
        }
    }

    // ---- PiSerial UART (ESP32 / Pi interface) ----
    static String piBuf = "";
    unsigned piBudget = 64;
    while (piBudget-- && PiSerial.available()) {
        char c = (char)PiSerial.read();
        if (receiveUartLineByte(c, piBuf, piBufDiscarding)) {
            if (piBuf.length() > 0) {
                dispatchMasterCommandLine(piBuf, "P");
                piBuf = "";
                if (livoCommunication.getinputBytesAvailable()) return;
                break;
            }
        }
    }
#endif
}

static void serviceRunBusyTelemetry() {
#ifdef Master
    static uint32_t reportedAt = 0;
    static int previous = -1;
    const bool busy = !ServiceDiagnostics::locked() && serviceProductionBusy();
    if (previous != (int)busy || millis() - reportedAt >= 5000UL) {
#ifdef Stainer_Master_PCB
        PiSerial.print("RUNBUSY "); PiSerial.println(busy ? 1 : 0);
#else
        MasterSerial.print("RUNBUSY "); MasterSerial.println(busy ? 1 : 0);
#endif
        previous = busy; reportedAt = millis();
    }
#endif
}

void Execution2019Handler::loop()
{
#ifdef Master
    if(ServiceDiagnostics::locked()) {
#ifdef Stainer_Master_PCB
        serviceNozzleSerial();serviceGantrySerial();
#elif defined(Stainer_Gantry_PCB)
        serviceMasterUartIntercept();
#elif defined(Nozzle_Mount_PCB)
        livoCommunication.serialEventStream(MasterSerial);
#endif
        ServiceDiagnostics::loop();serviceRunBusyTelemetry();return;
    }
#endif
#ifdef Master
	#ifdef Stainer_Gantry_PCB
		// Accept commands forwarded from master via UART. Custom reader intercepts
		// SETCAL<H><T> for runtime threshold overrides; everything else flows to the
		// normal protocol2019 parser identically to serialEventStream().
		serviceMasterUartIntercept();
        if(ServiceDiagnostics::locked()){ServiceDiagnostics::loop();return;}
		// Gantry Motors
		GXMotor.loop();
		GZMotor.loop();
		GRMotor.loop();
		GYMotor.loop();
		// Extra Motor
		XMotor.loop();

		// While any gantry motor is moving, skip slow sensor-bound services so the
		// software stepper isn't starved between step pulses. They resume the
		// instant the motor stops. Responses already received remain buffered.
		const bool gantryMoving =
			GXMotor.isMoving() || GZMotor.isMoving() ||
			GRMotor.isMoving() || GYMotor.isMoving() || XMotor.isMoving();

		if (!gantryMoving) {
			// Hall Modules
			Magazine1Ir.loop();
			Magazine2Ir.loop();
			serviceMagazine2PulseMonitor();
			serviceMagazineSlideUpdates();
			serviceMagazineAutomation();
			serviceMagazineBackgroundPolling();
		}
		tickAutoLoad();
	#endif

	#ifdef Nozzle_Mount_PCB
		// Accept commands forwarded from master via UART
		livoCommunication.serialEventStream(MasterSerial);
        if(ServiceDiagnostics::locked()){ServiceDiagnostics::loop();return;}
		if(!nzBed.synchronized() || (nzBedEnabled && !nzBedFault)) {
		// Stain Motors
		SXMotor.loop();
		SYMotor.loop();
		// Wash Motors
		WXMotor.loop();
		WYMotor.loop();
		// Buffer Motors
		BXMotor.loop();
		BYMotor.loop();
        } else {
		// Bed paused or faulted: freeze, as skipping loop() did with polled stepping.
		for (TMCModule* m : {&SXMotor,&SYMotor,&WXMotor,&WYMotor,&BXMotor,&BYMotor}) m->holdSteps();
        }
		serviceNozzleAutomation();
	#endif

	#ifdef Stainer_Master_PCB
		if(!masterBedEstablished || (tFeedActive && !masterBedFault)) {
		XMotor.loop();
		YMotor.loop();
		ZMotor.loop();
		// Spare TMC motors — needed for step-pulse generation
		M14Motor.loop();
		M15Motor.loop();
		M17Motor.loop();
		M18Motor.loop();
        } else {
		// Bed paused or faulted: freeze, as skipping loop() did with polled stepping.
		for (TMCModule* m : {&XMotor,&YMotor,&ZMotor,&M14Motor,&M15Motor,&M17Motor,&M18Motor}) m->holdSteps();
        }
        serviceMasterBed();

		serviceNozzleSerial();
		serviceGantrySerial();
		// ---- IR4 / IR6 master cascades ----
		serviceM15MixCascade();   // B1 pump completion only
		serviceS2Mixcade();       // M18 oscillation and dispatch completion only
		serviceMUnloadCascade();  // MUNLOAD: reverse-purge all pumps + DCW1/DCW2/DCD wash sequence
		serviceDrain();           // DRAIN: DCD on, stop when IR8 < 100 (after 3s pre-delay)
		serviceParaCheck();
	#endif

	serviceRunBusyTelemetry();
	ackManager.loop(displayDebug, ackAction);
	limitSwitch.loop();

#endif
}

void Execution2019Handler::displayHelpMenu()
{
	Serial.println("\n=== HELP MENU ===");
	Serial.println("PARA CHECK - Enable trace; select PROFILE and FEED next; counter starts at IR4");
	Serial.println("PARA STOP  - End trace; bed/automation continue (FEEDSTOP stops bed)");
	Serial.println("Board ID");
	Serial.println("ID 			- Give board ID and FW version");

	Serial.println("\nDebug & Info:");
	Serial.println("DEB <0/1>   - Toggle debug output");
	Serial.println("ACK <0/1>    - Toggle ACK management");

	Serial.println("\nMotion Commands - Gantry PCB:");
	Serial.println("GX/GY/GZ/GR <steps> - Absolute movement of Gantry X/Y/Z/R");
	Serial.println("GXR/GYR/GZR/GRR <steps> - Relative movement of Gantry X/Y/Z/R");

	Serial.println("\nMotion Commands - Nozzle Mount PCB:");
	Serial.println("SX/SY/WX/WY/BX/BY <steps> - Absolute movement of Stain X/Y, Wash X/Y, Buffer X/Y");
	Serial.println("SXR/SYR/WXR/WYR/BXR/BYR <steps> - Relative movement of Stain X/Y, Wash X/Y, Buffer X/Y");

	Serial.println("\nMotion Commands - Stainer Master PCB:");
	Serial.println("X/Y/Z/T <steps> - Absolute movement of X/Y/Z/T motors");
	Serial.println("XR/YR/ZR/TR <steps> - Relative movement of X/Y/Z/T motors");
	Serial.println("M14ML <uL>  - Dispense liquid via M14 (E) pump (mL x 1000)");
	Serial.println("XML <uL>    - Dispense liquid via X (S2) pump (mL x 1000)");
	Serial.println("YML <uL>    - Dispense liquid via Y (B2) pump (mL x 1000)");
	Serial.println("ZML <uL>    - Dispense liquid via Z (S1) pump (mL x 1000)");
	Serial.println("M15ML <uL>  - Dispense liquid via M15 (B1) pump (mL x 1000)");
	Serial.println("SXCAS       - Run SX cascade (DCW1 + DCS1 only)");
	Serial.println("S2MIX       - Run S2 dispense cascade (S2/X + B2/Y pumps + DCF2)");
	Serial.println("B1MIX       - Run B1 cascade (M15 dispense, then DCM1+DCM2 mix)");
	Serial.println("WYCAS       - Run WY cascade (DCW2, DCS2)");
	Serial.println("MLOAD       - Prime all liquid lines until each IR sensor drops below 100");
	Serial.println("MUNLOAD     - Reverse-purge all pumps, then DCW1+DCW2 5s, then DCD 10s");
	Serial.println("DRAIN       - DCD on at full duty, stop when IR8 rises above 100 (3s pre-delay)");
	Serial.println("ETSTOP/S1STOP/S2STOP/B1STOP/B2STOP - Safety stop for pump motors");
	Serial.println("ETDIS/S1DIS/S2DIS/B1DIS/B2DIS     - Disabled; use <pump>ML <uL> or profile dispense");
	Serial.print("FEED <label> - Start T motor continuous feed at label*");
	Serial.print(T_MOTOR_STEPS_PER_REV);
	Serial.println("/3600 steps/sec");
	Serial.println("FEEDSTOP    - Pause bed and retain spatial queues");
    Serial.println("BEDRESET    - Abort and clear queues after FEEDSTOP; re-home before restarting");

	Serial.println("\nConfiguration - Gantry PCB:");
	Serial.println("GXI/GYI/GZI/GRI <current> - Set Gantry X/Y/Z/R current");
	Serial.println("GXM/GYM/GZM/GRM <microsteps> - Set Gantry X/Y/Z/R microstepping");
	Serial.println("GXS/GYS/GZS/GRS <speed>   - Set Gantry X/Y/Z/R speed");
	Serial.println("GXA/GYA/GZA/GRA <acceleration> - Set Gantry X/Y/Z/R acceleration");
	Serial.println("GXHO <val> - Home Gantry X using lidar to 156.6 mm");
	Serial.println("GRHO <val> - Home Gantry R until IR3 value (PA5) drops under 500");
	Serial.println("GHOME <val> - Home all gantry axes (GXHO+GYHO+GZHO+GRHO)");
	Serial.println("GLOAD <val> - Move all axes to load position (GX=-33750,GY=0,GZ=0,GR=0)");
	Serial.println("RST - Run IR-based gantry recovery logic");
	Serial.println("LOAD/LOAD<mag> - Load highest-priority detected slide from active or explicit magazine (slot 1 highest)");
	Serial.println("LDM<mag><slide> - Load direct magazine slot, e.g. LDM11 or LDM114");
	#if defined(Stainer_Gantry_PCB) && GANTRY_MAG_HALL_CAL_EXPERIMENT
	Serial.println("MAGCAL [RUN|STATUS|APPLY|CLEAR] - Experimental holder-position Hall scan");
	#endif
	Serial.println("RLD - Read corrected lidar distance on GX (mm)");
	Serial.println("GRLD <val> - Sequence: R+12800, Y-927000, Y+927000, R+12800 then print LOADED");
	Serial.println("GRYO <val> - Home Gantry Y until IR4 value (PA6) rises to 1500");

	Serial.println("\nConfiguration - Nozzle Mount PCB:");
	#if defined(Nozzle_Mount_PCB) && NZ_USE_HALL_HOME_SENSORS
	Serial.println("NZHO <val>  - Home all nozzle axes (SY- BX+ SX+ WY- WX+) via AH49E Hall sensors");
	Serial.println("SXHO <val>  - Home SX  (+, IR13 >= 3100)");
	Serial.println("SYHO <val>  - Home SY  (-, IR3 >= 3100)");
	Serial.println("WXHO <val>  - Home WX  (+, IR10 >= 1750)");
	Serial.println("WYHO <val>  - Home WY  (-, IR7 >= 2200)");
	Serial.println("BXHO <val>  - Home BX  (+, IR11 >= 1950)");
	#else
	Serial.println("NZHO <val>  - Home all nozzle axes via retained DIN limit switches");
	Serial.println("SXHO <val>  - Home SX  (+, DIN5/PD14)");
	Serial.println("SYHO <val>  - Home SY  (-, DIN2/PD13)");
	Serial.println("WXHO <val>  - Home WX  (+, DIN3/PE7)");
	Serial.println("WYHO <val>  - Home WY  (+, DIN6/PE10)");
	Serial.println("BXHO <val>  - Home BX  (+, DIN4/PE8)");
	#endif
	Serial.println("SXI/SYI/WXI/WYI/BXI/BYI <current> - Set Stain X/Y, Wash X/Y, Buffer X/Y current");
	Serial.println("SXM/SYM/WXM/WYM/BXM/BYM <microsteps> - Set Stain X/Y, Wash X/Y, Buffer X/Y microstepping");
	Serial.println("SXS/SYS/WXS/WYS/BXS/BYS <speed>   - Set Stain X/Y, Wash X/Y, Buffer X/Y speed");
	Serial.println("SXA/SYA/WXA/WYA/BXA/BYA <acceleration> - Set Stain X/Y, Wash X/Y, Buffer X/Y acceleration");

	Serial.println("\nConfiguration - Stainer Master PCB:");
	Serial.println("XI/YI/ZI/TI <current> - Set X/Y/Z/T current");
	Serial.println("XM/YM/ZM/TM <microsteps> - Set X/Y/Z/T microstepping");
	Serial.println("XS/YS/ZS/TS <speed>   - Set X/Y/Z/T speed");
	Serial.println("XA/YA/ZA/TA <acceleration> - Set X/Y/Z/T acceleration");

	Serial.println("\nStatus & Info:");
	Serial.println("GXGP - Get Gantry X position from lidar (mm, home = 156.6)");
	Serial.println("GYGP/GZGP/GRGP - Get Gantry Y/Z/R position");
	Serial.println("SXGP/SYGP/WXGP/WYGP/BXGP/BYGP - Get Stain X/Y, Wash X/Y, Buffer X/Y position");
	Serial.println("XGP/YGP/ZGP/TGP - Get X/Y/Z/T position");
	Serial.println("GI  -  Get all motor currents");
	Serial.println("GM  -  Get all motor microstepping");
	Serial.println("GS  -  Get all motor speed");
	Serial.println("GA  -  Get all motor acceleration");

	Serial.println("\nIR Sensors:");
	Serial.println("PIR <0/1>   - Disable/Enable IR printing");
	Serial.println("RIR   - Read IR sensors");
#ifdef Nozzle_Mount_PCB
	Serial.println("RDHT  - Read DHT11 temperature/humidity on IR9 (PB1)");
	Serial.println("DCF1 - Always automatic humidity PWM; no enable command required");
#endif

	Serial.println("\nLimit Switches:");
	Serial.println("PLS <0/1>   - Disable/Enable Limit Switch printing");
	Serial.println("RLS   - Read Limit Switches");

	Serial.println("\nMagzine Checks");
	Serial.println("SMC <0/1> - Stop/Start Magazine Checks");

	Serial.println("\nHall Sensor: ");
	Serial.println("HS11 <1/2/3> - I2C1_1 hall sensor <values/filtered values/position>");
	Serial.println("HS12 <1/2/3> - I2C2_1 hall sensor <values/filtered values/position>");
	Serial.println("HS21 <1/2/3> - I2C2_1 hall sensor <values/filtered values/position>");
	Serial.println("HS22 <1/2/3> - I2C2_2 hall sensor <values/filtered values/position>");
	Serial.println("GXH <1/2/3>  - Gantry X Hall sensor <values/filtered values/position>");
	Serial.println("GXHI <1-10>  - Gantry X Hall individual sensor value");
	Serial.println("GZH <1/2/3>  - Gantry Z Hall sensor <values/filtered values/position>");
	Serial.println("GZHI <1-10>  - Gantry Z Hall individual sensor value");
	Serial.println("MH1         - Magazine1 (V2) IR sensor values 1-24");
	Serial.println("MH2         - Magazine2 (V2) IR sensor values 1-24");
	Serial.println("MH2<count>  - Magazine2 (V2) print <count> lines of hall values 1-24");
	Serial.println("MHP2<count> - Magazine2 (V2) start continuous 10 ms PE5-triggered monitoring, MHP20 stops it");
	Serial.println("MH111/112/121/122/14/15/16/17 - Magazine1 (V2) half/raw/filtered/status/lock commands");
	Serial.println("MH211/212/221/222/24/25/26/27 - Magazine2 (V2) half/raw/filtered/status/lock commands");

	Serial.println("\nSolenoid Commands:");
	Serial.println("M1L <val> - Lock Magazine 1 solenoid for 0.5 sec");
	Serial.println("M1U <val> - Unlock Magazine 1 solenoid for 0.5 sec");
	Serial.println("M2L <val> - Lock Magazine 2 solenoid for 0.5 sec");
	Serial.println("M2U <val> - Unlock Magazine 2 solenoid for 0.5 sec");

	Serial.println("\nAccelerometer: ");
	Serial.println("RAM <1/2> - Read Accelerometer 1/2");
	Serial.println("PAM <0/1/2> - Print Accelerometer stop/1/2");

	Serial.println("\nDC Motor: ");
	Serial.println("DCM <0/1-255>  -  Stop DC Motor/Run at speed pwm given");
	Serial.println("DCM1 <0/1-255> -  Stop Mixing1 DC Motor/Run at speed pwm given");
	Serial.println("DCM2 <0/1-255> -  Stop Mixing2 DC Motor/Run at speed pwm given");
	Serial.println("DCD <0/1-255>  -  Stop Drain DC Motor/Run at speed pwm given");
	Serial.println("DCW1 <0/1-255> -  Stop Wash1 DC Motor/Run at speed pwm given");
	Serial.println("DCW2 <0/1-255> -  Stop Wash2 DC Motor/Run at speed pwm given");
	Serial.println("DCS1 <0/1-255> -  Stop Suction1 DC Motor/Run at speed pwm given");
	Serial.println("DCS2 <0/1-255> -  Stop Suction2 DC Motor/Run at speed pwm given");

	Serial.println("\nDC Fan: ");
	Serial.println("DCF1 - Nozzle: automatic humidity control; Master: deprecated");
	Serial.println("DCF2 = 0/1-255 - Stop DC Fan2/Run at speed pwm given (nozzle only)");
	Serial.println("MIXD <0..255> - Master MX1919 H-bridge forward (IN1=PB14 PWM, IN2=PB15 LOW)");
	Serial.println("MIXR <0..255> - Master MX1919 H-bridge reverse (IN1=PB14 LOW,  IN2=PB15 PWM)");
	Serial.println("                0 = stop. MIXM/DCF1 commands deprecated on master.");

	Serial.println("===========================");
}

inline void Execution2019Handler::printIfValid(const char* label, int value) {
    if (value == protocol2019Handler.ignoreValue) return;

    if (isFirstPrint) isFirstPrint = false;
    else Serial.print(",");

    if (displayDebug)
    {
        Serial.print(label);
        Serial.print(":");
        Serial.print(value);
    }
}

void Execution2019Handler::displayPacketValues()
{
	isFirstPrint = true;
	// General Commands
	printIfValid("ID", protocol2019Handler.getIDValue());
	printIfValid("HELP", protocol2019Handler.getHelpValue());
    printIfValid("ACK",  protocol2019Handler.getAckValue());
    printIfValid("DEB",  protocol2019Handler.getDebValue());

	// Extra Motor
	printIfValid("X",   protocol2019Handler.getXValue());
    printIfValid("XR",  protocol2019Handler.getXRValue());
    printIfValid("XI",  protocol2019Handler.getXIValue());
    printIfValid("XM",  protocol2019Handler.getXMValue());
    printIfValid("XS",  protocol2019Handler.getXSValue());
    printIfValid("XA",  protocol2019Handler.getXAValue());
	printIfValid("XGP",  protocol2019Handler.getXGPValue());

	// TMC Stepper Motor Y
	printIfValid("Y",   protocol2019Handler.getYValue());
    printIfValid("YR",  protocol2019Handler.getYRValue());
    printIfValid("YI",  protocol2019Handler.getYIValue());
    printIfValid("YM",  protocol2019Handler.getYMValue());
    printIfValid("YS",  protocol2019Handler.getYSValue());
    printIfValid("YA",  protocol2019Handler.getYAValue());
	printIfValid("YGP",  protocol2019Handler.getYGPValue());

	// Gantry-X
	printIfValid("GX",  protocol2019Handler.getGXValue());
    printIfValid("GXR", protocol2019Handler.getGXRValue());
	printIfValid("GXI", protocol2019Handler.getGXIValue());
	printIfValid("GXM", protocol2019Handler.getGXMValue());
	printIfValid("GXS", protocol2019Handler.getGXSValue());
	printIfValid("GXA", protocol2019Handler.getGXAValue());
	printIfValid("GXHO",protocol2019Handler.getGXHOValue());
	printIfValid("GXGP",protocol2019Handler.getGXGPValue());

	// Gantry-Y
	printIfValid("GY",  protocol2019Handler.getGYValue());
    printIfValid("GYR", protocol2019Handler.getGYRValue());
	printIfValid("GYI", protocol2019Handler.getGYIValue());
	printIfValid("GYM", protocol2019Handler.getGYMValue());
	printIfValid("GYS", protocol2019Handler.getGYSValue());
	printIfValid("GYA", protocol2019Handler.getGYAValue());
	printIfValid("GYGP", protocol2019Handler.getGYGPValue());

	// Gantry-Z
	printIfValid("GZ",  protocol2019Handler.getGZValue());
    printIfValid("GZR", protocol2019Handler.getGZRValue());
	printIfValid("GZI", protocol2019Handler.getGZIValue());
	printIfValid("GZM", protocol2019Handler.getGZMValue());
	printIfValid("GZS", protocol2019Handler.getGZSValue());
	printIfValid("GZA", protocol2019Handler.getGZAValue());
	printIfValid("GZHO",protocol2019Handler.getGZHOValue());
	printIfValid("GZGP",protocol2019Handler.getGZGPValue());
	printIfValid("LDM1",protocol2019Handler.getLDM1Value());

	// Gantry-R
	printIfValid("GR",  protocol2019Handler.getGRValue());
    printIfValid("GRR", protocol2019Handler.getGRRValue());
	printIfValid("GRI", protocol2019Handler.getGRIValue());
	printIfValid("GRM", protocol2019Handler.getGRMValue());
	printIfValid("GRS", protocol2019Handler.getGRSValue());
	printIfValid("GRA", protocol2019Handler.getGRAValue());
	printIfValid("GRGP",protocol2019Handler.getGRGPValue());
	printIfValid("LOAD", protocol2019Handler.getLOADValue());
	printIfValid("RLD", protocol2019Handler.getRLDValue());
	printIfValid("RST", protocol2019Handler.getRSTValue());

	// Stain X
	printIfValid("SX",  protocol2019Handler.getSXValue());
    printIfValid("SXR", protocol2019Handler.getSXRValue());
	printIfValid("SXI", protocol2019Handler.getSXIValue());
	printIfValid("SXS", protocol2019Handler.getSXSValue());
	printIfValid("SXA", protocol2019Handler.getSXAValue());
	printIfValid("SXGP",protocol2019Handler.getSXGPValue());

	// Stain Y
	printIfValid("SY",  protocol2019Handler.getSYValue());
    printIfValid("SYR", protocol2019Handler.getSYRValue());
	printIfValid("SYI", protocol2019Handler.getSYIValue());
	printIfValid("SYS", protocol2019Handler.getSYSValue());
	printIfValid("SYA", protocol2019Handler.getSYAValue());
	printIfValid("SYGP",protocol2019Handler.getSYGPValue());

	// Wash X
	printIfValid("WX",  protocol2019Handler.getWXValue());
    printIfValid("WXR", protocol2019Handler.getWXRValue());
	printIfValid("WXI", protocol2019Handler.getWXIValue());
	printIfValid("WXS", protocol2019Handler.getWXSValue());
	printIfValid("WXA", protocol2019Handler.getWXAValue());
	printIfValid("WXGP",protocol2019Handler.getWXGPValue());

	// Wash Y
	printIfValid("WY",  protocol2019Handler.getWYValue());
    printIfValid("WYR", protocol2019Handler.getWYRValue());
	printIfValid("WYI", protocol2019Handler.getWYIValue());
	printIfValid("WYS", protocol2019Handler.getWYSValue());
	printIfValid("WYA", protocol2019Handler.getWYAValue());
	printIfValid("WYGP",protocol2019Handler.getWYGPValue());

	// Buffer X
	printIfValid("BX",  protocol2019Handler.getBXValue());
    printIfValid("BXR", protocol2019Handler.getBXRValue());
	printIfValid("BXI", protocol2019Handler.getBXIValue());
	printIfValid("BXS", protocol2019Handler.getBXSValue());
	printIfValid("BXA", protocol2019Handler.getBXAValue());
	printIfValid("BXGP",protocol2019Handler.getBXGPValue());

	// Buffer Y
	printIfValid("BY",  protocol2019Handler.getBYValue());
    printIfValid("BYR", protocol2019Handler.getBYRValue());
	printIfValid("BYI", protocol2019Handler.getBYIValue());
	printIfValid("BYS", protocol2019Handler.getBYSValue());
	printIfValid("BYA", protocol2019Handler.getBYAValue());
	printIfValid("BYGP",protocol2019Handler.getBYGPValue());

	// Z Motor
	printIfValid("Z",   protocol2019Handler.getZValue());
    printIfValid("ZR",  protocol2019Handler.getZRValue());
	printIfValid("ZI",  protocol2019Handler.getZIValue());
	printIfValid("ZM",  protocol2019Handler.getZMValue());
	printIfValid("ZS",  protocol2019Handler.getZSValue());
	printIfValid("ZA",  protocol2019Handler.getZAValue());
	printIfValid("ZGP", protocol2019Handler.getZGPValue());
	printIfValid("ZML", protocol2019Handler.getZMLValue());

	// T Motor
	printIfValid("T",   protocol2019Handler.getTValue());
    printIfValid("TR",  protocol2019Handler.getTRValue());
	printIfValid("TI",  protocol2019Handler.getTIValue());
	printIfValid("TM",  protocol2019Handler.getTMValue());
	printIfValid("TS",  protocol2019Handler.getTSValue());
	printIfValid("TA",  protocol2019Handler.getTAValue());
	printIfValid("TGP", protocol2019Handler.getTGPValue());

	// Common
	printIfValid("GI",  protocol2019Handler.getGIValue());
	printIfValid("GM",  protocol2019Handler.getGMValue());
	printIfValid("GS",  protocol2019Handler.getGSValue());
	printIfValid("GA",  protocol2019Handler.getGAValue());

	printIfValid("H",   protocol2019Handler.getHValue());
	printIfValid("HS",  protocol2019Handler.getHSValue());
	printIfValid("SMC", protocol2019Handler.getSMCValue());

	// Read IR Sensor Values
	printIfValid("RIR", protocol2019Handler.getRIRValue());
	printIfValid("PIR", protocol2019Handler.getPIRValue());

	// Read Limit Switch Values
	printIfValid("RLS", protocol2019Handler.getRLSValue());
	printIfValid("PLS", protocol2019Handler.getPLSValue());

	// Magazine Holder IR sensor modules
	printIfValid("MH",  protocol2019Handler.getMHValue());
	printIfValid("MHP2", protocol2019Handler.getMHP2SampleCountValue());
	printIfValid("GXH", protocol2019Handler.getGXHValue());
	printIfValid("GZH", protocol2019Handler.getGZHValue());

	printIfValid("M1L", protocol2019Handler.getM1LValue());
	printIfValid("M1U", protocol2019Handler.getM1UValue());
	printIfValid("M2L", protocol2019Handler.getM2LValue());
	printIfValid("M2U", protocol2019Handler.getM2UValue());

	// Read Accelerometer
	printIfValid("RAM", protocol2019Handler.getRAMValue());
	printIfValid("PAM", protocol2019Handler.getPAMValue());

	// DC Motor
	printIfValid("DCM",  protocol2019Handler.getDCMValue());
	printIfValid("DCM1",  protocol2019Handler.getDCM1Value());
	printIfValid("DCM2",  protocol2019Handler.getDCM2Value());
	printIfValid("DCD",  protocol2019Handler.getDCDValue());
	printIfValid("DCW1",  protocol2019Handler.getDCW1Value());
	printIfValid("DCW2",  protocol2019Handler.getDCW2Value());
	printIfValid("DCS1",  protocol2019Handler.getDCS1Value());
	printIfValid("DCS2",  protocol2019Handler.getDCS2Value());

	// DC Fan
	printIfValid("DCF1",  protocol2019Handler.getDCF1Value());
	printIfValid("DCF2",  protocol2019Handler.getDCF2Value());
	printIfValid("MIXM",  protocol2019Handler.getMIXMValue());
	printIfValid("DCF3",  protocol2019Handler.getDCF3Value());
	printIfValid("DCF4",  protocol2019Handler.getDCF4Value());
	printIfValid("DCF",  protocol2019Handler.getDCFValue());

	if (displayDebug) {
		Serial.print(" ");
	}

	isFirstPrint = true;
}

void Execution2019Handler::performPacketActions()
{
    if(ServiceDiagnostics::locked() && protocol2019Handler.getIDValue()==protocol2019Handler.ignoreValue)return;
	// General Commands
	if (protocol2019Handler.getIDValue() != protocol2019Handler.ignoreValue) {
		Serial.print("ID:" + String(DEVICE_ID) + ",IDHEX:");
		Serial.print(DEVICE_ID, HEX);
		Serial.print(",");
		Serial.print(FWver);
		Serial.print(",");
		Serial.print("BUILD:");
		Serial.print(FWbuildNo);
		Serial.print(",");
		Serial.print("UID:");
		Serial.print(controllerHardwareUid());
		Serial.print(",");
	}
    if(ServiceDiagnostics::locked())return;
	if (protocol2019Handler.getHelpValue() != protocol2019Handler.ignoreValue) {
		displayHelpMenu();
	}
	if (protocol2019Handler.getDebValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDebValue() == 0) {
			displayDebug = false;
		}
		else if (protocol2019Handler.getDebValue() == 1) {
			displayDebug = true;
		}
	}
	if (protocol2019Handler.getAckValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getAckValue() == 0) {
			ackAction = false;
		}
		else if (protocol2019Handler.getAckValue() == 1) {
			ackAction = true;
		}
	}

	#ifdef Master
		#ifdef Stainer_Gantry_PCB
			performGantryCommands();
		#endif
		#ifdef Nozzle_Mount_PCB
			performNozzleMountCommands();
		#endif
		#ifdef Stainer_Master_PCB
			performStainerMasterCommands();
		#endif

	if (protocol2019Handler.getHValue() != protocol2019Handler.ignoreValue) {

	}

	// Read IR Sensor values
	if (protocol2019Handler.getRIRValue() != protocol2019Handler.ignoreValue) {
		int* values = limitSwitch.readIR();
	}
	if (protocol2019Handler.getPIRValue() != protocol2019Handler.ignoreValue) {
#ifdef Nozzle_Mount_PCB
        if (protocol2019Handler.getPIRValue() == 4) {
            printIR = false;
            nzSetIR12Trace(false);
            nzSetIR4Trace(true);
        } else if (protocol2019Handler.getPIRValue() == 12) {
            printIR = false;
            nzSetIR4Trace(false);
            nzSetIR12Trace(true);
        } else if (protocol2019Handler.getPIRValue() == 0 || protocol2019Handler.getPIRValue() == 1) {
            nzSetIR4Trace(false);
            nzSetIR12Trace(false);
        }
#endif
		if (protocol2019Handler.getPIRValue() == 0) {
			printIR = false;
		}
		if (protocol2019Handler.getPIRValue() == 1) {
			printIR = true;
		}
	}

	// Read Limit Switch Values
	if (protocol2019Handler.getRLSValue() != protocol2019Handler.ignoreValue) {
		int* values = limitSwitch.readLimitSwitch();
	}
	if (protocol2019Handler.getPLSValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getPLSValue() == 0) {
			printLimit = false;
		}
		if (protocol2019Handler.getPLSValue() == 1) {
			printLimit = true;
		}
	}
	#endif

	// CMD Printing
	if (isHallSensorCommand()) {

	} else {
		Serial.print(" CMD");
		Serial.println();
	}
	protocol2019Handler.resetValues();
}

void Execution2019Handler::performGantryCommands() {
	#ifdef Stainer_Gantry_PCB
	const long uartTest = protocol2019Handler.getMagazineUartTestValue();
	#if MAGAZINE_UART_TEST
	bool refreshMagazine1 = false;
	bool refreshMagazine2 = false;
	if (uartTest == 1 || uartTest == 3) refreshMagazine1 = runMagazineUartLinkTest(1);
	if (uartTest == 2 || uartTest == 3) refreshMagazine2 = runMagazineUartLinkTest(2);
	if (uartTest != protocol2019Handler.ignoreValue) {
		// A system check needs a fresh slot snapshot even when no slide or
		// magazine state has changed since the previous report. Link-test success
		// therefore restarts background sampling and explicitly queues one status
		// read for each responding holder. serviceMagazineSlideUpdates() publishes
		// the resulting MAGSTAT frames to the ESP32.
		startMagzinechecks = true;
		if (refreshMagazine1) queueSilentMagazineSlideRequest(Magazine1Ir);
		if (refreshMagazine2) queueSilentMagazineSlideRequest(Magazine2Ir);
	}
	#else
	if (uartTest != protocol2019Handler.ignoreValue) {
		Serial.println("UARTTEST UNAVAILABLE: SELECT Stainer_Gantry_PCB_UART");
	}
	#endif
	// Extra Motor
	if (protocol2019Handler.getXValue() != protocol2019Handler.ignoreValue) {
		XMotor.moveTo(protocol2019Handler.getXValue());
		ackManager.requestMovementAck(&XMotor, "X", micros());
	}
	if (protocol2019Handler.getXRValue() != protocol2019Handler.ignoreValue) {
		XMotor.move(protocol2019Handler.getXRValue());
		ackManager.requestMovementAck(&XMotor, "X", micros());
	}
	if (protocol2019Handler.getXIValue() != protocol2019Handler.ignoreValue) {
		XMotor.setRMSCurrentIRUN(protocol2019Handler.getXIValue());
	}
	if (protocol2019Handler.getXMValue() != protocol2019Handler.ignoreValue) {
		XMotor.setMicrosteps(protocol2019Handler.getXMValue());
	}
	if (protocol2019Handler.getXSValue() != protocol2019Handler.ignoreValue) {
		XMotor.setMaxSpeed(protocol2019Handler.getXSValue());
	}
	if (protocol2019Handler.getXAValue() != protocol2019Handler.ignoreValue) {
		XMotor.setAcceleration(protocol2019Handler.getXAValue());
	}
	if (protocol2019Handler.getXGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(XMotor.getCurrentPosition());
	}

	// Gantry-X
	if (protocol2019Handler.getGXValue() != protocol2019Handler.ignoreValue) {
		GXMotor.moveTo(protocol2019Handler.getGXValue());
		ackManager.requestMovementAck(&GXMotor, "GX", micros());
	}
	if (protocol2019Handler.getGXRValue() != protocol2019Handler.ignoreValue) {
		GXMotor.move(protocol2019Handler.getGXRValue());
		ackManager.requestMovementAck(&GXMotor, "GX", micros());
	}
	if (protocol2019Handler.getGXIValue() != protocol2019Handler.ignoreValue) {
		GXMotor.setRMSCurrentIRUN(protocol2019Handler.getGXIValue());
	}
	if (protocol2019Handler.getGXMValue() != protocol2019Handler.ignoreValue) {
		GXMotor.setMicrosteps(protocol2019Handler.getGXMValue());
	}
	if (protocol2019Handler.getGXSValue() != protocol2019Handler.ignoreValue) {
		GXMotor.setMaxSpeed(protocol2019Handler.getGXSValue());
	}
	if (protocol2019Handler.getGXAValue() != protocol2019Handler.ignoreValue) {
		GXMotor.setAcceleration(protocol2019Handler.getGXAValue());
	}
	if (protocol2019Handler.getGXHOValue() != protocol2019Handler.ignoreValue) {
		if (homeGxWithLidarBlocking("GXHO")) {
			Serial.println("GXHO: done");
		}
	}
	if (protocol2019Handler.getGXGPValue() != protocol2019Handler.ignoreValue) {
		float gxPositionMm = 0.0f;
		if (readLidarGxPositionBlocking(gxPositionMm)) {
			Serial.println(gxPositionMm, 1);
		} else {
			Serial.println("GXGP LIDAR ERROR");
		}
	}

	if (protocol2019Handler.getRLDValue() != protocol2019Handler.ignoreValue) {
		float correctedDistanceMm = 0.0f;
		if (readLidarDistanceBlocking(correctedDistanceMm)) {
			Serial.println(correctedDistanceMm, 1);
		} else {
			Serial.println("RLD ERROR");
		}
	}

	// Gantry-Y
	if (protocol2019Handler.getGYValue() != protocol2019Handler.ignoreValue) {
		GYMotor.moveTo(protocol2019Handler.getGYValue());
		ackManager.requestMovementAck(&GYMotor, "GY", micros());
	}
	if (protocol2019Handler.getGYRValue() != protocol2019Handler.ignoreValue) {
		GYMotor.move(protocol2019Handler.getGYRValue());
		ackManager.requestMovementAck(&GYMotor, "GY", micros());
	}
	if (protocol2019Handler.getGYIValue() != protocol2019Handler.ignoreValue) {
		GYMotor.setRMSCurrentIRUN(protocol2019Handler.getGYIValue());
	}
	if (protocol2019Handler.getGYMValue() != protocol2019Handler.ignoreValue) {
		GYMotor.setMicrosteps(protocol2019Handler.getGYMValue());
	}
	if (protocol2019Handler.getGYSValue() != protocol2019Handler.ignoreValue) {
		GYMotor.setMaxSpeed(protocol2019Handler.getGYSValue());
	}
	if (protocol2019Handler.getGYAValue() != protocol2019Handler.ignoreValue) {
		GYMotor.setAcceleration(protocol2019Handler.getGYAValue());
	}
    if (protocol2019Handler.getGYHOValue() != protocol2019Handler.ignoreValue) {
        Serial.println(homeGyIr4Blocking()?"GYHO: done":"GYHO: FAILED; origin unchanged");
    }

	/*if (protocol2019Handler.getGYHOValue() != protocol2019Handler.ignoreValue) {
		// HomeR: rotate R until IR3 (PA5 in gantry PCB) drops below threshold
		Serial.println("GYHO: homing R using IR3");
		const int targetSensorIndex = 3; // IR4 is FOURTH element in irsensorPins[]
		const int homeThreshold = 1000;
		const unsigned long timeoutMs = 30000;
		unsigned long startMs = millis();

		int* irVals = limitSwitch.getSmoothedSensorValues();
		if (irVals == nullptr) {
			Serial.println("GYHO: failed to read IR values");
		} else {
			while (irVals[targetSensorIndex] <= homeThreshold) {
				if (!GYMotor.isMoving()) {
					GYMotor.move(1000); // run in small chunks
				}
				GYMotor.loop();
				limitSwitch.loop();
				irVals = limitSwitch.getSmoothedSensorValues();
				if (millis() - startMs > timeoutMs) {
					Serial.println("GYHO: timeout");
					break;
				}
			}
			GYMotor.stop();
			Serial.println("GYHO: done");
		}
	}*/
	if (protocol2019Handler.getGYGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(GYMotor.getCurrentPosition());
	}

	// Gantry-Z
	if (protocol2019Handler.getGZValue() != protocol2019Handler.ignoreValue) {
		GZMotor.moveTo(protocol2019Handler.getGZValue());
		ackManager.requestMovementAck(&GZMotor, "GZ", micros());
	}
	if (protocol2019Handler.getGZRValue() != protocol2019Handler.ignoreValue) {
		GZMotor.move(protocol2019Handler.getGZRValue());
		ackManager.requestMovementAck(&GZMotor, "GZ", micros());
	}
	if (protocol2019Handler.getGZIValue() != protocol2019Handler.ignoreValue) {
		GZMotor.setRMSCurrentIRUN(protocol2019Handler.getGZIValue());
	}
	if (protocol2019Handler.getGZMValue() != protocol2019Handler.ignoreValue) {
		GZMotor.setMicrosteps(protocol2019Handler.getGZMValue());
	}
	if (protocol2019Handler.getGZSValue() != protocol2019Handler.ignoreValue) {
		GZMotor.setMaxSpeed(protocol2019Handler.getGZSValue());
	}
	if (protocol2019Handler.getGZAValue() != protocol2019Handler.ignoreValue) {
		GZMotor.setAcceleration(protocol2019Handler.getGZAValue());
	}
	if (protocol2019Handler.getGZHOValue() != protocol2019Handler.ignoreValue) {
		if (homeGzWithHallBlocking("GZHO")) {
			Serial.println("GZHO: done");
		}
	}
	if (protocol2019Handler.getGZGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(GZMotor.getCurrentPosition());
	}

	const bool hasMagazineBusCommand =
		protocol2019Handler.getLDM1Value() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getLOADValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getMHP2SampleCountValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getMH2SampleCountValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getMHValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getM1LValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getM1UValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getM2LValue() != protocol2019Handler.ignoreValue ||
		protocol2019Handler.getM2UValue() != protocol2019Handler.ignoreValue;
	if (hasMagazineBusCommand) {
		clearMagazineBusState();
	}

	if (protocol2019Handler.getLDM1Value() != protocol2019Handler.ignoreValue) {
		long rawVal = protocol2019Handler.getLDM1Value();
		int magazine = 0;
		int slideNumber = 0;

		// Parse LDM<Mag><Slide> format
		if (rawVal >= 100) {
			magazine = rawVal / 100;
			slideNumber = rawVal % 100;
		} else {
			magazine = rawVal / 10;
			slideNumber = rawVal % 10;
		}

		(void)executeMagazineLoadBlocking(magazine, slideNumber);
	}

	if (protocol2019Handler.getLOADValue() != protocol2019Handler.ignoreValue) {
		// Retry detection up to 3 times — magazine PCB / I2C bus can be in a stale state after idle.
		// First attempt typically wakes the bus, second succeeds.
		int mag1Type = 0, mag2Type = 0;
		for (int attempt = 0; attempt < 3 && mag1Type == 0 && mag2Type == 0; attempt++) {
			clearMagazineBusState();

			if (requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
				if (requestFullMagazineIrValuesBlocking(Magazine1Ir, i2c1)) {
					mag1Type = detectMagazineType(Magazine1Ir);
				}
				requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
			}
			clearMagazineBusState();
			if (requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
				if (requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2)) {
					mag2Type = detectMagazineType(Magazine2Ir);
				}
				requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
			}

			if (mag1Type == 0 && mag2Type == 0 && attempt < 2) {
				delay(50); // brief pause before retry
			}
		}

		if (mag1Type == 0 && mag2Type == 0) {
			Serial.println("LOAD Error: No magazine detected in any holder");
		} else {
			autoLoadMagazine = (mag1Type != 0) ? 1 : 2;
			autoLoadState    = AutoLoadState::WaitIr2Clear;
			autoLoadStopRequested = false;
			autoLoadActive   = true;
			moveGantryToLoadPositionBlocking(false);
		}
	}

	// Gantry-R
	if (protocol2019Handler.getGRValue() != protocol2019Handler.ignoreValue) {
		GRMotor.moveTo(protocol2019Handler.getGRValue());
		ackManager.requestMovementAck(&GRMotor, "GR", micros());
	}
	if (protocol2019Handler.getGRRValue() != protocol2019Handler.ignoreValue) {
		GRMotor.move(protocol2019Handler.getGRRValue());
		ackManager.requestMovementAck(&GRMotor, "GR", micros());
	}
	if (protocol2019Handler.getGRIValue() != protocol2019Handler.ignoreValue) {
		GRMotor.setRMSCurrentIRUN(protocol2019Handler.getGRIValue());
	}
	if (protocol2019Handler.getGRMValue() != protocol2019Handler.ignoreValue) {
		GRMotor.setMicrosteps(protocol2019Handler.getGRMValue());
	}
	if (protocol2019Handler.getGRSValue() != protocol2019Handler.ignoreValue) {
		GRMotor.setMaxSpeed(protocol2019Handler.getGRSValue());
	}
	if (protocol2019Handler.getGRAValue() != protocol2019Handler.ignoreValue) {
		GRMotor.setAcceleration(protocol2019Handler.getGRAValue());
	}
	if (protocol2019Handler.getGRHOValue() != protocol2019Handler.ignoreValue) {
		Serial.println("GRHO: homing R using IR3");
		(void)homeGrWithIr3Blocking("GRHO", true);
	}

	if (protocol2019Handler.getRSTValue() != protocol2019Handler.ignoreValue) {
		(void)executeRstBlocking();
	}

	// GHOME: Home all gantry axes
	if (protocol2019Handler.getGHOMEValue() != protocol2019Handler.ignoreValue) {
		(void)runGhomeBlocking(true);
	}

	// GLOAD: Move all gantry axes to load position
	if (protocol2019Handler.getGLOADValue() != protocol2019Handler.ignoreValue) {
		moveGantryToLoadPositionBlocking(true);
	}

	/*if (protocol2019Handler.getGRLDValue() != protocol2019Handler.ignoreValue) {
		Serial.println("GRLD: executing sequence");
		GRMotor.move(12800);
		ackManager.requestMovementAck(&GRMotor, "GR", micros());
		while (GRMotor.isMoving()) { GRMotor.loop(); limitSwitch.loop(); }

		// Increase GY speed to maximum for the sequence
		long originalGYSpeed = GYMotor.getMaxSpeed();
		GYMotor.setMaxSpeed(100000);

		GYMotor.move(-463500);
		ackManager.requestMovementAck(&GYMotor, "GY", micros());
		while (GYMotor.isMoving()) { GYMotor.loop(); limitSwitch.loop(); }

		GYMotor.move(463500);
		ackManager.requestMovementAck(&GYMotor, "GY", micros());
		while (GYMotor.isMoving()) { GYMotor.loop(); limitSwitch.loop(); }

		// Restore original GY speed
		GYMotor.setMaxSpeed(originalGYSpeed);

		GRMotor.move(12800);
		ackManager.requestMovementAck(&GRMotor, "GR", micros());
		while (GRMotor.isMoving()) { GRMotor.loop(); limitSwitch.loop(); }

		Serial.println("LOADED");
	}*/

	if (protocol2019Handler.getGRGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(GRMotor.getCurrentPosition());
	}

	// TMC Stepper Common Commands
	if (protocol2019Handler.getGIValue() != protocol2019Handler.ignoreValue) {
		Serial.print("RMS Irun Currents -");
		Serial.print(" GX: "); Serial.print(GXMotor.getRMSCurrentIRUN());
		Serial.print(", GY: "); Serial.print(GYMotor.getRMSCurrentIRUN());
		Serial.print(", GZ: "); Serial.print(GZMotor.getRMSCurrentIRUN());
		Serial.print(", GR: "); Serial.print(GRMotor.getRMSCurrentIRUN());
	}
	if (protocol2019Handler.getGMValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Microsteps -");
		Serial.print(" GX: "); Serial.print(GXMotor.getMicrosteps());
		Serial.print(", GY: "); Serial.print(GYMotor.getMicrosteps());
		Serial.print(", GZ: "); Serial.print(GZMotor.getMicrosteps());
		Serial.print(", GR: "); Serial.print(GRMotor.getMicrosteps());
	}
	if (protocol2019Handler.getGSValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Max Speed -");
		Serial.print(" GX: "); Serial.print(GXMotor.getMaxSpeed());
		Serial.print(", GY: "); Serial.print(GYMotor.getMaxSpeed());
		Serial.print(", GZ: "); Serial.print(GZMotor.getMaxSpeed());
		Serial.print(", GR: "); Serial.print(GRMotor.getMaxSpeed());
	}
	if (protocol2019Handler.getGAValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Acceleration -");
		Serial.print(" GX: "); Serial.print(GXMotor.getAcceleration());
		Serial.print(", GY: "); Serial.print(GYMotor.getAcceleration());
		Serial.print(", GZ: "); Serial.print(GZMotor.getAcceleration());
		Serial.print(", GR: "); Serial.print(GRMotor.getAcceleration());
	}

	// I2Cs
	if (protocol2019Handler.getHSValue() != protocol2019Handler.ignoreValue) {
		// I2C1_1 connection
		if (protocol2019Handler.getHSValue() == 111) {
			magazine1IrModule.requestIrValues();
		}
		if (protocol2019Handler.getHSValue() == 112) {
			magazine1IrModule.requestFilteredIrValues();
		}
		if (protocol2019Handler.getHSValue() == 113) {
			Serial.println("Magazine 1 IR holder has no position channel");
		}

		// I2C2_1 connection
		if (protocol2019Handler.getHSValue() == 211) {
			magazine2IrModule.requestIrValues();
		}
		if (protocol2019Handler.getHSValue() == 212) {
			magazine2IrModule.requestFilteredIrValues();
		}
		if (protocol2019Handler.getHSValue() == 213) {
			Serial.println("Magazine 2 IR holder has no position channel");
		}
	}
	if (protocol2019Handler.getSMCValue() != protocol2019Handler.ignoreValue) {
		// Continuous magazine polling is intentionally disabled. Holders update
		// LEDs locally; use explicit holder commands when Gantry needs fresh data.
		startMagzinechecks = false;
		Serial.println("SMC: continuous magazine polling disabled; use on-demand holder reads");
	}

	// Magazine Calibration
	if (protocol2019Handler.getCAL1Value() != protocol2019Handler.ignoreValue) {
		clearMagazineBusState();
		if (requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
			requestFullMagazineIrValuesBlocking(Magazine1Ir, i2c1);
			requestMagazineTriggerCommandBlocking(i2c1, Magazine1Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
		}
		const long* h1 = Magazine1Ir.getIrValues();
		Serial.print("CAL1: S21="); Serial.print(h1[20]);
		Serial.print(" S22="); Serial.print(h1[21]);
		Serial.print(" S23="); Serial.print(h1[22]);
		Serial.print(" MagType="); Serial.println(detectMagazineType(Magazine1Ir));
		const int MAX_CAL_ATTEMPTS = 5;
		bool calSuccess = false;
		for (int attempt = 1; attempt <= MAX_CAL_ATTEMPTS && !calSuccess; attempt++) {
			Serial.print("CAL1: Attempt "); Serial.println(attempt);
			calSuccess = runMagazineCalibrationBlocking(1);
		}
		if (!calSuccess) {
			Serial.println("CAL1: REINSERT MAGAZINE AND TRY AGAIN");
		}
	}
	if (protocol2019Handler.getCAL2Value() != protocol2019Handler.ignoreValue) {
		clearMagazineBusState();
		if (requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_ON)) {
			requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2);
			requestMagazineTriggerCommandBlocking(i2c2, Magazine2Ir.getPcbID(), CMD_MAGAZINE_TRIGGER_OFF);
		}
		const long* h2 = Magazine2Ir.getIrValues();
		Serial.print("CAL2: S21="); Serial.print(h2[20]);
		Serial.print(" S22="); Serial.print(h2[21]);
		Serial.print(" S23="); Serial.print(h2[22]);
		Serial.print(" MagType="); Serial.println(detectMagazineType(Magazine2Ir));
		const int MAX_CAL_ATTEMPTS = 5;
		bool calSuccess = false;
		for (int attempt = 1; attempt <= MAX_CAL_ATTEMPTS && !calSuccess; attempt++) {
			Serial.print("CAL2: Attempt "); Serial.println(attempt);
			calSuccess = runMagazineCalibrationBlocking(2);
		}
		if (!calSuccess) {
			Serial.println("CAL2: REINSERT MAGAZINE AND TRY AGAIN");
		}
	}

	// Magazine Holder IR sensor modules
	if (protocol2019Handler.getMHP2SampleCountValue() != protocol2019Handler.ignoreValue) {
		startMagazine2PulseMonitor(protocol2019Handler.getMHP2SampleCountValue());
	}
	if (protocol2019Handler.getMH2SampleCountValue() != protocol2019Handler.ignoreValue) {
		long sampleCount = protocol2019Handler.getMH2SampleCountValue();
		if (sampleCount > 0) {
			for (long sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
				if (!requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2)) {
					Serial.println("MH2 TIMEOUT");
					break;
				}
				printIrValuesCsvLine(sampleIndex + 1, Magazine2Ir.getIrValues(), 24);
			}
		}
	}
	if (protocol2019Handler.getMHValue() != protocol2019Handler.ignoreValue) {
		// Magzine1
		if (protocol2019Handler.getMHValue() == 1) {
			Magazine1Ir.requestAllIrValues();
		}
		if (protocol2019Handler.getMHValue() == 111) {
			Magazine1Ir.requestHalfIrValues(1);
		}
		if (protocol2019Handler.getMHValue() == 112) {
			Magazine1Ir.requestHalfIrValues(2);
		}
		if (protocol2019Handler.getMHValue() == 121) {
			Magazine1Ir.requestHalfFilteredIrValues(1);
		}
		if (protocol2019Handler.getMHValue() == 122) {
			Magazine1Ir.requestHalfFilteredIrValues(2);
		}
		if (protocol2019Handler.getMHValue() == 14) {
			Magazine1Ir.requestSlideStatus();
		}
		if (protocol2019Handler.getMHValue() == 15) {
			Magazine1Ir.requestMagzineStatus();
		}
		if (protocol2019Handler.getMHValue() == 16) {
			Magazine1Ir.lockMagzine();
		}
		if (protocol2019Handler.getMHValue() == 17) {
			Magazine1Ir.unlockMagzine();
		}
		// Magzine2
		if (protocol2019Handler.getMHValue() == 2) {
			if (!requestFullMagazineIrValuesBlocking(Magazine2Ir, i2c2)) {
				Serial.println("MH2 TIMEOUT");
			} else {
				printIrValuesLine(Magazine2Ir.getIrValues(), 24);
			}
		}
		if (protocol2019Handler.getMHValue() == 211) {
			Magazine2Ir.requestHalfIrValues(1);
		}
		if (protocol2019Handler.getMHValue() == 212) {
			Magazine2Ir.requestHalfIrValues(2);
		}
		if (protocol2019Handler.getMHValue() == 221) {
			Magazine2Ir.requestHalfFilteredIrValues(1);
		}
		if (protocol2019Handler.getMHValue() == 222) {
			Magazine2Ir.requestHalfFilteredIrValues(2);
		}
		if (protocol2019Handler.getMHValue() == 24) {
			Magazine2Ir.requestSlideStatus();
		}
		if (protocol2019Handler.getMHValue() == 25) {
			Magazine2Ir.requestMagzineStatus();
		}
		if (protocol2019Handler.getMHValue() == 26) {
			Magazine2Ir.lockMagzine();
		}
		if (protocol2019Handler.getMHValue() == 27) {
			Magazine2Ir.unlockMagzine();
		}
	}

	// Read Accelerometer
	if (protocol2019Handler.getRAMValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getRAMValue() == 1) {
			int16_t* values = acc1.readAcceldata();
		}
		if (protocol2019Handler.getRAMValue() == 2) {
			#if MAGAZINE_UART_TEST
			Serial.println("RAM2 UNAVAILABLE: AM2 PB10/PB11 ARE MAGAZINE 2 UART");
			#else
			int16_t* values = acc2.readAcceldata();
			(void)values;
			#endif
		}
	}
	if (protocol2019Handler.getPAMValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getPAMValue() == 0) {
			acc1.readAccelContinuous(false);
			#if !MAGAZINE_UART_TEST
			acc2.readAccelContinuous(false);
			#endif
		}
		if (protocol2019Handler.getPAMValue() == 1) {
			acc1.readAccelContinuous(true);
		}
		if (protocol2019Handler.getPAMValue() == 2) {
			#if MAGAZINE_UART_TEST
			Serial.println("PAM2 UNAVAILABLE: AM2 PB10/PB11 ARE MAGAZINE 2 UART");
			#else
			acc2.readAccelContinuous(true);
			#endif
		}
	}

	// Solenoid Commands
	if (protocol2019Handler.getM1LValue() != protocol2019Handler.ignoreValue) {
		Magazine1Ir.lockMagzine();
		delay(500);
	}
	if (protocol2019Handler.getM1UValue() != protocol2019Handler.ignoreValue) {
		Magazine1Ir.unlockMagzine();
		delay(500);
	}
	if (protocol2019Handler.getM2LValue() != protocol2019Handler.ignoreValue) {
		Magazine2Ir.lockMagzine();
		delay(500);
	}
	if (protocol2019Handler.getM2UValue() != protocol2019Handler.ignoreValue) {
		Magazine2Ir.unlockMagzine();
		delay(500);
	}
	#endif
}

void Execution2019Handler::performNozzleMountCommands() {
	#ifdef Nozzle_Mount_PCB
    if (protocol2019Handler.getParaCheckValue() != protocol2019Handler.ignoreValue) {
        nzParaActive = protocol2019Handler.getParaCheckValue() == 1;
        nzParaAnchored = nzParaStainSeen = nzParaWashSeen = false;
        nzParaFanEnded = nzParaIdleSent = false;
        if (nzParaActive) {
            if (nzBedDepth() || nzBedEnabled) {
                nzParaActive = false;
                MasterSerial.println("PCE REJECT_BUSY");
            } else {
                nzBedFilters[0].reset();
                MasterSerial.println("PCE ARMED");
            }
        }
        return;
    }
	if (protocol2019Handler.getRDHTValue() != protocol2019Handler.ignoreValue) {
		reportNozzleDHT(Serial);
	}
	// NZHO: Home all nozzle mount axes
	if (protocol2019Handler.getNZHOValue() != protocol2019Handler.ignoreValue) {
		(void)runNzhoBlocking();
	}
	// Stain X
	if (protocol2019Handler.getSXValue() != protocol2019Handler.ignoreValue) {
		SXMotor.moveTo(protocol2019Handler.getSXValue());
		ackManager.requestMovementAck(&SXMotor, "SX", micros());
	}
	if (protocol2019Handler.getSXRValue() != protocol2019Handler.ignoreValue) {
		SXMotor.move(protocol2019Handler.getSXRValue());
		ackManager.requestMovementAck(&SXMotor, "SX", micros());
	}
	if (protocol2019Handler.getSXIValue() != protocol2019Handler.ignoreValue) {
		SXMotor.setRMSCurrentIRUN(protocol2019Handler.getSXIValue());
	}
	if (protocol2019Handler.getSXSValue() != protocol2019Handler.ignoreValue) {
		SXMotor.setMaxSpeed(protocol2019Handler.getSXSValue());
	}
	if (protocol2019Handler.getSXAValue() != protocol2019Handler.ignoreValue) {
		SXMotor.setAcceleration(protocol2019Handler.getSXAValue());
	}
	if (protocol2019Handler.getSXGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(SXMotor.getCurrentPosition());
	}
	// Stain Y
	if (protocol2019Handler.getSYValue() != protocol2019Handler.ignoreValue) {
		SYMotor.moveTo(protocol2019Handler.getSYValue());
		ackManager.requestMovementAck(&SYMotor, "SY", micros());
	}
	if (protocol2019Handler.getSYRValue() != protocol2019Handler.ignoreValue) {
		SYMotor.move(protocol2019Handler.getSYRValue());
		ackManager.requestMovementAck(&SYMotor, "SY", micros());
	}
	if (protocol2019Handler.getSYIValue() != protocol2019Handler.ignoreValue) {
		SYMotor.setRMSCurrentIRUN(protocol2019Handler.getSYIValue());
	}
	if (protocol2019Handler.getSYSValue() != protocol2019Handler.ignoreValue) {
		SYMotor.setMaxSpeed(protocol2019Handler.getSYSValue());
	}
	if (protocol2019Handler.getSYAValue() != protocol2019Handler.ignoreValue) {
		SYMotor.setAcceleration(protocol2019Handler.getSYAValue());
	}
	if (protocol2019Handler.getSYGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(SYMotor.getCurrentPosition());
	}
	// Wash X
	if (protocol2019Handler.getWXValue() != protocol2019Handler.ignoreValue) {
		WXMotor.moveTo(protocol2019Handler.getWXValue());
		ackManager.requestMovementAck(&WXMotor, "WX", micros());
	}
	if (protocol2019Handler.getWXRValue() != protocol2019Handler.ignoreValue) {
		WXMotor.move(protocol2019Handler.getWXRValue());
		ackManager.requestMovementAck(&WXMotor, "WX", micros());
	}
	if (protocol2019Handler.getWXIValue() != protocol2019Handler.ignoreValue) {
		WXMotor.setRMSCurrentIRUN(protocol2019Handler.getWXIValue());
	}
	if (protocol2019Handler.getWXSValue() != protocol2019Handler.ignoreValue) {
		WXMotor.setMaxSpeed(protocol2019Handler.getWXSValue());
	}
	if (protocol2019Handler.getWXAValue() != protocol2019Handler.ignoreValue) {
		WXMotor.setAcceleration(protocol2019Handler.getWXAValue());
	}
	if (protocol2019Handler.getWXGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(WXMotor.getCurrentPosition());
	}
	// Wash Y
	if (protocol2019Handler.getWYValue() != protocol2019Handler.ignoreValue) {
		WYMotor.moveTo(protocol2019Handler.getWYValue());
		ackManager.requestMovementAck(&WYMotor, "WY", micros());
	}
	if (protocol2019Handler.getWYRValue() != protocol2019Handler.ignoreValue) {
		WYMotor.move(protocol2019Handler.getWYRValue());
		ackManager.requestMovementAck(&WYMotor, "WY", micros());
	}
	if (protocol2019Handler.getWYIValue() != protocol2019Handler.ignoreValue) {
		WYMotor.setRMSCurrentIRUN(protocol2019Handler.getWYIValue());
	}
	if (protocol2019Handler.getWYSValue() != protocol2019Handler.ignoreValue) {
		WYMotor.setMaxSpeed(protocol2019Handler.getWYSValue());
	}
	if (protocol2019Handler.getWYAValue() != protocol2019Handler.ignoreValue) {
		WYMotor.setAcceleration(protocol2019Handler.getWYAValue());
	}
	if (protocol2019Handler.getWYGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(WYMotor.getCurrentPosition());
	}
	// Buffer X
	if (protocol2019Handler.getBXValue() != protocol2019Handler.ignoreValue) {
		BXMotor.moveTo(protocol2019Handler.getBXValue());
		ackManager.requestMovementAck(&BXMotor, "BX", micros());
	}
	if (protocol2019Handler.getBXRValue() != protocol2019Handler.ignoreValue) {
		BXMotor.move(protocol2019Handler.getBXRValue());
		ackManager.requestMovementAck(&BXMotor, "BX", micros());
	}
	if (protocol2019Handler.getBXIValue() != protocol2019Handler.ignoreValue) {
		BXMotor.setRMSCurrentIRUN(protocol2019Handler.getBXIValue());
	}
	if (protocol2019Handler.getBXSValue() != protocol2019Handler.ignoreValue) {
		BXMotor.setMaxSpeed(protocol2019Handler.getBXSValue());
	}
	if (protocol2019Handler.getBXAValue() != protocol2019Handler.ignoreValue) {
		BXMotor.setAcceleration(protocol2019Handler.getBXAValue());
	}
	if (protocol2019Handler.getBXGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(BXMotor.getCurrentPosition());
	}
	// Buffer Y
	if (protocol2019Handler.getBYValue() != protocol2019Handler.ignoreValue) {
		BYMotor.moveTo(protocol2019Handler.getBYValue());
		ackManager.requestMovementAck(&BYMotor, "BY", micros());
	}
	if (protocol2019Handler.getBYRValue() != protocol2019Handler.ignoreValue) {
		BYMotor.move(protocol2019Handler.getBYRValue());
		ackManager.requestMovementAck(&BYMotor, "BY", micros());
	}
	if (protocol2019Handler.getBYIValue() != protocol2019Handler.ignoreValue) {
		BYMotor.setRMSCurrentIRUN(protocol2019Handler.getBYIValue());
	}
	if (protocol2019Handler.getBYSValue() != protocol2019Handler.ignoreValue) {
		BYMotor.setMaxSpeed(protocol2019Handler.getBYSValue());
	}
	if (protocol2019Handler.getBYAValue() != protocol2019Handler.ignoreValue) {
		BYMotor.setAcceleration(protocol2019Handler.getBYAValue());
	}
	if (protocol2019Handler.getBYGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(BYMotor.getCurrentPosition());
	}
	// Individual homing commands
	if (protocol2019Handler.getSXHOValue() != protocol2019Handler.ignoreValue) {
		if (!homeSX()) Serial.println("SXHO: failed");
		else Serial.println("SXHO: done");
	}
	if (protocol2019Handler.getSYHOValue() != protocol2019Handler.ignoreValue) {
		if (!homeSY()) Serial.println("SYHO: failed");
		else Serial.println("SYHO: done");
	}
	if (protocol2019Handler.getWXHOValue() != protocol2019Handler.ignoreValue) {
		if (!homeWX()) Serial.println("WXHO: failed");
		else Serial.println("WXHO: done");
	}
	if (protocol2019Handler.getWYHOValue() != protocol2019Handler.ignoreValue) {
		if (!homeWY()) Serial.println("WYHO: failed");
		else Serial.println("WYHO: done");
	}
	if (protocol2019Handler.getBXHOValue() != protocol2019Handler.ignoreValue) {
		if (!homeBX()) Serial.println("BXHO: failed");
		else Serial.println("BXHO: done");
	}
	// TMC Stepper Common Commands
	if (protocol2019Handler.getGIValue() != protocol2019Handler.ignoreValue) {
		Serial.print("RMS Irun Currents -");
		Serial.print(" SX: "); Serial.print(SXMotor.getRMSCurrentIRUN());
		Serial.print(", SY: "); Serial.print(SYMotor.getRMSCurrentIRUN());
		Serial.print(", WX: "); Serial.print(WXMotor.getRMSCurrentIRUN());
		Serial.print(", WY: "); Serial.print(WYMotor.getRMSCurrentIRUN());
		Serial.print(", BX: "); Serial.print(BXMotor.getRMSCurrentIRUN());
		Serial.print(", BY: "); Serial.print(BYMotor.getRMSCurrentIRUN());
	}
	if (protocol2019Handler.getGSValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Max Speed -");
		Serial.print(" SX: "); Serial.print(SXMotor.getMaxSpeed());
		Serial.print(", SY: "); Serial.print(SYMotor.getMaxSpeed());
		Serial.print(", WX: "); Serial.print(WXMotor.getMaxSpeed());
		Serial.print(", WY: "); Serial.print(WYMotor.getMaxSpeed());
		Serial.print(", BX: "); Serial.print(BXMotor.getMaxSpeed());
		Serial.print(", BY: "); Serial.print(BYMotor.getMaxSpeed());
	}
	if (protocol2019Handler.getGAValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Acceleration -");
		Serial.print(" SX: "); Serial.print(SXMotor.getAcceleration());
		Serial.print(", SY: "); Serial.print(SYMotor.getAcceleration());
		Serial.print(", WX: "); Serial.print(WXMotor.getAcceleration());
		Serial.print(", WY: "); Serial.print(WYMotor.getAcceleration());
		Serial.print(", BX: "); Serial.print(BXMotor.getAcceleration());
		Serial.print(", BY: "); Serial.print(BYMotor.getAcceleration());
	}

	// DC Motor
	if (protocol2019Handler.getDCMValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCMValue() == 0) {
			dcMotor.stopMotor();
		}
		else if ((protocol2019Handler.getDCMValue() > 1) && (protocol2019Handler.getDCMValue() < 256)){
			dcMotor.runMotor(protocol2019Handler.getDCMValue());
		}
	}

	// FEED — scale nozzle automation delays to match feed speed
	if (protocol2019Handler.getFEEDValue() != protocol2019Handler.ignoreValue) {
        if(nzBedDepth() && (unsigned long)protocol2019Handler.getFEEDValue()!=currentSph)
            nzBedFail(90);
        else nzUpdateDelaysForFeedSpeed(protocol2019Handler.getFEEDValue());
	}

    if(protocol2019Handler.getBEDTESTValue()!=protocol2019Handler.ignoreValue)
        Serial.println("BEDTEST retired: production uses actual bed STEP positions");

	// DC Fan
	if (protocol2019Handler.getDCF1Value() != protocol2019Handler.ignoreValue) {
		// Legacy fan commands must not latch humidity control into manual mode.
		Serial.println("DCF1: automatic humidity control; manual value ignored");
	}
	if (protocol2019Handler.getDCF2Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCF2Value() == 0) {
			dcFan2.stopFan();
		}
		else if ((protocol2019Handler.getDCF2Value() > 1) && (protocol2019Handler.getDCF2Value() < 256)){
			dcFan2.runFan(protocol2019Handler.getDCF2Value());
		}
	}
	if (protocol2019Handler.getDCF3Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCF3Value() == 0) {
			dcFan3.stopHighSpeedFan();
		}
		else if ((protocol2019Handler.getDCF3Value() > 1) && (protocol2019Handler.getDCF3Value() <= 100)){
			dcFan3.runHighSpeedFanPercent(protocol2019Handler.getDCF3Value());
		}
	}
	if (protocol2019Handler.getDCF4Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCF4Value() == 0) {
			dcFan4.stopHighSpeedFan();
		}
		else if ((protocol2019Handler.getDCF4Value() > 1) && (protocol2019Handler.getDCF4Value() <= 100)){
			dcFan4.runHighSpeedFanPercent(protocol2019Handler.getDCF4Value());
		}
	}
	if (protocol2019Handler.getDCFValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCFValue() == 3) {
			Serial.println(dcFan3.getHighSpeedFanSpeed());
		}
		if (protocol2019Handler.getDCFValue() == 4) {
			Serial.println(dcFan4.getHighSpeedFanSpeed());
		}
	}

	// PROFILE — pushed from master to sync the nozzle's per-recipe delays.
	// Same parser semantics as master (accepts numeric ID or RP/LM/MG/WG code).
	// Nozzle just applies; master is authoritative and already rejected mid-run.
	if (protocol2019Handler.getPROFILEValue() != protocol2019Handler.ignoreValue) {
		long pv = protocol2019Handler.getPROFILEValue();
		if (pv >= 0 && pv < PROFILE_COUNT) {
            if(nzBedDepth() && (ProfileId)pv!=nzActiveProfileId)nzBedFail(91);
            else nzApplyProfile((ProfileId)pv);
		} else if (pv == -1) {
			// Query — print current profile + scaled timings at the active SPH
			const StainProfile& p = nzProfile();
			const ProfileId pid = nzActiveProfileId;
			Serial.print("[nz PROFILE] current="); Serial.print(p.code);
			Serial.print(" sph=");                 Serial.print(currentSph);
			Serial.print(" bed-steps M14=");  Serial.print(timingStepsFor(pid, TK_M14_AT));
			Serial.print(" S1=");                  Serial.print(timingStepsFor(pid, TK_S1_AT));
			Serial.print(" B1=");                  Serial.print(timingStepsFor(pid, TK_B1_AT));
			Serial.print(" S2MIX=");               Serial.println(timingStepsFor(pid, TK_S2MIX_AT));
		}
	}
	#endif
}

void Execution2019Handler::performStainerMasterCommands() {
	#ifdef Stainer_Master_PCB
    if (protocol2019Handler.getParaCheckValue() != protocol2019Handler.ignoreValue) {
        if (protocol2019Handler.getParaCheckValue() == 0) {
            pcFinish("USER_STOP_BED_AND_AUTOMATION_CONTINUE");
        } else if (paraCheck.active) {
            Serial.println("[PARA] capture already active; use PARA STOP before re-arming");
        } else {
            pcConfigurationPending = true;
            pcProfileSelected = pcFeedSelected = false;
            Serial.println("[PARA] waiting for your PROFILE and FEED selections; existing bed motion continues");
            Serial.println("[PARA] example: M; PROFILE WG then M; FEED 60");
        }
        return;
    }
    if ((paraCheck.active || pcConfigurationPending) &&
        (protocol2019Handler.getBEDTESTValue() != protocol2019Handler.ignoreValue ||
         protocol2019Handler.getBTRESETValue() != protocol2019Handler.ignoreValue)) {
        Serial.println("[PARA] BEDTEST/BTRESET rejected; use PARA STOP first");
        return;
    }
	if (paraCheck.active && (protocol2019Handler.getFEEDValue() != protocol2019Handler.ignoreValue ||
        protocol2019Handler.getPROFILEValue() != protocol2019Handler.ignoreValue ||
        protocol2019Handler.getBEDTESTValue() != protocol2019Handler.ignoreValue ||
        protocol2019Handler.getBTRESETValue() != protocol2019Handler.ignoreValue)) {
        Serial.println("[PARA] PROFILE/FEED/BEDTEST/BTRESET change rejected during capture; use PARA STOP first");
        return;
    }
	// X Motor
	if (protocol2019Handler.getXValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("X: BLOCKED by REAGENTLOCK");
		else {
			XMotor.moveTo(protocol2019Handler.getXValue());
			ackManager.requestMovementAck(&XMotor, "X", micros());
		}
	}
	if (protocol2019Handler.getXRValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("XR: BLOCKED by REAGENTLOCK");
		else {
			XMotor.move(protocol2019Handler.getXRValue());
			ackManager.requestMovementAck(&XMotor, "X", micros());
		}
	}
	if (protocol2019Handler.getXIValue() != protocol2019Handler.ignoreValue) {
		XMotor.setRMSCurrentIRUN(protocol2019Handler.getXIValue());
	}
	if (protocol2019Handler.getXMValue() != protocol2019Handler.ignoreValue) {
		XMotor.setMicrosteps(protocol2019Handler.getXMValue());
	}
	if (protocol2019Handler.getXSValue() != protocol2019Handler.ignoreValue) {
		XMotor.setMaxSpeed(protocol2019Handler.getXSValue());
	}
	if (protocol2019Handler.getXAValue() != protocol2019Handler.ignoreValue) {
		XMotor.setAcceleration(protocol2019Handler.getXAValue());
	}
	if (protocol2019Handler.getXGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(XMotor.getCurrentPosition());
	}
	// XML <µL> — dispense via X (S2 pump). mL × 1000 (µL).
	if (protocol2019Handler.getXMLValue() != protocol2019Handler.ignoreValue && !reagentDispenseLocked) {
		const long uL    = protocol2019Handler.getXMLValue();
		const float ml   = (float)uL / 1000.0f;
		const long steps = (long)((ml * (float)X_STEPS_PER_REV) / X_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
		XMotor.setMaxSpeed(X_DISPENSE_SPEED);
		XMotor.setAcceleration(X_DISPENSE_ACCEL);
		XMotor.move(steps);
		ackManager.requestMovementAck(&XMotor, "X", micros());
		Serial.print("XML: ");  Serial.print(ml, 3);
		Serial.print(" mL @ "); Serial.print(X_FLOW_ML_PER_REV, 3);
		Serial.print(" mL/rev = "); Serial.print(steps); Serial.println(" steps");
	} else if (protocol2019Handler.getXMLValue() != protocol2019Handler.ignoreValue) {
		Serial.println("XML: BLOCKED by REAGENTLOCK");
	}
	// Y Motor
	if (protocol2019Handler.getYValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("Y: BLOCKED by REAGENTLOCK");
		else {
			YMotor.moveTo(protocol2019Handler.getYValue());
			ackManager.requestMovementAck(&YMotor, "Y", micros());
		}
	}
	if (protocol2019Handler.getYRValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("YR: BLOCKED by REAGENTLOCK");
		else {
			YMotor.move(protocol2019Handler.getYRValue());
			ackManager.requestMovementAck(&YMotor, "Y", micros());
		}
	}
	if (protocol2019Handler.getYIValue() != protocol2019Handler.ignoreValue) {
		YMotor.setRMSCurrentIRUN(protocol2019Handler.getYIValue());
	}
	if (protocol2019Handler.getYMValue() != protocol2019Handler.ignoreValue) {
		YMotor.setMicrosteps(protocol2019Handler.getYMValue());
	}
	if (protocol2019Handler.getYSValue() != protocol2019Handler.ignoreValue) {
		YMotor.setMaxSpeed(protocol2019Handler.getYSValue());
	}
	if (protocol2019Handler.getYAValue() != protocol2019Handler.ignoreValue) {
		YMotor.setAcceleration(protocol2019Handler.getYAValue());
	}
	if (protocol2019Handler.getYGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(YMotor.getCurrentPosition());
	}
	// YML <µL> — dispense via Y (B2 pump). mL × 1000 (µL).
	if (protocol2019Handler.getYMLValue() != protocol2019Handler.ignoreValue && !reagentDispenseLocked) {
		const long uL    = protocol2019Handler.getYMLValue();
		const float ml   = (float)uL / 1000.0f;
		const long steps = (long)((ml * (float)Y_STEPS_PER_REV) / Y_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
		YMotor.setMaxSpeed(Y_DISPENSE_SPEED);
		YMotor.setAcceleration(Y_DISPENSE_ACCEL);
		YMotor.move(steps);
		ackManager.requestMovementAck(&YMotor, "Y", micros());
		Serial.print("YML: ");  Serial.print(ml, 3);
		Serial.print(" mL @ "); Serial.print(Y_FLOW_ML_PER_REV, 3);
		Serial.print(" mL/rev = "); Serial.print(steps); Serial.println(" steps");
	} else if (protocol2019Handler.getYMLValue() != protocol2019Handler.ignoreValue) {
		Serial.println("YML: BLOCKED by REAGENTLOCK");
	}
	// PROFILE — select active stain profile (RP/LM/MG/WG).
	//   value -1 = query (print summary), 0..3 = set, -2 = invalid code (parser rejected).
	if (protocol2019Handler.getPROFILEValue() != protocol2019Handler.ignoreValue) {
		long pv = protocol2019Handler.getPROFILEValue();
		if (pv == -1) {
			printProfileSummary(profile());
		} else if (pv == -2) {
			Serial.println("[PROFILE] error: invalid code (use RP/LM/MG/WG or 0..3)");
		} else if (pv >= 0 && pv < PROFILE_COUNT) {
			const bool profileAccepted = (pcConfigurationPending && (ProfileId)pv == activeProfileId)
                || applyProfile((ProfileId)pv);
            if (profileAccepted && pcConfigurationPending) {
                pcProfileSelected = true;
                Serial.print("[PARA] selected PROFILE:"); Serial.println(profile().code);
                if (protocol2019Handler.getFEEDValue() == protocol2019Handler.ignoreValue &&
                    protocol2019Handler.getFEEDSTOPValue() == protocol2019Handler.ignoreValue) pcTryStart();
            }
		} else {
			Serial.print("[PROFILE] error: out of range "); Serial.println(pv);
		}
	}

	// DSP* — profile-driven dispense triggers. Volume read from active profile.
	// (Nozzle uses these via REQ:DSPE/DSPS1/DSPB1/DSPS2/DSPB2; manual ML commands still work too.)
	if (protocol2019Handler.getDSPEValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) {
			Serial.println("[DSPE] BLOCKED by REAGENTLOCK; transport continues");
		} else if (!profile().e_enabled) {
			Serial.print("[DSPE] skipped — E disabled in profile "); Serial.println(profile().code);
		} else if (M14Motor.isMoving()) {
            SensorFault::report("LIVO-SEQ-002", "DSPE", 0, "pump_busy_existing_dispense_preserved");
		} else {
			const long  uL    = profile().e_uL;
			const float ml    = (float)uL / 1000.0f;
			const long  steps = (long)((ml * (float)M14_STEPS_PER_REV) / M14_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
			M14Motor.setMaxSpeed(M14_DISPENSE_SPEED);
			M14Motor.setAcceleration(M14_DISPENSE_ACCEL);
			M14Motor.move(steps);
			pcEvent(TK_M14_AT);
			ackManager.requestMovementAck(&M14Motor, "M14", micros());
			Serial.print("[DSPE] profile="); Serial.print(profile().code);
			Serial.print(" uL="); Serial.print(uL); Serial.print(" steps="); Serial.println(steps);
		}
	}
	if (protocol2019Handler.getDSPS1Value() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) {
			Serial.println("[DSPS1] BLOCKED by REAGENTLOCK; transport continues");
		} else if (!profile().s1_enabled) {
			Serial.print("[DSPS1] skipped — S1 disabled in profile "); Serial.println(profile().code);
		} else if (ZMotor.isMoving()) {
            SensorFault::report("LIVO-SEQ-002", "DSPS1", 0, "pump_busy_existing_dispense_preserved");
		} else {
			const long  uL    = profile().s1_uL;
			const float ml    = (float)uL / 1000.0f;
			const long  steps = (long)((ml * (float)Z_STEPS_PER_REV) / Z_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
			ZMotor.setMaxSpeed(Z_DISPENSE_SPEED);
			ZMotor.setAcceleration(Z_DISPENSE_ACCEL);
			ZMotor.move(steps);
			pcEvent(TK_S1_AT);
			ackManager.requestMovementAck(&ZMotor, "Z", micros());
			Serial.print("[DSPS1] profile="); Serial.print(profile().code);
			Serial.print(" uL="); Serial.print(uL); Serial.print(" steps="); Serial.println(steps);
		}
	}
	if (protocol2019Handler.getDSPB1Value() != protocol2019Handler.ignoreValue) {
		// B1 runs through the B1MIX cascade (M15 dispense + DCM1/DCM2 mix tail).
		armB1Mixcade();
	}
	if (protocol2019Handler.getDSPS2Value() != protocol2019Handler.ignoreValue
	 || protocol2019Handler.getDSPB2Value() != protocol2019Handler.ignoreValue) {
		// S2 and B2 dispense via the S2MIX cascade (X + Y pumps + M18 stepper mix).
		// armS2Mixcade reads both volumes from the profile; either trigger fires it once per cycle.
		Serial.println("S2 cascade requires IR4 recipe positions; use individual pump commands for maintenance");
	}

	// SXCAS — IR6 stain wash/suction prep.
	if (protocol2019Handler.getSXCASValue() != protocol2019Handler.ignoreValue) {
	    armSxCascade();
	}
	// S2MIX — IR6 stain dispense cascade.
	if (protocol2019Handler.getS2MIXValue() != protocol2019Handler.ignoreValue) {
	    Serial.println("S2 cascade requires IR4 recipe positions; use individual pump commands for maintenance");
	}
	// B1MIX — IR6 B1 cascade (M15/B1 dispense + DCM1+DCM2 mix after).
	if (protocol2019Handler.getB1MIXValue() != protocol2019Handler.ignoreValue) {
	    armB1Mixcade();
	}
	// WYCAS — arm the WY cascade (DCW2, DCS2 — mirror of DCW1/DCS1 phase).
	if (protocol2019Handler.getWYCASValue() != protocol2019Handler.ignoreValue) {
		armWyCascade();
	}
	// MLOAD — prime every liquid line: each motor runs until its IR drops below MLOAD_IR_THRESHOLD.
	if (protocol2019Handler.getMLOADValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) {
			Serial.println("MLOAD: BLOCKED by REAGENTLOCK; recharge before preflight");
		} else {
			runMLoadBlocking();
		}
	}
	// MUNLOAD — reverse-purge all pumps, then DCW1/DCW2 5 s, then DCD 10 s. Non-blocking.
	if (protocol2019Handler.getMUNLOADValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("MUNLOAD: BLOCKED by REAGENTLOCK");
		else armMUnload();
	}
	// DRAIN — DCD on at full duty, monitor IR8 after 3s, stop when IR8 < 100. Non-blocking.
	if (protocol2019Handler.getDRAINValue() != protocol2019Handler.ignoreValue) {
		armDrain();
	}

	// ---- Legacy continuous pump starts disabled ----
	// Dispense commands must request a volume in uL. STOP commands remain active
	// as safety stops for any pump already moving.
	if (protocol2019Handler.getETDISValue() != protocol2019Handler.ignoreValue) {
		Serial.println("ETDIS disabled: use M14ML <uL> or DSPE/profile volume");
	}
	if (protocol2019Handler.getETSTOPValue() != protocol2019Handler.ignoreValue) {
		M14Motor.stop();
		Serial.println("ETSTOP: M14 stopped");
	}
	if (protocol2019Handler.getS1DISValue() != protocol2019Handler.ignoreValue) {
		Serial.println("S1DIS disabled: use ZML <uL> or DSPS1/profile volume");
	}
	if (protocol2019Handler.getS1STOPValue() != protocol2019Handler.ignoreValue) {
		ZMotor.stop();
		Serial.println("S1STOP: Z stopped");
	}
	if (protocol2019Handler.getS2DISValue() != protocol2019Handler.ignoreValue) {
		Serial.println("S2DIS disabled: use XML <uL> or DSPS2/profile volume");
	}
	if (protocol2019Handler.getS2STOPValue() != protocol2019Handler.ignoreValue) {
		XMotor.stop();
		Serial.println("S2STOP: X stopped");
	}
	if (protocol2019Handler.getB1DISValue() != protocol2019Handler.ignoreValue) {
		Serial.println("B1DIS disabled: use M15ML <uL> or B1MIX/profile volume");
	}
	if (protocol2019Handler.getB1STOPValue() != protocol2019Handler.ignoreValue) {
		M15Motor.stop();
		Serial.println("B1STOP: M15 stopped");
	}
	if (protocol2019Handler.getB2DISValue() != protocol2019Handler.ignoreValue) {
		Serial.println("B2DIS disabled: use YML <uL> or DSPB2/profile volume");
	}
	if (protocol2019Handler.getB2STOPValue() != protocol2019Handler.ignoreValue) {
		YMotor.stop();
		Serial.println("B2STOP: Y stopped");
	}
	// Z Motor
	if (protocol2019Handler.getZValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("Z: BLOCKED by REAGENTLOCK");
		else {
			ZMotor.moveTo(protocol2019Handler.getZValue());
			ackManager.requestMovementAck(&ZMotor, "Z", micros());
		}
	}
	if (protocol2019Handler.getZRValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("ZR: BLOCKED by REAGENTLOCK");
		else {
			ZMotor.move(protocol2019Handler.getZRValue());
			ackManager.requestMovementAck(&ZMotor, "Z", micros());
		}
	}
	if (protocol2019Handler.getZIValue() != protocol2019Handler.ignoreValue) {
		ZMotor.setRMSCurrentIRUN(protocol2019Handler.getZIValue());
	}
	if (protocol2019Handler.getZMValue() != protocol2019Handler.ignoreValue) {
		ZMotor.setMicrosteps(protocol2019Handler.getZMValue());
	}
	if (protocol2019Handler.getZSValue() != protocol2019Handler.ignoreValue) {
		ZMotor.setMaxSpeed(protocol2019Handler.getZSValue());
	}
	if (protocol2019Handler.getZAValue() != protocol2019Handler.ignoreValue) {
		ZMotor.setAcceleration(protocol2019Handler.getZAValue());
	}
	if (protocol2019Handler.getZGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(ZMotor.getCurrentPosition());
	}
	// ZML <µL> — dispense liquid by volume via Z motor (Kamoer KPAS300 pump).
	// Send mL × 1000 (µL) to keep integer precision.
	//   ZML 500   → 0.5 mL  (positive direction)
	//   ZML -500  → 0.5 mL reverse
	// Steps are derived from Z_FLOW_ML_PER_REV — change that constant to recalibrate.
	if (protocol2019Handler.getZMLValue() != protocol2019Handler.ignoreValue && !reagentDispenseLocked) {
		const long uL    = protocol2019Handler.getZMLValue();
		const float ml   = (float)uL / 1000.0f;
		const long steps = (long)((ml * (float)Z_STEPS_PER_REV) / Z_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
		ZMotor.setMaxSpeed(Z_DISPENSE_SPEED);
		ZMotor.setAcceleration(Z_DISPENSE_ACCEL);
		ZMotor.move(steps);
		ackManager.requestMovementAck(&ZMotor, "Z", micros());
		Serial.print("ZML: ");   Serial.print(ml, 3);
		Serial.print(" mL @ ");  Serial.print(Z_FLOW_ML_PER_REV, 3);
		Serial.print(" mL/rev = "); Serial.print(steps); Serial.println(" steps");
	} else if (protocol2019Handler.getZMLValue() != protocol2019Handler.ignoreValue) {
		Serial.println("ZML: BLOCKED by REAGENTLOCK");
	}
	// T Motor
	if (protocol2019Handler.getTValue() != protocol2019Handler.ignoreValue) {
		TMotor.moveTo(protocol2019Handler.getTValue());
		ackManager.requestMovementAck(&TMotor, "T", micros());
	}
	if (protocol2019Handler.getTRValue() != protocol2019Handler.ignoreValue) {
		TMotor.move(protocol2019Handler.getTRValue());
		ackManager.requestMovementAck(&TMotor, "T", micros());
	}
	if (protocol2019Handler.getTIValue() != protocol2019Handler.ignoreValue) {
		TMotor.setRMSCurrentIRUN(protocol2019Handler.getTIValue());
	}
	if (protocol2019Handler.getTMValue() != protocol2019Handler.ignoreValue) {
		TMotor.setMicrosteps(protocol2019Handler.getTMValue());
	}
	if (protocol2019Handler.getTSValue() != protocol2019Handler.ignoreValue) {
		TMotor.setMaxSpeed(protocol2019Handler.getTSValue());
	}
	if (protocol2019Handler.getTAValue() != protocol2019Handler.ignoreValue) {
		TMotor.setAcceleration(protocol2019Handler.getTAValue());
	}
	if (protocol2019Handler.getTGPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(TMotor.getCurrentPosition());
	}

	// M14 Motor
	if (protocol2019Handler.getM14Value() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("M14: BLOCKED by REAGENTLOCK");
		else {
			M14Motor.moveTo(protocol2019Handler.getM14Value());
			ackManager.requestMovementAck(&M14Motor, "M14", micros());
		}
	}
	if (protocol2019Handler.getM14RValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("M14R: BLOCKED by REAGENTLOCK");
		else {
			M14Motor.move(protocol2019Handler.getM14RValue());
			ackManager.requestMovementAck(&M14Motor, "M14", micros());
		}
	}
	if (protocol2019Handler.getM14IValue() != protocol2019Handler.ignoreValue) {
		M14Motor.setRMSCurrentIRUN(protocol2019Handler.getM14IValue());
	}
	if (protocol2019Handler.getM14MValue() != protocol2019Handler.ignoreValue) {
		M14Motor.setMicrosteps(protocol2019Handler.getM14MValue());
	}
	if (protocol2019Handler.getM14SValue() != protocol2019Handler.ignoreValue) {
		M14Motor.setMaxSpeed(protocol2019Handler.getM14SValue());
	}
	if (protocol2019Handler.getM14AValue() != protocol2019Handler.ignoreValue) {
		M14Motor.setAcceleration(protocol2019Handler.getM14AValue());
	}
	if (protocol2019Handler.getM14GPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(M14Motor.getCurrentPosition());
	}
	// M14ML <µL> — dispense via M14 (E pump). mL × 1000 (µL).
	if (protocol2019Handler.getM14MLValue() != protocol2019Handler.ignoreValue && !reagentDispenseLocked) {
		const long uL    = protocol2019Handler.getM14MLValue();
		const float ml   = (float)uL / 1000.0f;
		const long steps = (long)((ml * (float)M14_STEPS_PER_REV) / M14_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
		M14Motor.setMaxSpeed(M14_DISPENSE_SPEED);
		M14Motor.setAcceleration(M14_DISPENSE_ACCEL);
		M14Motor.move(steps);
		ackManager.requestMovementAck(&M14Motor, "M14", micros());
		Serial.print("M14ML: "); Serial.print(ml, 3);
		Serial.print(" mL @ "); Serial.print(M14_FLOW_ML_PER_REV, 3);
		Serial.print(" mL/rev = "); Serial.print(steps); Serial.println(" steps");
	} else if (protocol2019Handler.getM14MLValue() != protocol2019Handler.ignoreValue) {
		Serial.println("M14ML: BLOCKED by REAGENTLOCK");
	}

	// M15 Motor
	if (protocol2019Handler.getM15Value() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("M15: BLOCKED by REAGENTLOCK");
		else {
			M15Motor.moveTo(protocol2019Handler.getM15Value());
			ackManager.requestMovementAck(&M15Motor, "M15", micros());
		}
	}
	if (protocol2019Handler.getM15RValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("M15R: BLOCKED by REAGENTLOCK");
		else {
			M15Motor.move(protocol2019Handler.getM15RValue());
			ackManager.requestMovementAck(&M15Motor, "M15", micros());
		}
	}
	if (protocol2019Handler.getM15IValue() != protocol2019Handler.ignoreValue) {
		M15Motor.setRMSCurrentIRUN(protocol2019Handler.getM15IValue());
	}
	if (protocol2019Handler.getM15MValue() != protocol2019Handler.ignoreValue) {
		M15Motor.setMicrosteps(protocol2019Handler.getM15MValue());
	}
	if (protocol2019Handler.getM15SValue() != protocol2019Handler.ignoreValue) {
		M15Motor.setMaxSpeed(protocol2019Handler.getM15SValue());
	}
	if (protocol2019Handler.getM15AValue() != protocol2019Handler.ignoreValue) {
		M15Motor.setAcceleration(protocol2019Handler.getM15AValue());
	}
	if (protocol2019Handler.getM15GPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(M15Motor.getCurrentPosition());
	}
	// M15ML <µL> — dispense liquid by volume via M15 motor (peristaltic pump = "B1").
	// Send mL × 1000 (µL) to keep integer precision; same logic as ZML.
	// Steps are derived from M15_FLOW_ML_PER_REV — change that constant to recalibrate.
	// On finish, schedules DCM1 + DCM2 mix cascade (see serviceM15MixCascade).
	if (protocol2019Handler.getM15MLValue() != protocol2019Handler.ignoreValue && !reagentDispenseLocked) {
		const long uL    = protocol2019Handler.getM15MLValue();
		const float ml   = (float)uL / 1000.0f;
		const long steps = (long)((ml * (float)M15_STEPS_PER_REV) / M15_FLOW_ML_PER_REV + (ml >= 0 ? 0.5f : -0.5f));
		M15Motor.setMaxSpeed(M15_DISPENSE_SPEED);
		M15Motor.setAcceleration(M15_DISPENSE_ACCEL);
		M15Motor.move(steps);
		ackManager.requestMovementAck(&M15Motor, "M15", micros());
		// Arm post-B1 mix cascade — DISABLED (use B1MIX instead for combined dispense + mix)
		// m15MixState = M15MIX_WAIT_FINISH;
		Serial.print("M15ML: ");  Serial.print(ml, 3);
		Serial.print(" mL @ ");   Serial.print(M15_FLOW_ML_PER_REV, 3);
		Serial.print(" mL/rev = "); Serial.print(steps); Serial.println(" steps");
	} else if (protocol2019Handler.getM15MLValue() != protocol2019Handler.ignoreValue) {
		Serial.println("M15ML: BLOCKED by REAGENTLOCK");
	}

	// M17 Motor
	if (protocol2019Handler.getM17Value() != protocol2019Handler.ignoreValue) {
		M17Motor.moveTo(protocol2019Handler.getM17Value());
		ackManager.requestMovementAck(&M17Motor, "M17", micros());
	}
	if (protocol2019Handler.getM17RValue() != protocol2019Handler.ignoreValue) {
		M17Motor.move(protocol2019Handler.getM17RValue());
		ackManager.requestMovementAck(&M17Motor, "M17", micros());
	}
	if (protocol2019Handler.getM17IValue() != protocol2019Handler.ignoreValue) {
		M17Motor.setRMSCurrentIRUN(protocol2019Handler.getM17IValue());
	}
	if (protocol2019Handler.getM17MValue() != protocol2019Handler.ignoreValue) {
		M17Motor.setMicrosteps(protocol2019Handler.getM17MValue());
	}
	if (protocol2019Handler.getM17SValue() != protocol2019Handler.ignoreValue) {
		M17Motor.setMaxSpeed(protocol2019Handler.getM17SValue());
	}
	if (protocol2019Handler.getM17AValue() != protocol2019Handler.ignoreValue) {
		M17Motor.setAcceleration(protocol2019Handler.getM17AValue());
	}
	if (protocol2019Handler.getM17GPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(M17Motor.getCurrentPosition());
	}

	// M18 Motor
	if (protocol2019Handler.getM18Value() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("M18: BLOCKED by REAGENTLOCK");
		else {
			M18Motor.moveTo(protocol2019Handler.getM18Value());
			ackManager.requestMovementAck(&M18Motor, "M18", micros());
		}
	}
	if (protocol2019Handler.getM18RValue() != protocol2019Handler.ignoreValue) {
		if (reagentDispenseLocked) Serial.println("M18R: BLOCKED by REAGENTLOCK");
		else {
			M18Motor.move(protocol2019Handler.getM18RValue());
			ackManager.requestMovementAck(&M18Motor, "M18", micros());
		}
	}
	if (protocol2019Handler.getM18IValue() != protocol2019Handler.ignoreValue) {
		M18Motor.setRMSCurrentIRUN(protocol2019Handler.getM18IValue());
	}
	if (protocol2019Handler.getM18MValue() != protocol2019Handler.ignoreValue) {
		M18Motor.setMicrosteps(protocol2019Handler.getM18MValue());
	}
	if (protocol2019Handler.getM18SValue() != protocol2019Handler.ignoreValue) {
		M18Motor.setMaxSpeed(protocol2019Handler.getM18SValue());
	}
	if (protocol2019Handler.getM18AValue() != protocol2019Handler.ignoreValue) {
		M18Motor.setAcceleration(protocol2019Handler.getM18AValue());
	}
	if (protocol2019Handler.getM18GPValue() != protocol2019Handler.ignoreValue) {
		Serial.print(M18Motor.getCurrentPosition());
	}

    if(protocol2019Handler.getFEEDValue()!=protocol2019Handler.ignoreValue){
        const long feed=protocol2019Handler.getFEEDValue();bool supported=false;
        for(unsigned i=0;i<SPH_COUNT;++i)if(feed==(long)SPEED_VALUES[i])supported=true;
        if(!supported){Serial.println("FEED rejected: use 60/90/120/150/180");return;}
        if((tFeedActive||lastNozzleQueueDepth>0||anyCascadeActive()) && (unsigned long)feed!=currentSph){
            Serial.println("FEED rejected: drain existing slides before changing row");return;
        }
        if(tFeedActive)return;
        currentSph=feed;currentSphIdx=speedIdxForSph(currentSph);
        NozzleSerial.print("FEED ");NozzleSerial.println(feed);
        tFeedSpeed=((float)feed*T_MOTOR_STEPS_PER_REV)/3600.0f;
        if(!masterBedStart())return;
        tFeedActive=true;TMotor.motor.setMaxSpeed(tFeedSpeed);TMotor.move(-2000000000L);
        Serial.println("FEED: acknowledged bed-step scheduling active");
    }
    if(protocol2019Handler.getFEEDSTOPValue()!=protocol2019Handler.ignoreValue){
        masterBedPause();Serial.println("FEEDSTOP: bed held; step queues retained");
    }
    if(protocol2019Handler.getBEDTESTValue()!=protocol2019Handler.ignoreValue ||
       protocol2019Handler.getBTRESETValue()!=protocol2019Handler.ignoreValue){
        Serial.println("BEDTEST/BTRESET retired: use FEED with [BED] position telemetry");
    }

	// TMC Stepper Common Commands
	if (protocol2019Handler.getGIValue() != protocol2019Handler.ignoreValue) {
		Serial.print("RMS Irun Currents -");
		Serial.print(" X: "); Serial.print(XMotor.getRMSCurrentIRUN());
		Serial.print(", Y: "); Serial.print(YMotor.getRMSCurrentIRUN());
		Serial.print(", Z: "); Serial.print(ZMotor.getRMSCurrentIRUN());
		Serial.print(", T: "); Serial.print(TMotor.getRMSCurrentIRUN());
	}
	if (protocol2019Handler.getGMValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Microsteps -");
		Serial.print(" X: "); Serial.print(XMotor.getMicrosteps());
		Serial.print(", Y: "); Serial.print(YMotor.getMicrosteps());
		Serial.print(", Z: "); Serial.print(ZMotor.getMicrosteps());
		Serial.print(", T: "); Serial.print(TMotor.getMicrosteps());
	}
	if (protocol2019Handler.getGSValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Max Speed -");
		Serial.print(" X: "); Serial.print(XMotor.getMaxSpeed());
		Serial.print(", Y: "); Serial.print(YMotor.getMaxSpeed());
		Serial.print(", Z: "); Serial.print(ZMotor.getMaxSpeed());
		Serial.print(", T: "); Serial.print(TMotor.getMaxSpeed());
	}
	if (protocol2019Handler.getGAValue() != protocol2019Handler.ignoreValue) {
		Serial.print("Acceleration -");
		Serial.print(" X: "); Serial.print(XMotor.getAcceleration());
		Serial.print(", Y: "); Serial.print(YMotor.getAcceleration());
		Serial.print(", Z: "); Serial.print(ZMotor.getAcceleration());
		Serial.print(", T: "); Serial.print(TMotor.getAcceleration());
	}

	// DC Motors
	// Mixing DC
	if (protocol2019Handler.getDCM1Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCM1Value() == 0) {
			mixdc1.stopMotor();
		}
		else if ((protocol2019Handler.getDCM1Value() > 1) && (protocol2019Handler.getDCM1Value() < 256)){
			if (reagentDispenseLocked) Serial.println("DCM1: BLOCKED by REAGENTLOCK");
			else mixdc1.runMotor(protocol2019Handler.getDCM1Value());
		}
	}
	if (protocol2019Handler.getDCM2Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCM2Value() == 0) {
			mixdc2.stopMotor();
		}
		else if ((protocol2019Handler.getDCM2Value() > 1) && (protocol2019Handler.getDCM2Value() < 256)){
			if (reagentDispenseLocked) Serial.println("DCM2: BLOCKED by REAGENTLOCK");
			else mixdc2.runMotor(protocol2019Handler.getDCM2Value());
		}
	}
	// Drain DC — digital ON/OFF only. PE11 (D_Motor) must stay off TIM1_CH2
	// because PB14 (MIXD = TIM1_CH2N) would otherwise mirror its PWM.
	if (protocol2019Handler.getDCDValue() != protocol2019Handler.ignoreValue) {
		digitalWrite(D_Motor, protocol2019Handler.getDCDValue() != 0 ? HIGH : LOW);
		Serial.print("DCD "); Serial.println(protocol2019Handler.getDCDValue() != 0 ? "ON" : "OFF");
	}
	// Wash DC — DCW1 PWM via washdc1.runMotor() on PD15 = TIM4_CH4. Real PWM.
	// Command interface uniform with DCW2.
	if (protocol2019Handler.getDCW1Value() != protocol2019Handler.ignoreValue) {
		long v = protocol2019Handler.getDCW1Value();
		if (v == 0) {
			washdc1.stopMotor();
			Serial.println("DCW1 OFF");
		} else if (v >= 1 && v <= 255) {
			washdc1.runMotor((int)v);
			Serial.print("DCW1 duty="); Serial.println(v);
		}
	}
	// DCW2 PWM via TIM1_CH1 on PE9. 0 = off, 1..255 = PWM duty.
	if (protocol2019Handler.getDCW2Value() != protocol2019Handler.ignoreValue) {
		long v = protocol2019Handler.getDCW2Value();
		if (v == 0) {
			washdc2.stopMotor();
			Serial.println("DCW2 OFF");
		} else if (v >= 1 && v <= 255) {
			washdc2.runMotor((int)v);
			Serial.print("DCW2 duty="); Serial.println(v);
		}
	}
	// Suction DC
	if (protocol2019Handler.getDCS1Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCS1Value() == 0) {
			suctiondc1.stopMotor();
		}
		else if ((protocol2019Handler.getDCS1Value() > 1) && (protocol2019Handler.getDCS1Value() < 256)){
			suctiondc1.runMotor(protocol2019Handler.getDCS1Value());
		}
	}
	if (protocol2019Handler.getDCS2Value() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getDCS2Value() == 0) {
			suctiondc2.stopMotor();
		}
		else if ((protocol2019Handler.getDCS2Value() > 1) && (protocol2019Handler.getDCS2Value() < 256)){
			suctiondc2.runMotor(protocol2019Handler.getDCS2Value());
		}
	}

	// ---- DCF1 / MIXM — DEPRECATED on master. PB14/PB15 are now the MX1919
	// H-bridge IN1/IN2 driven by MIXD/MIXR. Old handlers commented out.
	// if (protocol2019Handler.getDCF1Value() != protocol2019Handler.ignoreValue) {
	//     digitalWrite(MIXD, protocol2019Handler.getDCF1Value() != 0 ? HIGH : LOW);
	//     Serial.print("DCF1 "); Serial.println(protocol2019Handler.getDCF1Value() != 0 ? "ON" : "OFF");
	// }
	// if (protocol2019Handler.getMIXMValue() != protocol2019Handler.ignoreValue) {
	//     long v = protocol2019Handler.getMIXMValue();
	//     if (v == 0) { dcFan2.stopFan(); Serial.println("MIXM OFF"); }
	//     else if (v >= 1 && v <= 255) { dcFan2.runFan((int)v); Serial.print("MIXM duty="); Serial.println(v); }
	// }

	// ---- MX1919 dual H-bridge mix motor — IN1=PB14 (MIXD), IN2=PB15 (MIXR) ----
	//   MIXD <0..255>  forward direction PWM (IN1 = duty, IN2 = 0)
	//   MIXR <0..255>  reverse direction PWM (IN1 = 0,    IN2 = duty)
	//   Sending one direction always zeroes the other first so they can never
	//   both be active. Sending 0 (or out-of-range) stops the motor.
	//   If the same packet contains both MIXD and MIXR, the last one wins.
	auto applyMixHBridge = [](int forwardDuty, int reverseDuty) {
		// Always zero the inactive side first to avoid both-driven moments.
		if (forwardDuty > 0) {
			analogWrite(MIXR, 0);
			analogWrite(MIXD, forwardDuty);
		} else if (reverseDuty > 0) {
			analogWrite(MIXD, 0);
			analogWrite(MIXR, reverseDuty);
		} else {
			analogWrite(MIXD, 0);
			analogWrite(MIXR, 0);
		}
	};
	if (protocol2019Handler.getMIXDValue() != protocol2019Handler.ignoreValue) {
		long v = protocol2019Handler.getMIXDValue();
		if (v < 0 || v > 255) {
			Serial.println("MIXD: duty out of range 0..255");
		} else if (v == 0) {
			applyMixHBridge(0, 0);
			Serial.println("MIXD STOP (MIXR also off)");
		} else {
			applyMixHBridge((int)v, 0);
			Serial.print("MIXD duty="); Serial.print(v);
			Serial.println(" (MIXR forced 0)");
		}
	}
	if (protocol2019Handler.getMIXRValue() != protocol2019Handler.ignoreValue) {
		long v = protocol2019Handler.getMIXRValue();
		if (v < 0 || v > 255) {
			Serial.println("MIXR: duty out of range 0..255");
		} else if (v == 0) {
			applyMixHBridge(0, 0);
			Serial.println("MIXR STOP (MIXD also off)");
		} else {
			applyMixHBridge(0, (int)v);
			Serial.print("MIXR duty="); Serial.print(v);
			Serial.println(" (MIXD forced 0)");
		}
	}

	// Read Accelerometer
	if (protocol2019Handler.getRAMValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getRAMValue() == 1) {
			int16_t* values = acc1.readAcceldata();
		}
	}
	if (protocol2019Handler.getPAMValue() != protocol2019Handler.ignoreValue) {
		if (protocol2019Handler.getPAMValue() == 0) {
			acc1.readAccelContinuous(false);
		}
		if (protocol2019Handler.getPAMValue() == 1) {
			acc1.readAccelContinuous(true);
		}
	}
	#endif
}

bool Execution2019Handler::isHallSensorCommand() {
    if (protocol2019Handler.getHSValue()  != protocol2019Handler.ignoreValue) return true;
	if (protocol2019Handler.getMHP2SampleCountValue() != protocol2019Handler.ignoreValue) return true;
	if (protocol2019Handler.getMH2SampleCountValue() != protocol2019Handler.ignoreValue) return true;
	if (protocol2019Handler.getMHValue()  != protocol2019Handler.ignoreValue) return true;
    if (protocol2019Handler.getGXHValue() != protocol2019Handler.ignoreValue) return true;
    if (protocol2019Handler.getGZHValue() != protocol2019Handler.ignoreValue) return true;
    return false;
}

// Clear pending production work on entry. Releasing service ownership must not
// restart a suspended dispense, feed, timed fan, or queued slide operation.
static bool serviceSavedMagazineChecks=false;
bool serviceProductionBusy() {
#if defined(Master) && defined(Stainer_Master_PCB)
    return tFeedActive || nozzleProductionBusy || gantryProductionBusy ||
        lastNozzleQueueDepth > 0 || anyCascadeActive() ||
        mUnloadState != MUNLOAD_IDLE || drainState != DRAIN_IDLE ||
        XMotor.isMoving() || YMotor.isMoving() || ZMotor.isMoving() || TMotor.isMoving() ||
        M14Motor.isMoving() || M15Motor.isMoving() || M17Motor.isMoving() || M18Motor.isMoving();
#elif defined(Master) && defined(Stainer_Gantry_PCB)
    return autoLoadActive || GXMotor.isMoving() || GYMotor.isMoving() ||
        GZMotor.isMoving() || GRMotor.isMoving() || XMotor.isMoving();
#elif defined(Master) && defined(Nozzle_Mount_PCB)
    return nzTotalPipelineDepth() > 0 || nzBedEnabled ||
        nzSYOpState != MOP_IDLE || nzBXOpState != MOP_IDLE || nzSXOpState != MOP_IDLE ||
        nzWYOpState != MOP_IDLE || nzWXOpState != MOP_IDLE || nzFan4Running || nzWXParkedFanActive ||
        SXMotor.isMoving() || SYMotor.isMoving() || BXMotor.isMoving() || BYMotor.isMoving() ||
        WXMotor.isMoving() || WYMotor.isMoving();
#else
    return false;
#endif
}
void serviceDiagnosticQuiesce() {
#ifdef Stainer_Master_PCB
    masterBedFault=true;masterBedWaiting=false;
    tFeedActive=false;m15MixState=M15MIX_IDLE;sxCasState=SXCAS_IDLE;s2MixState=S2MIX_IDLE;
    wyCasState=WYCAS_IDLE;m18OscState=M18_OSC_IDLE;mUnloadState=MUNLOAD_IDLE;drainState=DRAIN_IDLE;
    dcFan2.stopFan();
#elif defined(Stainer_Gantry_PCB)
    autoLoadActive=false;autoLoadStopRequested=false;serviceSavedMagazineChecks=startMagzinechecks;startMagzinechecks=false;
#elif defined(Nozzle_Mount_PCB)
    nzBedFault=true;nzBedEnabled=false;
    for(auto& f:nzBedFilters)f.reset();nzFan4Running=false;nzWXParkedFanActive=false;
    nzSYOpState=nzBXOpState=nzSXOpState=nzWYOpState=nzWXOpState=MOP_IDLE;
    dcFan1.stopFan();dcFan2.stopFan();dcFan3.stopHighSpeedFan();dcFan4.stopHighSpeedFan();dcMotor.stopMotor();
#endif
}
void serviceDiagnosticPeripherals() {
#ifdef Stainer_Gantry_PCB
    for(unsigned h=0;h<2;++h) {
        auto& sensor=h?Magazine2Ir:Magazine1Ir;auto& bus=h?i2c2:i2c1;
        bool fresh=requestFullMagazineIrValuesBlocking(sensor,bus,2000);
        ServiceDiagnostics::result("LIVO-MAG-010",fresh?"PASS":"FAIL",String("HOLDER=")+String(h+1)+" fresh_sensor_frame="+(fresh?"yes":"no"));
        if(fresh)for(unsigned ch=0;ch<24;++ch) {
            long raw=sensor.getIrValues()[ch];
            ServiceDiagnostics::result("LIVO-SEN-021",SensorFaultPolicy::adcValid(raw)?"PASS":"FAIL",String("HOLDER=")+String(h+1)+" SENSOR=IR"+String(ch+1)+" RAW="+String(raw)+" ADC_domain_only");
        }
    }
#endif
}

void serviceDiagnosticRelease() {
#ifdef Stainer_Gantry_PCB
    startMagzinechecks=serviceSavedMagazineChecks;
#elif defined(Nozzle_Mount_PCB)
    dcFan2.runFan(255); // Restore the normal always-on cooling fan; other outputs stay stopped.
#endif
}
