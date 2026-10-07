/*
 * Every PROFILE_TIMINGS cell is an absolute BED position from that slide's
 * IR4 detection. No entry is a duration or an offset from another operation.
 * IR6/IR12 are observation-only; they never move a slide's origin/deadlines.
 * Motor strokes and calibrated pump volumes remain motor-step quantities.
 */

#ifndef STAIN_PARAMETERS_H_
#define STAIN_PARAMETERS_H_

#include <Arduino.h>
#include "TMCConstants.h"

// =============================================================================
// Gantry magazine-position Hall calibration experiment
// =============================================================================
#ifdef Stainer_Gantry_PCB

// Set this to 0 to compile out the experiment and retain only the established
// hard-coded magazine pickup positions. The experiment never writes EEPROM.
#define GANTRY_MAG_HALL_CAL_EXPERIMENT 1

#if GANTRY_MAG_HALL_CAL_EXPERIMENT
// One north-up magnet is fitted to each magazine holder. The AH49E sensor moves
// with GX and its ADC value rises as it approaches either magnet. IR5/PA7 is an
// otherwise-unused analog input on the Gantry PCB.
static const uint8_t GANTRY_MAG_HALL_PIN = IR5;

// GX scans from its homed zero toward holder 2. The two non-overlapping search
// windows associate the first peak with holder 1 and the second with holder 2.
static const long GANTRY_MAG_HALL_GZ_SCAN_POSITION = 70000L;
static const long GANTRY_MAG_HALL_SCAN_END_STEPS = -105000L;
static const long GANTRY_MAG_HALL_H1_WINDOW_MAX  =       0L;
static const long GANTRY_MAG_HALL_H1_WINDOW_MIN  =  -20000L;
static const long GANTRY_MAG_HALL_H2_WINDOW_MAX  =  -60000L;
static const long GANTRY_MAG_HALL_H2_WINDOW_MIN  = -105000L;
static const long GANTRY_MAG_HALL_SCAN_SPEED     =    8000L;
static const long GANTRY_MAG_HALL_SCAN_ACCEL     =   16000L;
static const long GANTRY_MAG_HALL_SAMPLE_STEPS   =      20L;
static const int  GANTRY_MAG_HALL_MIN_RISE       =     150;
static const unsigned long GANTRY_MAG_HALL_LEG_TIMEOUT_MS = 45000UL;

// If a magnet is not physically centred on the required slide-pickup point,
// enter the measured mechanical offset here after the experiment is verified.
static const long GANTRY_MAG_HALL_H1_PICK_OFFSET_STEPS = 0L;
static const long GANTRY_MAG_HALL_H2_PICK_OFFSET_STEPS = 0L;
#endif

#endif // Stainer_Gantry_PCB

// =============================================================================
// Nozzle-side mechanical constants (NOT per-profile, NOT FEED-scaled)
// =============================================================================
#ifdef Nozzle_Mount_PCB

// ── Nozzle homing sensor selection ───────────────────────────────────────────
// Hall sensors are the validated/default homing system. Set to 0 to restore
// the original DIN limit-switch homing path; that code remains compiled and
// maintained as the fallback while the physical switches remain installed.
#define NZ_USE_HALL_HOME_SENSORS 1

// Two-pass homing, shared by all five nozzle axes and the switch fallback.
// Speeds are nozzle microsteps/second; backoff/chunk match the GX/GZ sequence.
static const long NZ_HOME_SPEED = 8000L;
static const long NZ_HOME_SLOW_SPEED = 1000L;
static const long NZ_HOME_BACKOFF_STEPS = 2500L;
static const long NZ_HOME_SLOW_STEP_CHUNK = 200L;
static const unsigned long NZ_HOME_BACKOFF_TIMEOUT_MS = 10000UL;
static const uint8_t NZ_HOME_STABLE_SAMPLES = 5;

// AH49ENTR-G1 analog inputs. IR9 remains dedicated to the DHT11.
static const uint8_t NZ_SY_HOME_HALL_PIN = IR3;   // PA4 / ADC12_IN4
static const uint8_t NZ_BX_HOME_HALL_PIN = IR11;  // PC1 / ADC123_IN11
static const uint8_t NZ_SX_HOME_HALL_PIN = IR13;  // PC3 / ADC123_IN13
static const uint8_t NZ_WY_HOME_HALL_PIN = IR7;   // PC4 / ADC12_IN14
static const uint8_t NZ_WX_HOME_HALL_PIN = IR10;  // PC0 / ADC123_IN10

#if NZ_USE_HALL_HOME_SENSORS
// Per-axis trigger levels measured and verified on the machine. All five
// axes home when their Hall reading is at or above the configured value.
static const int NZ_SY_HOME_HALL_THRESHOLD = 3100;  // IR3
static const int NZ_BX_HOME_HALL_THRESHOLD = 1950;  // IR11
static const int NZ_SX_HOME_HALL_THRESHOLD = 3100;  // IR13
static const int NZ_WY_HOME_HALL_THRESHOLD = 2200;  // IR7
static const int NZ_WX_HOME_HALL_THRESHOLD = 1750;  // IR10
#endif

// ── IR sensor thresholds and array indices ───────────────────────────────────
// IR4 — ethanol station slide-detect
static const int NZ_IR4_IDX             = 3;      // index in smoothed-IR array
static const int NZ_IR4_THRESHOLD       = 3550;   // below = present; adjusted from 3500 after IR4 capture
static const int NZ_IR4_CLEAR_THRESHOLD = 3700;   // measured inter-slide gaps reach 3716-3732; re-arm below both peaks
constexpr unsigned long NZ_SLIDE_PRESENT_STABLE_MS = 20UL;
constexpr unsigned long NZ_SLIDE_CLEAR_STABLE_MS = 40UL; // IR4; bench waveform verification pending
// Restore the established IR6/IR12 re-arm interval; a 40 ms interval can
// re-arm on brief clear reflections. Confirm against actual sensor traces.
constexpr unsigned long NZ_CASCADE_CLEAR_STABLE_MS = 40UL;

// IR6 — stain station slide-detect (with hysteresis re-arm)
static const int NZ_IR6_IDX             = 5;
static const int NZ_IR6_THRESHOLD       = 3550;   // slide ~3000, clear ~4000; below = present
static const int NZ_IR6_CLEAR_THRESHOLD = 3700;   // re-arm only after slide has clearly passed

// IR12 — wash station slide-detect (with hysteresis re-arm)
static const int NZ_IR12_IDX             = 11;
static const int NZ_IR12_THRESHOLD       = 3750;   // below = slide present
static const int NZ_IR12_CLEAR_THRESHOLD = 3900;   // re-arm only after slide has clearly passed

// ── Motor stroke distances (steps — NOT speeds, NOT scaled) ──────────────────
// Stroke values are PHYSICAL distances the motor travels. They describe
// machine geometry and don't change with feed speed.
static const long    NZ_SY_STROKE        = 8640L;    // SY forward stroke (positive) — reduced 10% from 9600 (2026-08-26)
static const long    NZ_SX_STROKE        = -15000L;  // SX forward stroke (negative = retract)
static const long    NZ_SX_DEFAULT_PARK_POSITION = 0L;
static const long    NZ_SX_WG_PARK_POSITION      = -35000L; // WG fixed park; no automated stroke
// MG/WG keep SX fixed at their profile-specific park during stain automation.
static const long    NZ_SX_MG_PARK_POSITION      = -3000L;
// WY parks at -18000 after homing. MG/WG travel to -9000 and return; RP/LM
// retain the original full travel to home (0) and return.
static const long    NZ_WY_FIXED_PARK_POSITION = 0L;
static const long    NZ_WY_DEFAULT_STROKE      =  9000L;
static const long    NZ_WY_MG_WG_STROKE        =   9000L;
static const long    NZ_WX_STROKE        = -15000L;  // WX forward stroke from home (negative)
// MG/WG keep WX parked at home; at FEED 60 DCF3 provides a fixed drying cycle.

// ── Fan duty cycles (NOT scaled) ─────────────────────────────────────────────
// Percent duty for the two high-speed nozzle fans.
static const int     NZ_ETHANOL_SPEED_PCT  = 25;     // DCF4 during ethanol cycle
static const uint8_t NZ_WX_DCF3_SPEED_PCT  = 50U;    // DCF3 drying fan duty

// ── Legacy direct-BX-motion constants (NOT used by IR-driven flow) ───────────
// Kept for any manual BX commands outside the choreography.
static const long NZ_BX_STROKE       = -15000L;
static const long NZ_BX_STROKE_SPEED = 1200L;
static const long NZ_BX_RETURN_SPEED = 6400L;

#endif // Nozzle_Mount_PCB

// =============================================================================
// Master-side mechanical constants (NOT per-profile, NOT FEED-scaled)
// =============================================================================
#ifdef Stainer_Master_PCB

// ── Volumetric pump calibration ──────────────────────────────────────────────
// M14 (E pump) — KPHM100 ethanol head
constexpr int   M14_FULL_STEPS_PER_REV = 200;
constexpr float M14_FLOW_ML_PER_REV    = 0.2f;
constexpr int   M14_MICROSTEPS         = DEF_TMC_MICROSTEPS;
constexpr long  M14_STEPS_PER_REV      = (long)M14_FULL_STEPS_PER_REV * (long)M14_MICROSTEPS;
constexpr long  M14_DISPENSE_SPEED     = 25600;   // was 12800 — bumped 2× for faster ethanol dispense (2026-07-06)
constexpr long  M14_DISPENSE_ACCEL     = 128000;

// Z (S1 pump) — KPAS300 head
// Practical on-slide calibration (2026-07-14): 34595 steps delivered
// approximately 600-700 uL; midpoint 650 uL = 0.48099 mL per 25600 steps.
constexpr int   Z_FULL_STEPS_PER_REV = 200;
constexpr float Z_FLOW_ML_PER_REV    = 0.48099f;
constexpr int   Z_MICROSTEPS         = DEF_TMC_MICROSTEPS;
constexpr long  Z_STEPS_PER_REV      = (long)Z_FULL_STEPS_PER_REV * (long)Z_MICROSTEPS;
constexpr long  Z_DISPENSE_SPEED     = 5000;
constexpr long  Z_DISPENSE_ACCEL     = 128000;

// M15 (B1 pump) — KPHM100 buffer head
constexpr int   M15_FULL_STEPS_PER_REV = 200;
constexpr float M15_FLOW_ML_PER_REV    = 0.2f;
constexpr int   M15_MICROSTEPS         = DEF_TMC_MICROSTEPS;
constexpr long  M15_STEPS_PER_REV      = (long)M15_FULL_STEPS_PER_REV * (long)M15_MICROSTEPS;
constexpr long  M15_DISPENSE_SPEED     = 25600;
constexpr long  M15_DISPENSE_ACCEL     = 128000;

// X (S2 pump) — KPAS300 head
// Practical on-slide calibration: 25600 steps delivers approximately 370 uL.
constexpr int   X_FULL_STEPS_PER_REV = 200;
constexpr float X_FLOW_ML_PER_REV    = 0.370f;
constexpr int   X_MICROSTEPS         = DEF_TMC_MICROSTEPS;
constexpr long  X_STEPS_PER_REV      = (long)X_FULL_STEPS_PER_REV * (long)X_MICROSTEPS;
constexpr long  X_DISPENSE_SPEED     = 25600;
constexpr long  X_DISPENSE_ACCEL     = 128000;

// Y (B2 pump) — KPHM100 buffer head
constexpr int   Y_FULL_STEPS_PER_REV = 200;
constexpr float Y_FLOW_ML_PER_REV    = 0.2f;
constexpr int   Y_MICROSTEPS         = DEF_TMC_MICROSTEPS;
constexpr long  Y_STEPS_PER_REV      = (long)Y_FULL_STEPS_PER_REV * (long)Y_MICROSTEPS;
constexpr long  Y_DISPENSE_SPEED     = 50000;    // ~1.6 s for 540 µL B2 (was 25600 → ~2.9 s)
constexpr long  Y_DISPENSE_ACCEL     = 256000;   // ~0.2 s ramp to 50000 sps

// ── DC motor duty cycles (NOT FEED-scaled, NOT per-profile) ──────────────────
constexpr uint8_t DCM1_DUTY = 80;    // B1MIX cascade mixer 1
constexpr uint8_t DCM2_DUTY = 0;   // B1MIX cascade mixer 2
// EXPERIMENT S2-DCM2 (2026-09-29): MG/WG, ON at s2mix_at, OFF at s2disp_at.
// Experiment revision: ON at s2mix_at + 150 bed steps; OFF remains s2disp_at.
// Set false to restore the original DCM2 air-window ownership without deleting code.
constexpr bool S2_DCM2_EXPERIMENT_ENABLED = true;
constexpr uint8_t S2_DCM2_EXPERIMENT_VALUE = 150; // PG9: digital fallback, >=128 is ON.
// Baseline bed steps, scaled like recipe positions. Set 0 to restore immediate S2 start.
constexpr unsigned long S2_DCM2_EXPERIMENT_DELAY_STEPS = 150UL;
constexpr uint8_t DCW1_DUTY = 90;   // SXCAS wash motor
constexpr uint8_t DCS1_DUTY = 255;   // SXCAS suction motor (terminator)
constexpr uint8_t DCW2_DUTY = 90;   // WYCAS wash motor
constexpr uint8_t DCS2_DUTY = 255;   // WYCAS suction motor (terminator)
// ── S2MIX mixing motor — now a STEPPER on the M18 slot ──────────────────────
// Hardware change (2026-06-23): the MX1919 H-bridge DC mixing pump was
// replaced with a stepper motor wired to the M18 stepper driver. S2MIX cascade
// drives M18Motor (TMC2209).
//
// Mixing mode (2026-06-29): M18 OSCILLATES (forward → reverse → forward → …)
// from cascade arm time, concurrent with X+Y pumps and continuing past pump
// completion. Each oscillation moves the paddle MIX_OSC_AMPLITUDE_STEPS in one
// direction then reverses — produces true to-and-fro mixing instead of
// one-way spin. Oscillation amplitude tuned so each forward/reverse half-cycle
// is short enough to cause turbulent mixing, but long enough that the paddle
// actually displaces fluid.
constexpr long          MIX_OSC_SPEED            = 75000;   // peak oscillation speed (sps) — was 50000
// Dispatch-only speed (2026-08-26): mixing must finish and dispatch onto the
// slide as fast as possible once the arm starts its return, so dispatch now
// runs faster than the oscillation mixing phase. +33% over MIX_OSC_SPEED as a
// conservative first bump — NOT bench-verified, check for M18 stalls/skipped
// steps and pipe/tubing pressure before relying on this in production.
constexpr long          MIX_DISPATCH_SPEED       = 100000;  // was MIX_OSC_SPEED (75000) for dispatch too
constexpr long          MIX_OSC_AMPLITUDE_STEPS  = 25600;   // 1 full rev each direction at 128 microsteps
constexpr long          MIX_ACCEL                = 400000;  // smooth start/stop ramp (sps²) — bumped to match higher speed
constexpr long          MIX_RMS_CURRENT          = 1000;    // RMS current in mA — TMC2209 IRUN (was 1500)
// M18 oscillates from s2mix_at until the independent s2disp_at position.
// Both pumps must be complete at s2disp_at or production faults.
// Dispatch moves MIX_DISPATCH_STEPS of M18.
constexpr long          MIX_DISPATCH_STEPS         = 420000;  // M18 forward steps at end-of-cascade dispatch+purge — restored 2026-08-26: 300000 (~600 µL) was leaving mixture in the pipe; 420000 is the measured value that empties the pipe completely.
// Legacy alias — retained so any older code path still compiles.
constexpr long          MIX_FORWARD_SPEED          = MIX_OSC_SPEED;  // legacy alias
constexpr long          MIX_REVERSE_SPEED          = MIX_OSC_SPEED;  // legacy alias

// MIXD/MIXR PWM duties — retained ONLY for the manual H-bridge test command
// (analogWrite(MIXD/MIXR, …)) in case the hardware is still wired. S2MIX
// cascade no longer reads these.
constexpr uint8_t MIXD_DUTY  = 255;
constexpr uint8_t MIXR_DUTY  = 255;
constexpr uint8_t DCD_DUTY   = 255;   // DRAIN motor

// ── MX1919 H-bridge PWM carrier frequency (NOT FEED-scaled) ──────────────────
constexpr unsigned long PWM_FREQUENCY_HZ = 200;

#endif // Stainer_Master_PCB

// =============================================================================
// STAIN PROFILES — shared between master and nozzle
// =============================================================================
// Four tables make up the profile system:
//   1. PROFILES[]              — non-timing per-recipe values
//   2. PROFILE_TIMINGS[][][]   — every absolute IR4 position in T MOTOR STEPS,
//                                per FEED × profile × knob (per-FEED tunable)
//   3. PROFILE_SPEEDS[][][]    — motor stroke/return speeds, per speed × profile × knob
//   4. SPEED_VALUES[]          — list of operator-supported FEED speeds
//
// Operator changes recipe via the M; PROFILE <code> command. Operator changes
// belt speed via M; FEED <label>.
//
// PROFILE_TIMINGS values are in STEPS — physical slide-travel distance from
// IR4 leading edge to event fire point. Universal across all FEED speeds because the
// slide must reach the same position regardless of how fast the bed runs.
// Use timingStepsFor() for microstep-scaled runtime positions.
//
// PROFILE_SPEEDS is still 3D — motor stroke/return speeds may genuinely want
// per-FEED tuning (e.g., faster SY stroke at higher feed to keep up with bed).
//
// Supported FEED labels: 60, 90, 120, 150, 180 → 1.0/1.5/2.0/2.5/3.0 RPM.
// FEED commands with other values snap to the nearest supported speed.
// =============================================================================

// ── Profile identifier enum ──────────────────────────────────────────────────
enum ProfileId : uint8_t {
    PROFILE_RP = 0,   // Rapid
    PROFILE_LM = 1,   // Leishman
    PROFILE_MG = 2,   // May-Grunwald & Giemsa
    PROFILE_WG = 3,   // Wright & Giemsa
    PROFILE_COUNT
};

// ── Supported feed speeds and their index into PROFILE_TIMINGS/SPEEDS ────────
// FEED labels are 60, 90, 120, 150, 180. Mapping:
//   RPM = label / 60   →   label 60 = 1 RPM, label 180 = 3 RPM
//   steps/sec = label x T_MOTOR_STEPS_PER_REV / 3600
enum SpeedIdx : uint8_t {
    SPH_60  = 0,    // label 60  -> 1.0 RPM
    SPH_90  = 1,    // label 90  -> 1.5 RPM
    SPH_120 = 2,    // label 120 -> 2.0 RPM
    SPH_150 = 3,    // label 150 -> 2.5 RPM
    SPH_180 = 4,    // label 180 -> 3.0 RPM
    SPH_COUNT
};

// ── Bed geometry: step counts between IR sensors ────────────────────────────
// Measured by BEDTEST command (2026-06-19). Speed-invariant — these are
// physical distances along the bed expressed as T motor steps.
// Diagnostic geometry only: production is anchored exclusively at IR4.
constexpr unsigned long BED_STEPS_IR4_TO_IR6_BASE  = 2738UL;    // X at timing baseline
constexpr unsigned long BED_STEPS_IR4_TO_IR12_BASE = 15348UL;   // Y at timing baseline
constexpr unsigned long BED_STEPS_IR4_TO_IR6  = scaleTMotorSteps(BED_STEPS_IR4_TO_IR6_BASE);
constexpr unsigned long BED_STEPS_IR4_TO_IR12 = scaleTMotorSteps(BED_STEPS_IR4_TO_IR12_BASE);

// ── Slide geometry (measured 2026-08-25) ─────────────────────────────────────
// Slide width (direction of bed travel) = 25mm; one T-motor rotation advances
// the bed 29mm. Used to shift a step-position event from the slide's leading
// edge to its physical middle: add SLIDE_HALF_WIDTH_STEPS_BASE to an absolute
// bed-step column value. Expressed at T_MOTOR_TIMING_BASE_MICROSTEPS baseline,
// same convention as BED_STEPS_IR4_TO_IR6_BASE — scale with scaleTMotorSteps()
// before comparing against a live TMotor position.
constexpr unsigned long SLIDE_WIDTH_MM_BASE       = 25UL;
constexpr unsigned long T_MOTOR_MM_PER_REV_BASE   = 29UL;
constexpr unsigned long SLIDE_HALF_WIDTH_STEPS_BASE =
    ((unsigned long)T_MOTOR_TIMING_BASE_STEPS_PER_REV * SLIDE_WIDTH_MM_BASE +
     T_MOTOR_MM_PER_REV_BASE) / (2UL * T_MOTOR_MM_PER_REV_BASE);

constexpr unsigned long BED_SLIDE_MIN_PITCH_STEPS = scaleTMotorSteps(
    (T_MOTOR_TIMING_BASE_STEPS_PER_REV * SLIDE_WIDTH_MM_BASE) / T_MOTOR_MM_PER_REV_BASE);

// SUPERSEDED 2026-08-26: this assumed the S2MIX nozzle was a fixed point 15mm
// downstream of the wash nozzle, independent of the SX arm. Bench feedback
// showed that's wrong — the S2MIX nozzle rides on the same arm as wash/
// suction, so its trigger needs to be anchored to the wash-cascade-end / SX
// and s2m_at in the FEED 60 PROFILE_TIMINGS row). Left here for history only
// — no longer referenced.
// constexpr unsigned long S2MIX_NOZZLE_GAP_MM_BASE  = 15UL;
// constexpr unsigned long S2MIX_NOZZLE_GAP_STEPS_BASE =
//     ((unsigned long)T_MOTOR_TIMING_BASE_STEPS_PER_REV * S2MIX_NOZZLE_GAP_MM_BASE +
//      T_MOTOR_MM_PER_REV_BASE) / (2UL * T_MOTOR_MM_PER_REV_BASE);

// Numeric FEED label for each row. Used by speedIdxForSph() to snap arbitrary
// FEED values to the nearest table row.
constexpr unsigned long SPEED_VALUES[SPH_COUNT] = { 60, 90, 120, 150, 180 };

// ── Timing knob enum (column index into PROFILE_TIMINGS) ─────────────────────
// Each enum value is one independently editable absolute bed position.
// Every cell is an absolute bed position from this slide's IR4 edge.
// Declaration order is for editing; events execute in numeric position order.
enum TimingKnob : uint8_t {
    TK_M14_AT, // m14_at — absolute IR4 position
    TK_ETH_ON, // eth_on — absolute IR4 position
    TK_ETH_OFF, // eth_off — absolute IR4 position
    TK_SY_AT, // sy_at — absolute IR4 position
    TK_S1_AT, // s1_at — absolute IR4 position
    TK_B1_AT, // b1_at — absolute IR4 position
    TK_AIR_ON, // air_on — absolute IR4 position
    TK_AIR_OFF, // air_off — absolute IR4 position
    TK_SX_AT, // sx_at — absolute IR4 position
    TK_W1_ON, // w1_on — absolute IR4 position
    TK_W1_OFF, // w1_off — absolute IR4 position
    TK_SUCTION1_ON, // suction1_on — absolute IR4 position
    TK_SUCTION1_OFF, // suction1_off — absolute IR4 position
    TK_S2MIX_AT, // s2mix_at — absolute IR4 position
    TK_S2DISP_AT, // s2disp_at — absolute IR4 position
    TK_W2_ON, // w2_on — absolute IR4 position
    TK_W2_OFF, // w2_off — absolute IR4 position
    TK_SUCTION2_ON, // suction2_on — absolute IR4 position
    TK_SUCTION2_OFF, // suction2_off — absolute IR4 position
    TK_WY_AT, // wy_at — absolute IR4 position
    TK_WX_AT, // wx_at — absolute IR4 position
    TK_DRY_ON, // dry_on — absolute IR4 position
    TK_DRY_OFF, // dry_off — absolute IR4 position
    TK_COUNT
};

// ── Speed knob enum (column index into PROFILE_SPEEDS) ───────────────────────
enum SpeedKnob : uint8_t {
    SK_SY_STROKE,
    SK_SY_RETURN,
    SK_SX_STROKE,
    SK_SX_RETURN,
    SK_WY_STROKE,
    SK_WY_RETURN,
    SK_WX_STROKE,
    SK_WX_RETURN,
    SK_COUNT
};

// All positions scale with T microsteps.
// UNSET remains a validation sentinel. Enabled S2 recipes require an
// explicit absolute s2disp_at; disabled RP/LM entries may be zero.
constexpr unsigned long UNSET_BED_POSITION = 0xFFFFFFFFUL;
constexpr unsigned long PROFILE_TIMINGS[SPH_COUNT][PROFILE_COUNT][TK_COUNT] = {

// FEED 60; ALL values are absolute bed steps from IR4.
//           m14_at  eth_on  eth_off  sy_at  s1_at  b1_at  air_on  air_off  sx_at  w1_on  w1_off  suction1_on  suction1_off  s2mix_at  s2disp_at  w2_on  w2_off  suction2_on  suction2_off  wy_at  wx_at  dry_on  dry_off
{
  /* RP */ { 425,    1010,   2290,    3538,  3618,  5908,  5948,   6748,    9938,  9938,  10338,  9954,        10434,        10381,    0,         17748, 18148,  17828,       18628,        17748, 20148, 20148,  21148   },
  /* LM */ { 267,    1010,   2290,    3738,  3798,  7602,  7642,   8442,    9938,  9938,  10338,  9954,        10434,        10381,    0,         17700, 18100,  17780,       18580,        17700, 18516, 18516,  19516   },
  /* MG */ { 267,    1360,   2240,    3838,  3950,  8200,  8240,   9040,    12000, 12000, 12400,  12416,       12896,        13350,    14082,     17965, 18422,  18127,       18516,        18100, 18516, 18516,  19983   },
  /* WG */ { 267,    1360,   2240,    3838,  3950,  7000,  7040,   7840,    10400, 10400, 10800,  10416,       10896,        10128,    10611,     17565, 18022,  17727,       18207,        17645, 18516, 18516,  19983   },
},

// FEED 90; ALL values are absolute bed steps from IR4.
//           m14_at  eth_on  eth_off  sy_at  s1_at  b1_at  air_on  air_off  sx_at  w1_on  w1_off  suction1_on  suction1_off  s2mix_at  s2disp_at  w2_on  w2_off  suction2_on  suction2_off  wy_at  wx_at  dry_on  dry_off
{
  /* RP */ { 425,    1010,   2290,    3538,  3618,  5218,  5258,   6058,    9938,  9938,  10338,  9954,        10434,        10498,    0,         17748, 18148,  17828,       18628,        17748, 20148, 20148,  21148   },
  /* LM */ { 267,    1010,   2290,    3738,  3798,  7112,  7152,   7952,    9938,  9938,  10338,  9954,        10434,        10498,    0,         17700, 18100,  17780,       18580,        17700, 18516, 18516,  19516   },
  /* MG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        13498,    13682,     17748, 18148,  17828,       18308,        17748, 19108, 19108,  20108   },
  /* WG */ { 400,    960,    2240,    3538,  3748,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10498,    10611,     17748, 18148,  17828,       18308,        17748, 19108, 19108,  20108   },
},

// FEED 120; ALL values are absolute bed steps from IR4.
//           m14_at  eth_on  eth_off  sy_at  s1_at  b1_at  air_on  air_off  sx_at  w1_on  w1_off  suction1_on  suction1_off  s2mix_at  s2disp_at  w2_on  w2_off  suction2_on  suction2_off  wy_at  wx_at  dry_on  dry_off
{
  /* RP */ { 425,    1010,   2290,    3538,  3618,  5218,  5258,   6058,    9938,  9938,  10338,  9954,        10434,        10471,    0,         17748, 18148,  17828,       18628,        17748, 20148, 20148,  21148   },
  /* LM */ { 267,    1010,   2290,    3838,  3958,  7212,  7252,   8052,    9938,  9938,  10338,  9954,        10434,        10471,    0,         17700, 18100,  17780,       18580,        17700, 18516, 18516,  19516   },
  /* MG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10471,    13682,     17748, 18148,  17828,       18308,        17748, 19375, 19375,  20375   },
  /* WG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10471,    10611,     17748, 18148,  17828,       18308,        17748, 19375, 19375,  20375   },
},

// FEED 150; ALL values are absolute bed steps from IR4.
//           m14_at  eth_on  eth_off  sy_at  s1_at  b1_at  air_on  air_off  sx_at  w1_on  w1_off  suction1_on  suction1_off  s2mix_at  s2disp_at  w2_on  w2_off  suction2_on  suction2_off  wy_at  wx_at  dry_on  dry_off
{
  /* RP */ { 267,    1010,   2290,    3688,  3800,  5068,  5108,   5908,    9938,  9938,  10338,  9954,        10434,        10445,    0,         17700, 18250,  17780,       18780,        17700, 18516, 18516,  20183   },
  /* LM */ { 513,    1010,   2290,    3538,  3565,  7062,  7102,   7902,    9938,  9938,  10338,  9954,        10434,        10445,    0,         17748, 18148,  17828,       18628,        17748, 20148, 20148,  21148   },
  /* MG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10445,    13682,     17748, 18148,  17828,       18308,        17748, 19641, 19641,  20641   },
  /* WG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10445,    10611,     17748, 18148,  17828,       18308,        17748, 19641, 19641,  20641   },
},

// FEED 180; ALL values are absolute bed steps from IR4.
//           m14_at  eth_on  eth_off  sy_at  s1_at  b1_at  air_on  air_off  sx_at  w1_on  w1_off  suction1_on  suction1_off  s2mix_at  s2disp_at  w2_on  w2_off  suction2_on  suction2_off  wy_at  wx_at  dry_on  dry_off
{
  /* RP */ { 267,    1010,   2290,    3738,  3858,  5418,  5458,   6258,    9938,  9938,  10338,  9954,        10434,        10418,    0,         17700, 18300,  17780,       18780,        17700, 18516, 18516,  20363   },
  /* LM */ { 663,    1010,   2290,    3538,  3698,  6912,  6952,   7752,    9938,  9938,  10338,  9954,        10434,        10418,    0,         17748, 18148,  17828,       18628,        17748, 20148, 20148,  21148   },
  /* MG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10418,    13682,     17748, 18148,  17828,       18308,        17748, 19908, 19908,  20908   },
  /* WG */ { 400,    960,    2240,    3538,  3698,  6418,  6458,   7258,    9938,  9938,  10338,  9954,        10434,        10418,    10611,     17748, 18148,  17828,       18308,        17748, 19908, 19908,  20908   },
},
};
// PROFILE_SPEEDS are each nozzle motor's own steps/second (not deadlines).
constexpr long PROFILE_SPEEDS[SPH_COUNT][PROFILE_COUNT][SK_COUNT] = {

// FEED 60 (1.0 RPM) -- motors x0.333 of 180-baseline
//           sy_st  sy_rt  sx_st  sx_rt  wy_st  wy_rt  wx_st  wx_rt
{ /* RP */ { 400,   2133,  400,   2133,  667,   2133,  400,   2133  },
  /* LM */ { 400,   2133,  400,   2133,  667,   2133,  400,   2133  },
  /* MG */ { 400,   2133,  400,   2133,  667,   2133,  400,   2133  },
  /* WG */ { 400,   2133,  400,   2133,  667,   2133,  400,   2133  } },

// FEED 90 (1.5 RPM) -- motors x0.5 of 180-baseline
//           sy_st  sy_rt  sx_st  sx_rt  wy_st  wy_rt  wx_st  wx_rt
{ /* RP */ { 600,   3200,  600,   3200,  1000,  3200,  600,   3200  },
  /* LM */ { 600,   3200,  600,   3200,  1000,  3200,  600,   3200  },
  /* MG */ { 600,   3200,  600,   3200,  1000,  3200,  600,   3200  },
  /* WG */ { 600,   3200,  600,   3200,  1000,  3200,  600,   3200  } },

// FEED 120 (2.0 RPM) -- motors x0.667 of 180-baseline
//           sy_st  sy_rt  sx_st  sx_rt  wy_st  wy_rt  wx_st  wx_rt
{ /* RP */ { 800,   4267,  800,   4267,  1333,  4267,  800,   4267  },
  /* LM */ { 800,   4267,  800,   4267,  1333,  4267,  800,   4267  },
  /* MG */ { 800,   4267,  800,   4267,  1333,  4267,  800,   4267  },
  /* WG */ { 800,   4267,  800,   4267,  1333,  4267,  800,   4267  } },

// FEED 150 (2.5 RPM) -- motors x0.833 of 180-baseline
//           sy_st  sy_rt  sx_st  sx_rt  wy_st  wy_rt  wx_st  wx_rt
{ /* RP */ { 1000,  5333,  1000,  5333,  1667,  5333,  600,   6000  },
  /* LM */ { 1000,  5333,  1000,  5333,  1667,  5333,  1000,  5333  },
  /* MG */ { 1000,  5333,  1000,  5333,  1667,  5333,  1000,  5333  },
  /* WG */ { 1000,  5333,  1000,  5333,  1667,  5333,  1000,  5333  } },

// FEED 180 (3.0 RPM) -- calibrated motor speed baseline
//           sy_st  sy_rt  sx_st  sx_rt  wy_st  wy_rt  wx_st  wx_rt
{ /* RP */ { 1200,  6400,  1200,  6400,  2000,  6400,  650,   7000  },
  /* LM */ { 1200,  6400,  1200,  6400,  2000,  6400,  1200,  6400  },
  /* MG */ { 1200,  6400,  1200,  6400,  2000,  6400,  1200,  6400  },
  /* WG */ { 1200,  6400,  1200,  6400,  2000,  6400,  1200,  6400  } },

};

// =============================================================================
// PROFILE_SPEEDS — column reference (8 motor-speed knobs)
// =============================================================================
// All values in steps/sec, after each motor's own microstepping (configured
// in TMCConstants.cpp — typically 128 for nozzle motors).
// Every motor here lives on the NOZZLE PCB. The MASTER PCB does NOT consume
// any of these values — the nozzle reads speedFor(profile, SK_*) and sets
// the corresponding motor's setMaxSpeed() at stroke time.
//
//  Col  Header  Enum            Meaning                                                  PCB
//  ───  ──────  ───────────────  ──────────────────────────────────────────────────────── ──────
//   0   sy_st   SK_SY_STROKE    SY motor stroke (forward) speed                           Nozzle
//   1   sy_rt   SK_SY_RETURN    SY motor return-to-home speed (typically faster than st)  Nozzle
//   2   sx_st   SK_SX_STROKE    SX motor stroke (forward) speed                           Nozzle
//   3   sx_rt   SK_SX_RETURN    SX motor return-to-home speed                             Nozzle
//   4   wy_st   SK_WY_STROKE    WY motor stroke (forward) speed                           Nozzle
//   5   wy_rt   SK_WY_RETURN    WY motor return-to-home speed                             Nozzle
//   6   wx_st   SK_WX_STROKE    WX motor stroke (forward) speed                           Nozzle
//   7   wx_rt   SK_WX_RETURN    WX motor return-to-home speed                             Nozzle
//
// ── Why return speeds are typically faster than stroke speeds ───────────────
// The stroke is timed to coordinate with slide arrival. The return doesn't
// need precise timing — it just needs to get the nozzle out of the way for
// the next slide. So return speeds are tuned higher (here, ~5× of stroke).
//
// ── PCB ownership summary ───────────────────────────────────────────────────
// All 8 columns: NOZZLE PCB. The motors are physically wired to the nozzle.
// Pump motors (M14/Z/M15/X/Y, all on master) have their own DISPENSE_SPEED
// constants elsewhere in STAIN-PARAMETERS.h — they are NOT in this table.
// =============================================================================

// =============================================================================
// StainProfile struct — non-timing per-recipe values
// =============================================================================
struct StainProfile {
    ProfileId   id;
    const char* code;
    const char* description;

    // Per-pump dispense volumes (uL) + enable flags.
    // enabled=false → the corresponding REQ:DSP*/cascade arm is fully skipped.
    long  e_uL;     bool e_enabled;
    long  s1_uL;    bool s1_enabled;
    long  b1_uL;    bool b1_enabled;
    long  s2_uL;    bool s2_enabled;
    long  b2_uL;    bool b2_enabled;

    // BX park position applied at end of NZHO homing (absolute steps from home).
    long  bx_position;

    // Per-cascade DC motor enables. enabled=false → that motor never fires
    // inside its cascade for this profile (cascade still arms; just no motor).
    bool sxcas_dcw1_enabled;
    bool sxcas_dcs1_enabled;
    bool wycas_dcw2_enabled;
    bool wycas_dcs2_enabled;
};

// =============================================================================
// PROFILES[] — recipe table (non-timing fields)
// =============================================================================
constexpr StainProfile PROFILES[PROFILE_COUNT] = {
//                                          E       S1     B1      S2       B2       bx     sxc_w1  sxc_s1  wyc_w2  wyc_s2
// Dispense tuning (2026-08-31): E +50 uL and S1 +140 uL for every profile.
/* RP */ { PROFILE_RP, "RP", "Rapid",        140,1,  380,1, 500,1,    0,0,    0,0, -29000,  false,  false,  true,   true  },
/* LM */ { PROFILE_LM, "LM", "Leishman",     140,1,  380,1, 500,1,    0,0,    0,0, -14000,  false,  false,  true,   true  },
// S2:B2 ratios, both retaining 1260 uL total:
//   MG = 1:8 (140:1120); WG = 1:9 (126:1134).
// WG S1 and B1 are each 400 uL; MG retains 500 uL each.
// Prior values (revert here if needed):
//   1:9 clinical std (2026-07-02) : S1=500, S2=100, B2=900
//   1:9 tuned        (2026-07-02) : S1=300, S2=90,  B2=810
/* MG */ { PROFILE_MG, "MG", "May-G & G",    140,1,  500,1, 500,1,  140,1, 1120,1,      0,  true,   true,   true,   true  },
/* WG */ { PROFILE_WG, "WG", "Wright & G",   140,1,  400,1, 400,1,  126,1, 1134,1, -13500,  true,   true,   true,   true  },
};

// =============================================================================
// Runtime FEED state — set by FEED command on each MCU
// =============================================================================
// Each MCU keeps its own copies. Both defined in Execution2019Handler.cpp.
//   currentSph    — the FEED label the operator typed (60/90/120/150/180).
//                   Kept under the legacy name `currentSph` for diff-friendliness;
//                   semantically it now stores the FEED-label (= bed RPM × 60),
//                   steps->ms conversion uses T_MOTOR_STEPS_PER_REV.
//   currentSphIdx — the snapped row index used by speedFor() to look up
//                   per-FEED motor speeds in PROFILE_SPEEDS.
// =============================================================================
extern unsigned long currentSph;
extern SpeedIdx      currentSphIdx;

// Snap an arbitrary FEED label to the nearest row in SPEED_VALUES[].
// Tie-break favors lower index (slower speed) for safety.
static inline SpeedIdx speedIdxForSph(unsigned long feedLabel) {
    SpeedIdx best       = SPH_180;
    unsigned long bestDiff = (unsigned long)-1;
    for (uint8_t i = 0; i < SPH_COUNT; i++) {
        unsigned long diff = (SPEED_VALUES[i] > feedLabel) ? (SPEED_VALUES[i] - feedLabel)
                                                          : (feedLabel - SPEED_VALUES[i]);
        if (diff < bestDiff) { bestDiff = diff; best = (SpeedIdx)i; }
    }
    return best;
}

// =============================================================================
// Production accessors: all timing values remain in bed steps.
// All cells are absolute from IR4, including every ON and OFF position.
static inline unsigned long timingStepsFor(ProfileId pid, TimingKnob k) {
    const unsigned long position=PROFILE_TIMINGS[currentSphIdx][pid][k];
    return position==UNSET_BED_POSITION?position:scaleTMotorSteps(position);
}

// Reject undefined, empty or inverted exposure windows before any bed motion.
static inline bool bedRecipeConfigured(ProfileId p) {
    const auto& recipe=PROFILES[p];
    for(unsigned k=0;k<TK_COUNT;++k) {
        if(k==TK_S2DISP_AT && !recipe.s2_enabled && !recipe.b2_enabled)continue;
        if(timingStepsFor(p,(TimingKnob)k)>=0x80000000UL)return false;
    }
    const TimingKnob pairs[][2]={{TK_ETH_ON,TK_ETH_OFF},{TK_AIR_ON,TK_AIR_OFF},
        {TK_W1_ON,TK_W1_OFF},{TK_SUCTION1_ON,TK_SUCTION1_OFF},
        {TK_W2_ON,TK_W2_OFF},{TK_SUCTION2_ON,TK_SUCTION2_OFF},{TK_DRY_ON,TK_DRY_OFF}};
    for(const auto& pair:pairs)if(timingStepsFor(p,pair[1])<=timingStepsFor(p,pair[0]))return false;
    if((recipe.s2_enabled||recipe.b2_enabled) && timingStepsFor(p,TK_S2DISP_AT)<=timingStepsFor(p,TK_S2MIX_AT))return false;
    return true;
}

static inline long speedFor(ProfileId pid, SpeedKnob k) {
    return PROFILE_SPEEDS[currentSphIdx][pid][k];
}

#endif /* STAIN_PARAMETERS_H_ */
