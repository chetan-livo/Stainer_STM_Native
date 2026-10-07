/*
* TMCConstants.h
*
*  Created on: June 7, 2025
*      Author: Varalakshmi
*/

#ifndef TMCCONSTANTS_H_
#define TMCCONSTANTS_H_

#include <Arduino.h>
#include "HeaderPCB.h"
#include "Stm32SerialCompat.h"

// Static constants for default values
const int DEF_TMC_MICROSTEPS = 128;
const int DEF_TMC_MAXACCELERATION = 1000 * DEF_TMC_MICROSTEPS;
const int DEF_TMC_MAXSPEED = 200 * DEF_TMC_MICROSTEPS;

constexpr int T_MOTOR_FULL_STEPS_PER_REV = 200;
constexpr int T_MOTOR_MICROSTEPS = 8;
constexpr int T_MOTOR_STEPS_PER_REV = T_MOTOR_FULL_STEPS_PER_REV * T_MOTOR_MICROSTEPS;

// All stored bed timing/geometry values are calibrated at this microstep
// setting. Runtime code scales them to T_MOTOR_MICROSTEPS automatically.
constexpr int T_MOTOR_TIMING_BASE_MICROSTEPS = 8;
constexpr int T_MOTOR_TIMING_BASE_STEPS_PER_REV =
    T_MOTOR_FULL_STEPS_PER_REV * T_MOTOR_TIMING_BASE_MICROSTEPS;

constexpr unsigned long scaleTMotorSteps(unsigned long baseSteps) {
    return (baseSteps * (unsigned long)T_MOTOR_MICROSTEPS +
            (T_MOTOR_TIMING_BASE_MICROSTEPS / 2UL)) /
           (unsigned long)T_MOTOR_TIMING_BASE_MICROSTEPS;
}

const int DEF_RMS_CURRENT_I_RUN = 800;
const int DEF_RMS_CURRENT_I_HOLD = 200;

const int DEF_MICROSTEP_RMS_CURRENT_IRUN = 800;
const int DEF_MICROSTEP_RMS_CURRENT_IHOLD = 100;

static constexpr float DEF_R_SENSE = 0.11f; 
static constexpr uint8_t DEF_DRIVER_ADDRESS = 0b00; 
// Gantry Address
static constexpr uint8_t GX_DRIVER_ADDRESS = 0b10; 
static constexpr uint8_t GY_DRIVER_ADDRESS = 0b00; 
static constexpr uint8_t GR_DRIVER_ADDRESS = 0b01;
static constexpr uint8_t GZ_DRIVER_ADDRESS = 0b00;
static constexpr uint8_t GY2_DRIVER_ADDRESS = 0b10;     // Extra Motor
// Nozzle Mount Address
static constexpr uint8_t SX_DRIVER_ADDRESS = 0b10;      // S2Y Motor
static constexpr uint8_t SY_DRIVER_ADDRESS = 0b00;      // SY Motor
static constexpr uint8_t WX_DRIVER_ADDRESS = 0b00;      // WX Motor
static constexpr uint8_t WY_DRIVER_ADDRESS = 0b10;      // WY Motor
static constexpr uint8_t BX_DRIVER_ADDRESS = 0b00;      // BX Motor
static constexpr uint8_t BY_DRIVER_ADDRESS = 0b10;      // BY Motor     Extra Motor
// Stainer Master Address
static constexpr uint8_t Z_DRIVER_ADDRESS   = 0b00;     // Z Motor
static constexpr uint8_t T_DRIVER_ADDRESS   = 0b11;     // T Motor
static constexpr uint8_t M14_DRIVER_ADDRESS = 0b10;     // M14 Motor
static constexpr uint8_t M15_DRIVER_ADDRESS = 0b01;     // M15 Motor
static constexpr uint8_t X_DRIVER_ADDRESS   = 0b00;     // X Motor
static constexpr uint8_t Y_DRIVER_ADDRESS   = 0b10;     // Y Motor
static constexpr uint8_t M17_DRIVER_ADDRESS = 0b01;     // M17 Motor
static constexpr uint8_t M18_DRIVER_ADDRESS = 0b11;     // M18 Motor

struct TMCConfig {
    int microsteps = DEF_TMC_MICROSTEPS;
    int speed = DEF_TMC_MAXSPEED;
    int acceleration = DEF_TMC_MAXACCELERATION;
    long rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN; 
    long rms_current_I_HOLD = DEF_RMS_CURRENT_I_HOLD; 
};

#ifdef Stainer_Gantry_PCB
    extern LivoHardwareSerial GXZR_SERIAL_PORT;
    extern LivoHardwareSerial GY_SERIAL_PORT;
#endif

#ifdef Nozzle_Mount_PCB
    extern LivoHardwareSerial SXY_SERIAL_PORT;
    extern LivoHardwareSerial WXY_SERIAL_PORT;
    extern LivoHardwareSerial BXY_SERIAL_PORT;
#endif

#ifdef Stainer_Master_PCB
    extern LivoHardwareSerial ZT_SERIAL_PORT;
    extern LivoHardwareSerial XY_SERIAL_PORT;
#endif

// Gantry
extern const TMCConfig TMCMotorGXConfig;
extern const TMCConfig TMCMotorGYConfig;
extern const TMCConfig TMCMotorGRConfig;
extern const TMCConfig TMCMotorGZConfig;
extern const TMCConfig TMCMotorG2YConfig;        // Extra Motor
// Nozzle
extern const TMCConfig TMCMotorSXConfig;
extern const TMCConfig TMCMotorSYConfig;
extern const TMCConfig TMCMotorWXConfig;
extern const TMCConfig TMCMotorWYConfig;
extern const TMCConfig TMCMotorBXConfig;
extern const TMCConfig TMCMotorBYConfig;        // Extra Motor
// Stainer Master
extern const TMCConfig TMCMotorZConfig;
extern const TMCConfig TMCMotorTConfig;
extern const TMCConfig TMCMotorM14Config;
extern const TMCConfig TMCMotorM15Config;
extern const TMCConfig TMCMotorXConfig;
extern const TMCConfig TMCMotorYConfig;
extern const TMCConfig TMCMotorM17Config;
extern const TMCConfig TMCMotorM18Config;

#endif /* TMCCONSTANTS_H_ */
