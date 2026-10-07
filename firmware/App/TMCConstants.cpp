#include "Stm32SerialCompat.h"
/*
 * TMCConstants.cpp
 *
 *  Created on: June 7, 2025
 *      Author: Varalakshmi
 */

#include "TMCConstants.h"

#ifdef Master

#ifdef Stainer_Gantry_PCB
LivoHardwareSerial GXZR_SERIAL_PORT(GXZR_UART_RX, GXZR_UART_TX);  // gantry X, Z, R
LivoHardwareSerial GY_SERIAL_PORT(GY_UART_RX, GY_UART_TX);        // Gantry X
#endif

#ifdef Nozzle_Mount_PCB
LivoHardwareSerial SXY_SERIAL_PORT(SXY_UART_RX, SXY_UART_TX);  // Stain
LivoHardwareSerial WXY_SERIAL_PORT(WXY_UART_RX, WXY_UART_TX);  // Wash
LivoHardwareSerial BXY_SERIAL_PORT(BXY_UART_RX, BXY_UART_TX);  // Buffer
#endif

#ifdef Stainer_Master_PCB
LivoHardwareSerial ZT_SERIAL_PORT(ZT_UART_RX, ZT_UART_TX);  // Master
LivoHardwareSerial XY_SERIAL_PORT(XY_UART_RX, XY_UART_TX);  // Master
#endif

const TMCConfig TMC_DEFAULT_CONFIG = {
  .microsteps = DEF_TMC_MICROSTEPS,
  .speed = DEF_TMC_MAXSPEED,
  .acceleration = DEF_TMC_MAXACCELERATION,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_RMS_CURRENT_I_HOLD,
};

const TMCConfig TMC_MICRO_DEFAULT_CONFIG = {
  // Micro Stepper
  .microsteps = DEF_TMC_MICROSTEPS,
  .speed = DEF_TMC_MAXSPEED,
  .acceleration = DEF_TMC_MAXACCELERATION,
  .rms_current_I_RUN = DEF_MICROSTEP_RMS_CURRENT_IRUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};

// Gantry
const TMCConfig TMCMotorGXConfig = {
  .microsteps = 32,
  .speed = 30000,
  .acceleration = 10000,
  .rms_current_I_RUN = 1200,
  .rms_current_I_HOLD = DEF_RMS_CURRENT_I_HOLD,
};
// GY drives a 0.5mm-pitch leadscrew (12800 steps/mm at 32 microsteps) pushing
// an unloaded slide only. acceleration=100000 -> ~7.8 mm/s^2, ~0.2s ramp to
// full speed (was 2100 -> ~0.16 mm/s^2, a 9.5s ramp -- far slower than GX/GZ's
// ~0.8-1.25s ramps despite GY carrying the least load of the three).
// I_RUN provides enough torque headroom for that
// acceleration; I_HOLD given an explicit value instead of the prior TODO.
const TMCConfig TMCMotorGYConfig = {
  .microsteps = 32,
  .speed = 20000,
  .acceleration = 100000,
  .rms_current_I_RUN = 800,
  .rms_current_I_HOLD = 200,
};
const TMCConfig TMCMotorGRConfig = {
  .microsteps = DEF_TMC_MICROSTEPS,
  .speed = DEF_TMC_MAXSPEED,
  .acceleration = DEF_TMC_MAXACCELERATION,
  .rms_current_I_RUN = 1000,
  .rms_current_I_HOLD = 200,
};
const TMCConfig TMCMotorGZConfig = {
  .microsteps = 32,
  .speed = 30000,
  .acceleration = 10000,
  .rms_current_I_RUN = 1000,
  .rms_current_I_HOLD = DEF_RMS_CURRENT_I_HOLD,
};
const TMCConfig TMCMotorG2YConfig = TMC_MICRO_DEFAULT_CONFIG;  // Extra Motor
// Nozzle Mount — full-step (microsteps=1), 200 steps/rev
// speed=800 → 240 RPM | acceleration=2000 → ramps to 800 in 0.4sTHE 
const TMCConfig TMCMotorSXConfig = {
  .microsteps = 16,
  .speed = 3200,
  .acceleration = 400000,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorSYConfig = {
  .microsteps = 16,
  .speed = 10000,
  .acceleration = 100000,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorWXConfig = {
  .microsteps = 16,
  .speed = 22000,
  .acceleration = 400000,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorWYConfig = {
  .microsteps = 16,
  .speed = 22000,
  .acceleration = 400000,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorBXConfig = {
  .microsteps = 16,
  .speed = 3200,
  .acceleration = 400000,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorBYConfig = {
  .microsteps = 16,
  .speed = 22000,
  .acceleration = 400000,
  .rms_current_I_RUN = DEF_RMS_CURRENT_I_RUN,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};  // Extra Motor
// Stainer Master
const TMCConfig TMCMotorZConfig = {
  // Micro Stepper
  .microsteps = DEF_TMC_MICROSTEPS,
  .speed = DEF_TMC_MAXSPEED,
  .acceleration = DEF_TMC_MAXACCELERATION,
  .rms_current_I_RUN = 1000,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorTConfig = {
  .microsteps = T_MOTOR_MICROSTEPS,
  .speed = T_MOTOR_STEPS_PER_REV,
  .acceleration = 1000 * T_MOTOR_MICROSTEPS,
  .rms_current_I_RUN = 800,
  .rms_current_I_HOLD = 100,  // near-zero hold — T motor needs no hold torque
};
const TMCConfig TMCMotorM14Config = TMC_DEFAULT_CONFIG;
const TMCConfig TMCMotorM15Config = TMC_DEFAULT_CONFIG;
const TMCConfig TMCMotorXConfig = {
  // Micro Stepper
  .microsteps = DEF_TMC_MICROSTEPS,
  .speed = DEF_TMC_MAXSPEED,
  .acceleration = DEF_TMC_MAXACCELERATION,
  .rms_current_I_RUN = 1000,
  .rms_current_I_HOLD = DEF_MICROSTEP_RMS_CURRENT_IHOLD,
};
const TMCConfig TMCMotorYConfig = TMC_DEFAULT_CONFIG;
const TMCConfig TMCMotorM17Config = TMC_DEFAULT_CONFIG;
const TMCConfig TMCMotorM18Config = TMC_DEFAULT_CONFIG;

#endif
