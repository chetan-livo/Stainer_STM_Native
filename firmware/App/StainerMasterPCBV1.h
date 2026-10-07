/*
 * StainerMasterPCBV1.h
 *
 *  Created on: Dec 02, 2025
 *      Author: Sasank
 */

#ifndef STAINERMASTERPCBV1_H_
#define STAINERMASTERPCBV1_H_


#include <Arduino.h>

const String pcbVersion = "V1";

// Stepper motors
// UART for Z, T, M14, M15 motors
const uint8_t ZT_UART_TX    =   PA2;        // Stepper2_TX
const uint8_t ZT_UART_RX    =   PA3;        // Stepper2_RX
const uint8_t ZT_En         =   PE12;       // EN1

// Z MOTOR  - M0, M1 = 0,0
const uint8_t Z_Step        =   PF10;       // Z_MOTOR_STEP
const uint8_t Z_Dir         =   PC13;       // Z_MOTOR_DIR
const uint8_t Z_SPRD        =   PC3;        // Z_SPRD/PDN

// T MOTOR  - M0, M1 = 1,1
const uint8_t T_Step        =   PE3;        // T_MOTOR_STEP
const uint8_t T_Dir         =   PE2;        // T_MOTOR_DIR
const uint8_t T_SPRD        =   PE6;        // T_SPRD/PDN

// M14 MOTOR - M0, M1 = 0,1
const uint8_t M14_Step      =   PF12;       // M14_MOTOR_STEP
const uint8_t M14_Dir       =   PF11;       // M14_MOTOR_DIR
const uint8_t M14_SPRD      =   PF13;       // M14_SPRD/PDN

// M15 MOTOR - M0, M1 = 1,0
const uint8_t M15_Step      =   PG0;        // M15_MOTOR_STEP
const uint8_t M15_Dir       =   PF15;       // M15_MOTOR_DIR
const uint8_t M15_SPRD      =   PG1;        // M15_SPRD/PDN

// UART for X, Y, M17, M18 motors
const uint8_t XY_UART_TX    =   PE8;        // Stepper1_TX
const uint8_t XY_UART_RX    =   PE7;        // Stepper1_RX
const uint8_t XY_En         =   PG5;        // EN2

// X MOTOR  - M0, M1 = 0,0
const uint8_t X_Step        =   PG14;       // X_MOTOR_STEP
const uint8_t X_Dir         =   PG13;       // X_MOTOR_DIR
const uint8_t X_SPRD        =   PG15;       // X_SPRD/PDN

// Y MOTOR  - M0, M1 = 0,1
const uint8_t Y_Step        =   PD0;        // Y_MOTOR_STEP
const uint8_t Y_Dir         =   PC12;       // Y_MOTOR_DIR
const uint8_t Y_SPRD        =   PD1;        // Y_SPRD/PDN

// M17 MOTOR - M0, M1 = 1,0
const uint8_t M17_Step      =   PG3;        // M17_MOTOR_STEP
const uint8_t M17_Dir       =   PG2;        // M17_MOTOR_DIR
const uint8_t M17_SPRD      =   PG4;        // M17_SPRD/PDN

// M18 MOTOR - M0, M1 = 1,1
const uint8_t M18_Step      =   PD13;       // M18_MOTOR_STEP
const uint8_t M18_Dir       =   PD12;       // M18_MOTOR_DIR
const uint8_t M18_SPRD      =   PD14;       // M18_SPRD/PDN

// Analog IR sensors
const uint8_t IR1           =   PA4;        // ETHNOL
const uint8_t IR2           =   PA5;        // STAIN-1
const uint8_t IR3           =   PA6;        // BUFFER-1
const uint8_t IR4           =   PA7;        // STAIN-2
const uint8_t IR5           =   PC4;        // BUFFER-2
const uint8_t IR6           =   PC5;        // WASH-1
const uint8_t IR7           =   PF3;        // WASH-2
const uint8_t IR8           =   PF6;        // DRAIN
const uint8_t IR9           =   PF7;        // A9
const uint8_t IR10          =   PF8;        // A10
const uint8_t IR11          =   PF9;        // A11
const uint8_t IR12          =   PC0;        // A12        

// I2cs for slave boards
// I2C1
const uint8_t I2C1_SDA      =   PB9;        // I2C1_SDA
const uint8_t I2C1_SCL      =   PB8;        // I2C1_SCL
const uint8_t I2C1_INT1     =   PF2;        // INT1
const uint8_t I2C1_INT2     =   PF1;        // INT2

// I2C3
const uint8_t I2C3_SDA      =   PC9;        // I2C3_SDA
const uint8_t I2C3_SCL      =   PA8;        // I2C3_SCL
const uint8_t I2C3_INT3     =   PE0;        // INT3
const uint8_t I2C3_INT4     =   PB12;       // INT4
const uint8_t Acc_INT       =   PD6;        // ACC_INT

// UARTs
// UART for Raspberry pi 
const uint8_t Pi_TX         =   PB10;       // Pi4_TX
const uint8_t Pi_RX         =   PB11;       // Pi4_RX

// UART for Gantry PCB
const uint8_t G_TX          =   PC6;        // MS1_TX
const uint8_t G_RX          =   PC7;        // MS1_RX
const uint8_t G_BootPin     =   PB1;        // GPIO1
const uint8_t G_ResetPin    =   PA10;       // GPIO2

// UART for NozzleMount PCB
const uint8_t NM_TX         =   PC10;       // MS2_TX
const uint8_t NM_RX         =   PC11;       // MS2_RX
const uint8_t NM_BootPin    =   PF14;       // GPIO3
const uint8_t NM_ResetPin   =   PE10;       // GPIO4

// RFID/ICSP
const uint8_t SPI_MISO      =   PC2;        // MISO
const uint8_t SPI_MOSI      =   PC1;        // MOSI
const uint8_t SPI_SCK       =   PB13;       // SCK
const uint8_t SPI_CS        =   PE4;        // CS
const uint8_t SPI_GPIO      =   PE5;        // GPIO_RFID

// Limit switches
const uint8_t Lim1          =   PF5;        // DIN1
const uint8_t Lim2          =   PF4;        // DIN2

// Relays
const uint8_t Relay_1       =   PD9;        // Relay_1
const uint8_t Relay_2       =   PD8;        // Relay_2

// DC Motors
// Drain motor — moved to PE11 (was PG9) so the no-PWM pin is freed up.
// DCD is driven digitally (digitalWrite) regardless — keeps PE11 off TIM1_CH2
// so PB14 (MIXD = TIM1_CH2N) doesn't cross-talk.
const uint8_t D_Motor       =   PE11;       // D_in   (was PG9, swapped with W_Motor1)

// Suction motors
const uint8_t S_Motor1      =   PG_11;      // S_inAN
const uint8_t S_Motor2      =   PG_10;      // S2_in
// Wash motors — DCW1 moved to PD15 (was PG9) so it has real PWM via TIM4_CH4.
// TIM4 is independent of TIM1 → no cross-talk with MIXD/MIXR/DCW2.
const uint8_t W_Motor1      =   PD15;       // W_in   (was PG9, swapped with M_Motor2 for PWM)
const uint8_t W_Motor2      =   PE9;        // W2_in

// Mixing motors — DCM2 moved to PG9 (was PD15). PG9 has no PWM timer mapping,
// so DCM2 is now digital ON/OFF (threshold-fallback). DCM2_DUTY was 255 so
// effectively unchanged behaviour; PWM speed-control on DCM2 is no longer
// available.
const uint8_t M_Motor1      =   PE14;       // M1_in
const uint8_t M_Motor2      =   PG_9;       // M2_in  (was PD15, swapped with W_Motor1)

// MX1919 dual H-bridge inputs (was Fan_1 / Fan_2 — repurposed for mix motor)
const uint8_t MIXD          =   PB14;       // DRAIN-2
const uint8_t MIXR          =   PB15;       // LED

// Extra pins
const uint8_t GPIO_PA0      =   PA0;        // Ex1
const uint8_t GPIO_PD7      =   PD7;        // Ex2

// RGB LED
const uint8_t Red           =   PB5;        // RED
const uint8_t Green         =   PB4;        // Green
const uint8_t Blue          =   PB0;        // Blue

const uint8_t irsensorPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8, IR9, IR10, IR11, IR12};
const uint8_t limitPins[] = {Lim1, Lim2};

#endif /* STAINERMASTERPCBV1_H_ */
