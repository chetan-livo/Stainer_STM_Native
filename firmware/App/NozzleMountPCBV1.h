/*
 * NozzleMountPCBV1.h
 *
 *  Created on: Dec 02, 2025
 *      Author: Sasank
 */

#ifndef NOZZLEMOUNTPCBV1_H_
#define NOZZLEMOUNTPCBV1_H_

#include <Arduino.h>

const String pcbVersion = "V1";

// Stepper motors

// SX_MOTOR1
const uint8_t SXY_UART_TX = PC6;      // Stepper_S_TX
const uint8_t SXY_UART_RX = PC7;      // Stepper_S_RX
const uint8_t SXY_En_____ = PB12;     // S_MOTOR_EN

const uint8_t SX_Step__  = PB14;     // S2Y_MOTOR_STEP 
const uint8_t SX_Dir___   = PB15;    // S2Y_MOTOR_DIR
const uint8_t SX_SPRD__  = PB13;     // S2Y_SPRD/PDN

//SY_MOTOR1
const uint8_t SY_Step__ = PD9;      // SY_MOTOR_STEP
const uint8_t SY_Dir___ = PD10;     // SY_MOTOR_DIR
const uint8_t SY_SPRD__ = PD12;     // SY_SPRD/PDN

//BX_MOTOR1
const uint8_t BXY_UART_TX = PA2;      // Stepper_B_TX
const uint8_t BXY_UART_RX = PA3;      // Stepper_B_RX
const uint8_t BXY_En_____ = PA15;    // B_MOTOR_EN

const uint8_t BX_Step__ = PD3;     // BX_MOTOR_STEP
const uint8_t BX_Dir___ = PD4;     // BX_MOTOR_DIR
const uint8_t BX_SPRD__ = PD1;     // BX_SPRD/PDN

//BY_MOTOR1
const uint8_t BY_Step__ = PD6;     // BY_MOTOR_STEP
const uint8_t BY_Dir___ = PB4;     // BY_MOTOR_DIR
const uint8_t BY_SPRD__ = PD5;     // BY_SPRD/PDN

//WX_MOTOR1
const uint8_t WXY_UART_TX = PC12;    // Stepper_W_TX
const uint8_t WXY_UART_RX = PD2;     // Stepper_W_RX
const uint8_t WXY_En_____ = PE0;     // W_MOTOR_EN

const uint8_t WX_Step__ = PB5;     // WX_MOTOR_STEP
const uint8_t WX_Dir___ = PB7;     // WX_MOTOR_DIR
const uint8_t WX_SPRD__ = PB9;     // WX_SPRD/PDN

//WY_MOTOR1
const uint8_t WY_Step__ = PE1;     // WY_MOTOR_STEP
const uint8_t WY_Dir___ = PE2;     // WY_MOTOR_DIR
const uint8_t WY_SPRD__ = PE4;     // WY_SPRD/PDN


// IR sensors
const uint8_t IR1       = PA0;      // A1
const uint8_t IR2       = PA1;      // A2
const uint8_t IR3       = PA4;      // A3
const uint8_t IR4       = PA5;      // A4
const uint8_t IR5       = PA6;      // A5
const uint8_t IR6       = PA7;      // A6
const uint8_t IR7       = PC4;      // A7
const uint8_t IR8       = PB0;      // A8
const uint8_t IR9       = PB1;      // A9
const uint8_t IR10      = PC0;      // A10
const uint8_t IR11      = PC1;      // A11
const uint8_t IR12      = PC2;      // A12
const uint8_t IR13      = PC3;      // A13


// DC MOTOR
const uint8_t M1_PWM  = PE9;      // PWM_Motor

// DC FANS
const uint8_t Fan_1     = PE11;     // PWM_FAN1         
const uint8_t Fan_2     = PE13;     // PWM_FAN2
const uint8_t Fan_3     = PC9;      // PWM_FAN31        // High Speed DC
const uint8_t Fan_3_Tacho = PC8; // PWM_FAN32
const uint8_t Fan_4     = PE5;      // PWM_FAN41        // High Speed DC
const uint8_t Fan_4_Tacho  = PE6; // PWM_FAN42

// Limit Switches
const uint8_t Lim1      = PE3;      // DIN1
const uint8_t Lim2      = PD13;     // DIN2
const uint8_t Lim3      = PE7;      // DIN3
const uint8_t Lim4      = PE8;      // DIN4
const uint8_t Lim5      = PD14;     // DIN5
const uint8_t Lim6      = PE10;     // DIN6

// UART ROM boot loader

// Raspberry pi UART for ROM boot loader
const uint8_t MS_TX     = PC10;     // MS_TX
const uint8_t MS_RX     = PC11;     // MS_RX

// Extra UART
const uint8_t Extra_UART_TX = PB10; // Extra_UART_TX
const uint8_t Extra_UART_RX = PB11; // Extra_UART_RX

const uint8_t irsensorPins[] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8, IR9, IR10, IR11, IR12, IR13};
const uint8_t limitPins[] = {Lim1, Lim2, Lim3, Lim4, Lim5, Lim6};

#endif
