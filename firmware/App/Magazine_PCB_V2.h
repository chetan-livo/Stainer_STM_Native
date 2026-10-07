/*
 * Magazine_PCB_V2.h
 *
 *  Created on: December 27, 2025
 *      Author: Muhmmad Juber
 */
 
#ifndef MAGAZINE_PCB_V2_H_
#define MAGAZINE_PCB_V2_H_

#include "Arduino.h"

const String pcbName = "MagazineHolderPCBV2";
const String pcbVersion = "v2";

//BUILTIN LED 
const uint8_t LED_BUILTIN_MAG_V2 = PE11;   // In_Built_LED1
const uint8_t IR_1_23_5V_EN      = PE5;    // IR1-23 5V enable
const uint8_t MAG_LED_DATA        = PE2;    // Local magazine NeoPixel data

//Slide sensors
const uint8_t Analog_pin0  = PA0;   //S1      AIN1
const uint8_t Analog_pin1  = PA1;   //S2      AIN2
const uint8_t Analog_pin2  = PA2;   //S3      AIN3
const uint8_t Analog_pin3  = PA3;   //S4      AIN4
const uint8_t Analog_pin4  = PA4;   //S5      AIN5
const uint8_t Analog_pin5  = PA5;   //S6      AIN6
const uint8_t Analog_pin6  = PA6;   //S7      AIN7
const uint8_t Analog_pin7  = PA7;   //S8      AIN8
const uint8_t Analog_pin8  = PC0;   //S9      AIN9
const uint8_t Analog_pin9  = PC1;   //S10     AIN10
const uint8_t Analog_pin10 = PC2;   //S11     AIN11
const uint8_t Analog_pin11 = PC3;   //S12     AIN12
const uint8_t Analog_pin12 = PC4;   //S13     AIN13
const uint8_t Analog_pin13 = PC5;   //S14     AIN14
const uint8_t Analog_pin14 = PB0;   //S15     AIN15
const uint8_t Analog_pin15 = PB1;   //S16     AIN16
const uint8_t Analog_pin16 = PF3;   //S17     AIN17
const uint8_t Analog_pin17 = PF4;   //S18     AIN18
const uint8_t Analog_pin18 = PF5;   //S19     AIN19
const uint8_t Analog_pin19 = PF6;   //S20     AIN20

// Type of Magazine
const uint8_t Analog_pin20 = PF7;   // AIN21
const uint8_t Analog_pin21 = PF8;   // AIN22
const uint8_t Analog_pin22 = PF9;   // AIN23

// Magazine lock status
const uint8_t Analog_pin23 = PF10;  // AIN24

// I2C connection
const uint8_t I2C1_SDA     = PB9;   // I2C1_SDA
const uint8_t I2C1_SCL     = PB8;   // I2C1_SCL
const uint8_t I2C1_INT     = PE0;   // INT1
const uint8_t MAG_UART_TX   = PB6;   // Prog1 pin 5
const uint8_t MAG_UART_RX   = PB7;   // Prog1 pin 3

// Solenoids
const uint8_t solenoid1    = PB12;  // Solenoid1
const uint8_t solenoid2    = PC9;   // Solenoid2

const uint8_t sensorPins[] = {Analog_pin0, Analog_pin1, Analog_pin2, Analog_pin3, Analog_pin4, Analog_pin5, Analog_pin6, Analog_pin7, Analog_pin8, Analog_pin9, Analog_pin10, Analog_pin11, Analog_pin12, Analog_pin13, Analog_pin14, Analog_pin15, Analog_pin16, Analog_pin17, Analog_pin18, Analog_pin19, Analog_pin20, Analog_pin21, Analog_pin22, Analog_pin23};

#endif /* MAGAZINE_PCB_V2_H_ */

