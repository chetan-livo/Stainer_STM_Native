/*
* StageXY_Connections.h
*
*  Created on: March 05, 2025
*      Author: Sravya
*/

#ifndef StageXY_Connections_H_
#define StageXY_Connections_H_

#include "Arduino.h"

const String pcbName = "StageXY_Connections_";
const String pcbVersion = "v1";

//BUILTIN LED 
const uint8_t LED_BUILTINStageXY = PC6; //D2

//HALL SENSORS
const uint8_t Analog_pin0 = PA0; //S7
const uint8_t Analog_pin1 = PA1; //S4
const uint8_t Analog_pin2 = PA2; //S5
const uint8_t Analog_pin3 = PA3; //S6
const uint8_t Analog_pin4 = PA4; //S1
const uint8_t Analog_pin5 = PA5; //S3
const uint8_t Analog_pin6 = PA6; //S2
const uint8_t Analog_pin7 = PA7; //S9
const uint8_t Analog_pin8 = PC4; //S8
const uint8_t Analog_pin9 = PC5; //S10

//COMMUNICATION LINES
const uint8_t I2C1_SCL = PB8; //Serial Clock
const uint8_t I2C1_SDA = PB9; //Serial Data

const uint8_t sensorPins[] = {Analog_pin0, Analog_pin1, Analog_pin2, Analog_pin3, Analog_pin4, Analog_pin5, Analog_pin6, Analog_pin7, Analog_pin8, Analog_pin9};

#endif /* StageXY_Connections_H_ */