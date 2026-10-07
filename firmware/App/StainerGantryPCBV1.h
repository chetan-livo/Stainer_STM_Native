/*
 * StainerGantryPCBV1.h
 *
 *  Created on: Dec 02, 2025
 *      Author: Sasank
 */

#ifndef STAINERGANTRYPCBV1_H_
#define STAINERGANTRYPCBV1_H_

#include <Arduino.h>

const String pcbVersion = "V1";

// Stepper motors
// Gantry X 
const uint8_t GXZR_UART_TX  = PC12;     // Stepper1_TX 
const uint8_t GXZR_UART_RX  = PD2;      // Stepper1_RX
const uint8_t GXZR_EN       = PD6;      // EN1

const uint8_t GX_Step       = PD1;      // GX_MOTOR_STEP
const uint8_t GX_Dir        = PD0;      // GX_MOTOR_DIR
const uint8_t GX_SPRD       = PD5;      // GX_SPRD/PDN

// Gantry Z 
const uint8_t GZ_Step       = PE4;      // GZ_MOTOR_STEP
const uint8_t GZ_Dir        = PE3;      // GZ_MOTOR_DIR
const uint8_t GZ_SPRD       = PE5;      // GZ_SPRD/PDN

// Gantry R 
const uint8_t GR_Step       = PC3;      // GR_MOTOR_STEP 
const uint8_t GR_Dir        = PC2;      // GR_MOTOR_DIR
const uint8_t GR_SPRD       = PC4;      // GR_SPRD/PDN

// Gantry Y MOTOR1
const uint8_t GY_UART_TX    = PA2;      // Stepper2_TX
const uint8_t GY_UART_RX    = PA3;      // Stepper2_RX
const uint8_t GY_En         = PA15;     // EN2

const uint8_t GY1_Step      = PD15;     // GY1_MOTOR_STEP
const uint8_t GY1_Dir       = PD14;     // GY1_MOTOR_DIR
const uint8_t GY1_SPRD      = PC8;      // GY1_SPRD/PDN

// Gantry Y MOTOR2
const uint8_t GY2_Step      = PD11;     // GY2_MOTOR_STEP
const uint8_t GY2_Dir       = PD10;     // GY2_MOTOR_DIR
const uint8_t GY2_SPRD      = PD12;     // GY2_SPRD/PDN

// IR sensors
const uint8_t IR1           = PB0;      // A5
const uint8_t IR2           = PA4;      // A1E
const uint8_t IR3           = PA5;      // A2
const uint8_t IR4           = PA6;      // A3
const uint8_t IR5           = PA7;      // A4

// Limit switches
const uint8_t Lim1          = PA0;      // DIN1
const uint8_t Lim2          = PC0;      // DIN2
const uint8_t Lim3          = PA1;      // DIN3
const uint8_t Lim4          = PC1;      // DIN4

// RGB LEDs
// LED1
const uint8_t Red1          = PE11;     // Red1
const uint8_t Green1        = PE13;     // Green1
const uint8_t Blue1         = PE14;     // Blue1
            
//LED2
const uint8_t Red2          = PB1;      // Red2
const uint8_t Green2        = PB5;      // Green2
const uint8_t Blue2         = PB4;      // Blue2

// UARTs
// ROM bootloader UART
const uint8_t MS_TX         = PC10;     // MS_TX
const uint8_t MX_RX         = PC11;     // MS_RX

// Extra UART 
const uint8_t Extra_TX      = PC6;      // Extra_TX
const uint8_t Extra_RX      = PC7;      // Extra_RX

// Magazine UART test wiring: Holder 1 uses UART2/USART6; Holder 2 uses
// AM2/USART3. Acc2 is unavailable while this transport is selected.
const uint8_t MAG1_UART_TX  = PC6;
const uint8_t MAG1_UART_RX  = PC7;
const uint8_t MAG2_UART_TX  = PB10;
const uint8_t MAG2_UART_RX  = PB11;

// I2Cs
const uint8_t I2C3_SDA      = PC9;      // Slave_SDA
const uint8_t I2C3_SCL      = PA8;      // Slave_SCL

// I2C interrupts
const uint8_t I2C3_INT1     = PE9;      // INT1
const uint8_t I2C3_INT2     = PE10;     // INT2
const uint8_t I2C3_INT3     = PE8;      // INT3
const uint8_t I2C3_INT4     = PE7;      // INT4

// Extra Pins 
const uint8_t Extra_pin1    = PC5;      // E_Pin1-M1 LED
const uint8_t Extra_pin2    = PE6;      // E_Pin2-M2 LED

// Accelerometers
// Accelerometer 1
const uint8_t Acc1_SDA      = PB7;      // Acc1_SDA
const uint8_t Acc1_SCL      = PB8;      // Acc1_SCL
const uint8_t Acc1_INT      = PE1;      // INTERRUPT1

// Accelerometer 2
const uint8_t Acc2_SDA      = PB11;     // Acc2_SDA
const uint8_t Acc2_SCL      = PB10;     // Acc2_SCL
const uint8_t Acc2_INT      = PB12;     // INTERRUPT2

const uint8_t irsensorPins[] = {IR1, IR2, IR3, IR4, IR5};
const uint8_t limitPins[] = {Lim1, Lim2, Lim3, Lim4};

#endif /* STAINERGANTRYPCBV1_H_ */
