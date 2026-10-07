#pragma once
// The Arduino build aliased the STM32 core's UART class; natively it is Uart.
#include "Arduino.h"
using LivoHardwareSerial = Uart;
using HardwareSerial = Uart;
