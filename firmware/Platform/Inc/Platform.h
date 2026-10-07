#pragma once
// Native platform layer: everything the application uses from the MCU.
#include "Pins.h"
#include "Gpio.h"
#include "Timebase.h"
#include "Print.h"
#include "WString.h"
#include "Uart.h"
#include "UsbSerial.h"
#include "Analog.h"
#include "Interrupts.h"
#include "Wire.h"
#include "MotionPlanner.h"
#include "StepEngine.h"
#include "PolledStepper.h"
#include "Watchdog.h"
#include "board_identity.h"
#include "platform_irq.h"

void platformInit();     // after HAL_Init() and SystemClock_Config()

// Application entry points, called by main().
void appSetup();
void appLoop();
