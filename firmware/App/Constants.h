#include "Stm32SerialCompat.h"
/*
 * Constants.h
 *
 *  Created on: Sep 10, 2024
 *      Author: Faisal
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <Arduino.h>
#include "CommunicationConstants.h"

// Stainer Master PCB    - STM32F446ZET6
// Stainer Gantry PCB    - STM32F407VET6
// Nozzle Mount PCB      - STM32F407VET6
// Magazine Holder IR PCB V2 - STM32F446ZET6

extern bool displayDebug;
extern bool ackAction;
extern bool printEncoderChange;
extern bool printloopTime;
extern volatile bool autoUpdateEncoder;
extern bool printIR;
extern bool printLimit;
extern bool startMagzinechecks;


#define BAUD_RATE 115200

#define MAGAZINE_UART_BAUD 115200


//Define Type of PCB
// Native port: the board comes from the STM32CubeIDE build configuration
// (tools/generate_cubeide_project.py); none is selected here.
//#define Stainer_Master_PCB
//#define Stainer_Gantry_PCB
//#define Nozzle_Mount_PCB
//#define Magazine1_IR_PCB
//#define Magazine2_IR_PCB
//#define Gantry_X_Hall_PCB
//#define Gantry_Z_Hall_PCB
//#define Magazine1_IR_PCB_UART
//#define Magazine2_IR_PCB_UART
//#define Stainer_Gantry_PCB_UART

#if defined(Stainer_Master_PCB) + defined(Stainer_Gantry_PCB) + defined(Nozzle_Mount_PCB) + \
    defined(Magazine1_IR_PCB) + defined(Magazine2_IR_PCB) + defined(Gantry_X_Hall_PCB) + \
    defined(Gantry_Z_Hall_PCB) + defined(Magazine1_IR_PCB_UART) + \
    defined(Magazine2_IR_PCB_UART) + defined(Stainer_Gantry_PCB_UART) != 1
#error "Select exactly one PCB target in Constants.h."
#endif

// UART targets reuse the original board feature sets and only replace the
// Gantry-to-Magazine transport. Original target names continue to use I2C.
#if defined(Stainer_Gantry_PCB_UART)
  #define Stainer_Gantry_PCB
  #define MAGAZINE_UART_TEST 1
#elif defined(Magazine1_IR_PCB_UART)
  #define Magazine1_IR_PCB
  #define MAGAZINE_UART_TEST 1
#elif defined(Magazine2_IR_PCB_UART)
  #define Magazine2_IR_PCB
  #define MAGAZINE_UART_TEST 1
#else
  #define MAGAZINE_UART_TEST 0
#endif

// For X and Y axes
static const long XY_ENCODER_COUNTS_PER_ROTATION = 40000;
static const long XY_STEP_COUNTS_PER_ROTATION = 51200;

static const long XY_HOMING_MOVE = -51200;

static const long MAX_ACCEPTABLE_ENCODER_CHANGE = 500;

const int ARRAY_MAXSIZE = 46;

#ifdef Stainer_Master_PCB
#ifndef Master
#define Master
#endif
const byte DEVICE_ID = Stainer_Master_PCB_ID;
#endif

#ifdef Stainer_Gantry_PCB
#ifndef Master
#define Master
#endif
const byte DEVICE_ID = Stainer_Gantry_PCB_ID;
#endif

#ifdef Nozzle_Mount_PCB
#ifndef Master
#define Master
#endif
const byte DEVICE_ID = Nozzle_Mount_PCB_ID;
#endif

#ifdef Magazine1_IR_PCB
#ifndef Slave
#define Slave
#define Magazine_PCB_V2
#endif
const byte DEVICE_ID = Stainer_Magzine1_PCB_ID;
#endif

#ifdef Magazine2_IR_PCB
#ifndef Slave
#define Slave
#define Magazine_PCB_V2
#endif
const byte DEVICE_ID = Stainer_Magzine2_PCB_ID;
#endif

#ifdef Gantry_X_Hall_PCB
#ifndef Slave
#define Slave
#endif
const byte DEVICE_ID = Gantry_X_Hall_PCB_ID;
#endif

#ifdef Gantry_Z_Hall_PCB
#ifndef Slave
#define Slave
#endif
const byte DEVICE_ID = Gantry_Z_Hall_PCB_ID;
#endif

// Firmware versions are intentionally defined per PCB target. They may be
// equal for the current release, but each target advances independently when
// its own firmware changes. The OTA device-software version is maintained by
// the ESP32 release manifest and is not derived from these component versions.
#if defined(Stainer_Master_PCB)
const String FWverNo = "V2.6.0";
#elif defined(Stainer_Gantry_PCB)
const String FWverNo = "V2.3.0-rc5";
#elif defined(Nozzle_Mount_PCB)
const String FWverNo = "V2.5.0";
#elif defined(Magazine1_IR_PCB)
const String FWverNo = "V0.7.3";
#elif defined(Magazine2_IR_PCB)
const String FWverNo = "V0.7.3";
#elif defined(Gantry_X_Hall_PCB)
const String FWverNo = "V0.7.2";
#elif defined(Gantry_Z_Hall_PCB)
const String FWverNo = "V0.8.0-rc5";
#else
#error "Firmware version is not defined for the selected PCB target."
#endif
const String FWver = "FW:" + FWverNo;

// Per-image build identity. Release automation may override this macro with
// a reproducible UTC timestamp and source revision.
#ifndef LIVO_BUILD_STAMP
#define LIVO_BUILD_STAMP __DATE__ " " __TIME__
#endif
const String FWbuildNo = LIVO_BUILD_STAMP;

// ---- Magazine type detection (S21/S22/S23 on holder PCB) ----
// IR sensor index 20=S21, 21=S22, 22=S23
// below threshold = bit 1, above = bit 0
// binary S23 S22 S21 → S21 active (001) = MAG001 (type 1), S22 active (010) = MAG002 (type 2), 000 = none
static const long MAG_TYPE_S21_THRESHOLD = 3500;  // midpoint 3014(T1) / 4006(T2), ~500 margin each side
static const long MAG_TYPE_S22_THRESHOLD = 2500;  // midpoint 1921(T2) / 3222(T1), ~500 margin each side
static const long MAG_TYPE_S23_THRESHOLD = 2500;  // S23 ~2745 for both types — not used for detection

// ---- Calibration error margins ----
// CAL_ERROR_MARGIN[holderIndex][slot]  (0=Holder1, 1=Holder2)
// Subtracted from raw averaged calibration value before printing.
// Adjust per sensor to compensate for mechanical/optical offsets.
static const int CAL_ERROR_MARGIN[2][20] = {
  // Holder 1: S1 .. S20
  { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 },
  //Holder 2: S1 .. S20
  { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 100 },
};

// ---- Slide presence thresholds ----
// Gantry LOAD still uses calibration stored in ESP32 NVS via SETCAL<H><T>.
// Holder LEDs are independent: each holder learns empty-slot references locally and
// keeps them in RAM, so removal/reinsert works without blocking live I2C with flash writes.

// =============================================================================
// MirrorSerial — Stream subclass that writes to two underlying streams at once.
// Visible on every PCB build that defines `Master`. The `#define Serial mirroredSerial`
// macro is NOT applied globally (it would break LivoCommunication.cpp which uses
// USBSerial-specific Serial.begin() and !Serial). Apply it per-file at the top
// of any .cpp that wants its Serial.print() output mirrored.
//
// On Gantry/Nozzle PCB:  mirroredSerial = (Serial USB, MasterSerial UART → master)
// On Stainer Master PCB: mirroredSerial = (Serial USB, PiSerial UART     → ESP32/Pi)
// =============================================================================
#ifdef Master
class MirrorStream : public Stream {
public:
    Stream& _a;
    Stream& _b;
    MirrorStream(Stream& aRef, Stream& bRef) : _a(aRef), _b(bRef) {}
    int available() override { return _a.available(); }
    int read()      override { return _a.read(); }
    int peek()      override { return _a.peek(); }
    void flush()    override { _a.flush(); _b.flush(); }
    size_t write(uint8_t c) override { _b.write(c); if (_a.availableForWrite()>0) _a.write(c); return 1; }
    size_t write(const uint8_t* buf, size_t size) override { _b.write(buf, size); if ((size_t)_a.availableForWrite()>=size) _a.write(buf, size); return size; }
    using Print::write;
};
extern MirrorStream mirroredSerial;
#endif

#if defined(Master) && (defined(Stainer_Gantry_PCB) || defined(Nozzle_Mount_PCB))
extern LivoHardwareSerial MasterSerial;
#endif

#if defined(Master) && defined(Stainer_Master_PCB)
extern LivoHardwareSerial PiSerial;
#endif

#endif /* CONSTANTS_H_ */
