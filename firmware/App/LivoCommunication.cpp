#include "Stm32SerialCompat.h"
/*
 * LivoCommunication.cpp
 *
 *  Created on: Mar 28, 2024
 *      Author: Faisal
 */

#include "LivoCommunication.h"
#include "ServiceDiagnostics.h"
#include "LoopStats.h"
#ifdef Master
#include "StepperMotor.h"
#endif
#ifdef Gantry_Z_Hall_PCB
extern bool zHallDiagnosticCommand(const String& command);
#endif
#if defined(Gantry_X_Hall_PCB) || defined(Gantry_Z_Hall_PCB)
#include "HeaderPCB.h"
#include "I2CBusDiagnostics.h"
#endif
#ifdef Nozzle_Mount_PCB
extern bool nozzleBedInput(const String& command);
extern bool nozzleBedCommandAllowed(const String& command);
#endif

const uint8_t Communication_DELAY = 1;
byte inputBytes[ARRAY_MAXSIZE] = { };
bool inputBytesAvailable = false;
byte serialEventindex = 0;
bool isprotocol2024Flag = true;
bool runOnce = true;

const unsigned char* getinputBytes() {

	return inputBytes;
}

void setinputBytes(byte inputArray[], int arraySize) {

}

LivoCommunication::LivoCommunication() {
	inputBytesAvailable = false;
	startTimeCommunication = 0;
}

void LivoCommunication::init() {
	Serial.begin(BAUD_RATE);

	unsigned long startTime = millis();
    while (!Serial && millis() - startTime < 1000) { // wait max 1 second
        ; 
    }
	#ifdef Master
	    Serial.println("Device Started: Master");
    #endif

    #ifdef Slave
        Serial.println("Device Started: Slave");
    #endif

	Serial.print("Device ID: ");
	Serial.println(DEVICE_ID, HEX);
	Serial.println(FWver);

}

byte LivoCommunication::calculateCheckSum8bit(const byte dataArray[], int length) {
	int sum = 0;
	for (int index = 0; index < length; index++) {
		sum += dataArray[index];
	}
	return (byte) sum;
}

bool LivoCommunication::getinputBytesAvailable() {
    if (inputBytesAvailable) return true;
    byte frame[ARRAY_MAXSIZE];
    size_t length = 0;
    if (!inbox.pop(frame, length)) return false;
    String command;
    for (size_t i = 0; i < length; ++i) command += (char)frame[i];
    command.trim();
#ifdef Gantry_Z_Hall_PCB
    if(zHallDiagnosticCommand(command))return false;
#endif
#if defined(Gantry_X_Hall_PCB) || defined(Gantry_Z_Hall_PCB)
    if (command == "I2CSTAT") {
        Serial.print("I2CSTAT DEVICE_ID=0x"); Serial.print(DEVICE_ID, HEX);
        Serial.print(" EXPECTED7=0x"); Serial.println(DEVICE_ID & 0x7F, HEX);
        printI2cBusDiagnostics(Wire, Serial, "HALL PB9/PB8", I2C1_SDA, I2C1_SCL);
        return false;
    }
#endif
    if (command == "COMMSTAT") {
#ifdef Master
        mirroredSerial.print("COMMSTAT REJECTED:"); mirroredSerial.print(inbox.rejected);
        mirroredSerial.print(" EVICTED_QUERIES:"); mirroredSerial.print(inbox.evictedQueries);
        mirroredSerial.print(" OVERSIZE:"); mirroredSerial.print(oversizedLines);
        mirroredSerial.print(" SKIPPED_QUERIES:"); mirroredSerial.println(deferredQueries);
#endif
        return false;
    }
    if (command == "LOOPSTAT") {
#ifdef Master
        LoopStats::report(mirroredSerial);
#else
        LoopStats::report(Serial);
#endif
        return false;
    }
#ifdef Master
    // STEPISR [0|1]: report or switch interrupt-driven stepping (idle only).
    if (command == "STEPISR" || command == "STEPISR 0" || command == "STEPISR 1") {
        if (command.length() > 7 && !StepperMotor::setAllInterruptStepping(command.endsWith("1")))
            mirroredSerial.println("STEPISR BUSY motor_running");
        mirroredSerial.print("STEPISR MOTORS:"); mirroredSerial.println(StepperMotor::interruptSteppingCount());
        return false;
    }
#endif
    if (serviceProductionBusy() && serviceBackgroundCommand(command)) {
        ++deferredQueries;
        return false;
    }
    if (ServiceDiagnostics::consume(command)) return false;
    inputFramePacket.setBytes(frame, length);
    inputFramePacket.setSize(length);
    isprotocol2024Flag = false;
    inputBytesAvailable = true;
    return true;
}

void LivoCommunication::setinputBytesAvailable(bool input) {
	inputBytesAvailable = input;
}

bool LivoCommunication::isProtocol2024() {
    return isprotocol2024Flag;
}

void LivoCommunication::serialEvent_() {
    // Separate USB/UART assembly prevents fragments from different ports mixing.
    unsigned budget = 64;
    while (budget-- && Serial.available()) {
        byte value = Serial.read();
        receiveAscii(value, usbLine);
        if (value == '\n') break;
    }
}

void LivoCommunication::serialEventStream(LivoHardwareSerial& serial) {
    unsigned budget = 64;
    while (budget-- && serial.available()) {
        byte value = serial.read();
        receiveAscii(value, uartLine);
        if (value == '\n') break;
    }
}

void LivoCommunication::process2024Byte(byte inByte) {
    // Binary protocol reception is not implemented. A startup/noise byte must
    // not permanently latch the controller out of the ASCII command channel.
    // Discard the unsupported frame and recover at the next line boundary.
    if (inByte == '\n') {
        serialEventindex = 0;
        runOnce = true;
        isprotocol2024Flag = false;
    }
}

void LivoCommunication::process2019Byte(byte inByte) {
    receiveAscii(inByte, uartLine);
}

void LivoCommunication::receiveAscii(byte inByte, RxLine& line) {
    if (inByte == '\r') return;
    if (inByte == '\n') {
        if (line.overflow) {
            ++oversizedLines;
            line.used = 0; line.overflow = false;
            return; // Reject the whole frame; never execute a truncated suffix.
        }
        String command;
        for(size_t i=0;i<line.used;++i)command+=(char)line.data[i];
        command.trim();
        if (command.length()) {
#ifdef Nozzle_Mount_PCB
            if (&line == &uartLine && nozzleBedInput(command)) { line.used=0; return; }
            if(!nozzleBedCommandAllowed(command)){mirroredSerial.println("[BED] busy: command rejected");line.used=0;return;}
#endif
            if (serviceProductionBusy() && serviceBackgroundCommand(command)) ++deferredQueries;
            else inbox.push((const uint8_t*)command.c_str(), command.length(), serviceBackgroundCommand(command));
        }
        line.used = 0;
    } else if (!line.overflow) {
        if (line.used == ARRAY_MAXSIZE || (inByte < 32 && inByte != '\t') || inByte >= 127)
            line.overflow = true;
        else line.data[line.used++] = inByte;
    }
}


LivoCommunication::~LivoCommunication() {
}

long LivoCommunication::getStartTimeCommunication() {
	return startTimeCommunication;
}
void LivoCommunication::setStartTimeCommunication(long value) {
	startTimeCommunication = value;
}
