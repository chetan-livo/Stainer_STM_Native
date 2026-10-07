/*
 * LimitSwitch.cpp
 *
 *  Created on: May 9, 2025
 *      Author: Faisal
 */

#include "LimitSwitch.h"

// Mirror Serial.print(...) on every Master build (gantry/nozzle → MasterSerial, master → PiSerial)
#ifdef Master
#define Serial mirroredSerial
#endif

#ifdef Master

LimitSwitch limitSwitch;

int numirSensors = sizeof(irsensorPins) / sizeof(irsensorPins[0]);
int numlimits = sizeof(limitPins) / sizeof(limitPins[0]);

LimitSwitch::LimitSwitch() {
	// TODO Auto-generated constructor stub

}

LimitSwitch::~LimitSwitch() {
	// TODO Auto-generated destructor stub
	if (sensorReader) {
        delete sensorReader;
        sensorReader = nullptr;
    }

    if (filteredSensorValues) {
        delete[] filteredSensorValues;
        filteredSensorValues = nullptr;
    }

    if (smoothedValues) {
        delete[] smoothedValues;
        smoothedValues = nullptr;
    }

    if(limitValues) {
        delete[] limitValues;
        limitValues = nullptr;
    }
}

void LimitSwitch::setup() {
	sensorReader = new AnalogSensorReader(irsensorPins, numirSensors);

    if (sensorReader) {
        sensorReader->setup();
    }

    // Limit Switches setup
    for (int i = 0; i < numlimits; ++i) {
        pinMode(limitPins[i], INPUT_PULLUP);
    }
    
    // Allocate and initialize memory for the filter and output arrays
    filteredSensorValues = new float[numirSensors];
    smoothedValues       = new int[numirSensors];
    limitValues          = new int[numlimits];

    for (int i = 0; i < numirSensors; ++i) {
        filteredSensorValues[i] = 0.0f;
        smoothedValues[i]       = 0;
    }

    for (int i = 0; i < numlimits; ++i) {
        limitValues[i] = 0;
    }

    isFilterInitialized = false;
    lastReadMicros      = 0;
}

int* LimitSwitch::getSmoothedSensorValues() {
    // Safety check
    if (!sensorReader || !filteredSensorValues || !smoothedValues) {
        return nullptr;
    }

    unsigned long now = micros();

    // Enforce minimum read interval.
    // If it's too soon AND we have valid previous data, just return it.
    if (isFilterInitialized && (now - lastReadMicros < minReadIntervalMicros)) {
        return smoothedValues;
    }

    lastReadMicros = now; // update timestamp for this read

    // Perform sensor read
    sensorReader->readAllSensors();
    int* rawValues = sensorReader->getAllSensorValues();

    if (!rawValues) {
        // If something went wrong, just return the previous smoothed values
        return smoothedValues;
    }

    ++sampleRevision_;
    // Initialize filter on first call
    if (!isFilterInitialized) {
        for (int i = 0; i < numirSensors; ++i) {
            filteredSensorValues[i] = static_cast<float>(rawValues[i]);
        }
        isFilterInitialized = true;
    } else {
        // Apply exponential moving average (EMA) smoothing
        for (int i = 0; i < numirSensors; ++i) {
            if(rawValues[i]<0) filteredSensorValues[i]=-1;
            else if(filteredSensorValues[i]<0) filteredSensorValues[i]=rawValues[i];
            else filteredSensorValues[i] =
                (alpha * static_cast<float>(rawValues[i])) +
                ((1.0f - alpha) * filteredSensorValues[i]);
        }
    }

    // Convert filtered floats to integer output
    for (int i = 0; i < numirSensors; ++i) {
        // Never smooth an invalid sample into a plausible measurement.
        if (rawValues[i] < 0) { smoothedValues[i]=-1; filteredSensorValues[i]=-1; }
        else {
            if (filteredSensorValues[i] < 0) filteredSensorValues[i]=rawValues[i];
            smoothedValues[i] = static_cast<int>(roundf(filteredSensorValues[i]));
        }
    }

    return smoothedValues;
}

void LimitSwitch::setMinReadIntervalMicros(unsigned long intervalMicros) {
    minReadIntervalMicros = intervalMicros;
}

unsigned long LimitSwitch::getMinReadIntervalMicros() const {
    return minReadIntervalMicros;
}

void LimitSwitch::loop() {
	readIRloop();
    readLimitSwitchloop();
	static unsigned long lastUpdateMicros = 0;
    unsigned long now = micros();

    // Run every 10 ms — 1ms interval caused analogRead() to block AccelStepper at high step rates
    if (now - lastUpdateMicros >= minReadIntervalMicros) {
        lastUpdateMicros = now;
        getSmoothedSensorValues();
    }
}

int* LimitSwitch::readIR(){
    int* values = getSmoothedSensorValues();
    for (int i = 0; i < numirSensors; ++i) {
        Serial.print("IR");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(values[i]);
        Serial.print(", ");
    }
    return values;
}

int* LimitSwitch::readLimitSwitch() {
    for (int i = 0; i < numlimits; ++i) {
        limitValues[i] = digitalRead(limitPins[i]);

        Serial.print("L");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(limitValues[i]);
        Serial.print(", ");
    }
    return limitValues;
}

void LimitSwitch::readIRloop() {
	if (printIR) {
        readIR();
        Serial.println();
    }
}

void LimitSwitch::readLimitSwitchloop() {
    if (printLimit) {
        readLimitSwitch();
        Serial.println();
    }
}
#endif
