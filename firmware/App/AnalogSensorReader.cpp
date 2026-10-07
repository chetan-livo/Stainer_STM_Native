#include "AnalogSensorReader.h"
#include "SensorFault.h"
#ifdef Nozzle_Mount_PCB
#include "NozzleMountPCBV1.h"
#endif

AnalogSensorReader::AnalogSensorReader(const uint8_t* pins, int numSensors)
    : analogPins_(pins), numSensors_(numSensors), sensorValueChanged_(false)
{
    prevAnalogStates_ = new int[numSensors_];
    medianBuffer_ = new int[numSensors_ * MEDIAN_WINDOW];
    medianBufferIdx_ = new uint8_t[numSensors_];
    medianBufferCount_ = new uint8_t[numSensors_];
    for (int i = 0; i < numSensors_; ++i) {
        prevAnalogStates_[i] = 0;
        medianBufferIdx_[i] = 0;
        medianBufferCount_[i] = 0;
        for (int j = 0; j < MEDIAN_WINDOW; ++j) {
            medianBuffer_[i * MEDIAN_WINDOW + j] = 0;
        }
    }
}

AnalogSensorReader::~AnalogSensorReader() {
    delete[] prevAnalogStates_;
    delete[] medianBuffer_;
    delete[] medianBufferIdx_;
    delete[] medianBufferCount_;
}

void AnalogSensorReader::setup() {
    for (int i = 0; i < numSensors_; ++i) {
        analogReadResolution(12);
#ifdef Nozzle_Mount_PCB
        if (analogPins_[i] == IR9) {
            prevAnalogStates_[i] = -1; // Reserved digital connector; preserve IR numbering.
            continue;
        }
#endif
        pinMode(analogPins_[i], INPUT_PULLUP);
        prevAnalogStates_[i] = analogRead(analogPins_[i]);
    }
    readAllSensors();
}

// Median of up to MEDIAN_WINDOW samples in the ring buffer for one sensor.
// Uses simple insertion sort on a copy (n ≤ 5 — cheaper than any general-purpose median).
int AnalogSensorReader::computeMedian(int sensorIndex) const {
    const int n = medianBufferCount_[sensorIndex];
    if (n == 0) return 0;
    int sorted[MEDIAN_WINDOW];
    for (int i = 0; i < n; ++i) {
        sorted[i] = medianBuffer_[sensorIndex * MEDIAN_WINDOW + i];
    }
    for (int i = 1; i < n; ++i) {
        int key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = key;
    }
    return sorted[n / 2];
}

int AnalogSensorReader::readSensor(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= numSensors_) {
        return -1;
    }
#ifdef Nozzle_Mount_PCB
    if (analogPins_[sensorIndex] == IR9) return -1;
#endif
    const int raw = analogRead(analogPins_[sensorIndex]);
    if (!SensorFaultPolicy::adcValid(raw)) {
        char channel[12]; snprintf(channel,sizeof(channel),"ADC%u",(unsigned)sensorIndex+1);
        SensorFault::report("LIVO-SEN-021",channel,raw,"adc_out_of_range");
        prevAnalogStates_[sensorIndex]=-1;
        medianBufferCount_[sensorIndex]=0;
        return -1;
    }
    int filtered;

    if (sensorIndex == HALL_SENSOR_INDEX) {
        // Hall (lock) sensor — pass through unfiltered for instant lock-state detection
        filtered = raw;
    } else {
        // IR sensor — push raw into ring buffer and return median
        const int base = sensorIndex * MEDIAN_WINDOW;
        medianBuffer_[base + medianBufferIdx_[sensorIndex]] = raw;
        medianBufferIdx_[sensorIndex] = (medianBufferIdx_[sensorIndex] + 1) % MEDIAN_WINDOW;
        if (medianBufferCount_[sensorIndex] < MEDIAN_WINDOW) {
            medianBufferCount_[sensorIndex]++;
        }
        filtered = computeMedian(sensorIndex);
    }

    if (abs(prevAnalogStates_[sensorIndex] - filtered) >= 1) {
        prevAnalogStates_[sensorIndex] = filtered;
        sensorValueChanged_ = true;
    }
    return filtered;
}

void AnalogSensorReader::readAllSensors() {
    for (int i = 0; i < numSensors_; ++i) {
        readSensor(i);
    }
}

int AnalogSensorReader::getSensorValue(int sensorIndex) {
    if (sensorIndex >= 0 && sensorIndex < numSensors_) {
        return prevAnalogStates_[sensorIndex];
    }
    return -1;
}

int* AnalogSensorReader::getAllSensorValues() {
    return prevAnalogStates_;
}

void AnalogSensorReader::printAllSensorValues() {
    for (int i = 0; i < numSensors_; ++i) {
        Serial.print("A");
        Serial.print(i);
        Serial.print(":");
        Serial.print(prevAnalogStates_[i]);
        if (i < numSensors_ - 1) Serial.print(",");
    }
    Serial.println();
}

void AnalogSensorReader::printSingleSensorValue(int sensorIndex) {
    if (sensorIndex >= 0 && sensorIndex < numSensors_) {
        Serial.print("A");
        Serial.print(sensorIndex);
        Serial.print(":");
        Serial.println(prevAnalogStates_[sensorIndex]);
    }
}

bool AnalogSensorReader::hasSensorValueChanged() const {
    return sensorValueChanged_;
}

void AnalogSensorReader::clearSensorValueChangedFlag() {
    sensorValueChanged_ = false;
}
