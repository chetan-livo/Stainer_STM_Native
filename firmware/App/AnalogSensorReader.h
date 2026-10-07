#ifndef ANALOGSENSORREADER_H_
#define ANALOGSENSORREADER_H_

#include "Arduino.h"
#include "Constants.h"

class AnalogSensorReader {
private:
    static const int MEDIAN_WINDOW = 5;     // 5-sample median for IR noise rejection
    static const int HALL_SENSOR_INDEX = 23; // S24 = hall (lock) — bypass median to avoid latency

    const uint8_t* analogPins_;      // Pointer to external pin array
    int* prevAnalogStates_;          // Stores last read (median-filtered) values
    int numSensors_;
    bool sensorValueChanged_;

    // 5-sample median ring buffer per sensor (IR sensors only)
    int* medianBuffer_;              // flat [numSensors_ * MEDIAN_WINDOW]
    uint8_t* medianBufferIdx_;       // next write index per sensor
    uint8_t* medianBufferCount_;     // valid samples in buffer (until == MEDIAN_WINDOW)

    int computeMedian(int sensorIndex) const;

public:
    // Constructor now takes pin array + count
    AnalogSensorReader(const uint8_t* pins, int numSensors);
    virtual ~AnalogSensorReader();

    void setup();
    int readSensor(int sensorIndex);
    void readAllSensors();
    int getSensorValue(int sensorIndex);
    int* getAllSensorValues();
    void printAllSensorValues();
    void printSingleSensorValue(int sensorIndex);
    bool hasSensorValueChanged() const;
    void clearSensorValueChangedFlag();
};

#endif /* ANALOGSENSORREADER_H_ */
