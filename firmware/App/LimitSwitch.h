/*
 * LimitSwitch.h
 *
 *  Created on: May 9, 2025
 *      Author: Faisal
 */

#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_

#include "Constants.h"
#include "HeaderPCB.h"
#include "AnalogSensorReader.h"

class LimitSwitch {
public:
	LimitSwitch();
	virtual ~LimitSwitch();

	void setup();
	void loop();
	void readIRloop();
    void readLimitSwitchloop();

	int* readIR();
    int* readLimitSwitch();

	int* getSmoothedSensorValues();
    unsigned long sampleRevision() const { return sampleRevision_; }
    void setMinReadIntervalMicros(unsigned long intervalMicros);
    unsigned long getMinReadIntervalMicros() const;

private:
    unsigned long sampleRevision_ = 0;
	AnalogSensorReader* sensorReader = nullptr;

    // Members for EMA Filter
    float* filteredSensorValues = nullptr;  // Array to store the smoothed values
    int*   smoothedValues        = nullptr; // Integer version returned to caller
    float  alpha                 = 0.1f;   // Smoothing factor (0.0 to 1.0)
    bool   isFilterInitialized   = false;   // Has the filter been initialized?

    int* limitValues = nullptr;

    // Timing / rate limiting
    unsigned long lastReadMicros      = 0;      // Last time sensors were read (micros)
    unsigned long minReadIntervalMicros = 10000; // Default 10 ms; can be lowered temporarily for homing
};

extern LimitSwitch limitSwitch;

#endif /* LIMITSWITCH_H_ */
