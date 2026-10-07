/*
 * DCFan.h
 *
 *  Created on: Jan 23, 2026
 *      Author: Varalakshmi
 */

#ifndef DCFAN_H_
#define DCFAN_H_

#include "Arduino.h"
#include "Constants.h"
#include "HeaderPCB.h"
#include <HardwareTimer.h>

class DCFan {
public:
	virtual ~DCFan();

	DCFan(int fanPin);	// Normal DC Fan
	DCFan(int pwmCtrlPin, int tachPin, uintptr_t timerInstance, uint8_t timerChannel, uint32_t pwmFreqHz = 25000);

	void setup();

	void runFan(int pwmValue);
	void stopFan();

	void runHighSpeedFanPWM(uint8_t pwm255);
	void runHighSpeedFanPercent(uint8_t percent);
	void stopHighSpeedFan();

	float getHighSpeedFanSpeed(uint8_t pulsesPerRev = 2);

private:
	int _fanPin;		// Normal Fan

	int _pwmCtrlPin;	// High Speed Fan
	int _tachPin;
	uintptr_t _timerInstance;
	uint8_t _timerChannel;

	uint32_t _pwmFreqHz;
	HardwareTimer* _timer = nullptr;

	// Tach pulse counter
    volatile uint32_t _tachPulses = 0;
    uint32_t _lastSampleMs = 0;
    float _lastRpm = 0.0f;

	// Helpers
    void attachTachInterrupt();
    void detachTachInterrupt();

    // ISR routing (supports 2 instances with tach)
    static DCFan* _isr0;
    static DCFan* _isr1;
    static void tachThunk0();
    static void tachThunk1();
};

#ifdef Nozzle_Mount_PCB
	extern DCFan dcFan1;
	extern DCFan dcFan2;
	extern DCFan dcFan3;
	extern DCFan dcFan4;
#endif

#ifdef Stainer_Master_PCB
	extern DCFan dcFan1;
	extern DCFan dcFan2;
#endif

#endif /* DCFAN_H_ */