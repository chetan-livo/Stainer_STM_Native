/*
 * DCMotor.h
 *
 *  Created on: Sep 13, 2024
 *      Author: Varalakshmi
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "Arduino.h"
#include "Constants.h"
#include "HeaderPCB.h"

class DCMotor {
public:
	virtual ~DCMotor();

	DCMotor(int pwmPin);

	void setup();

	void runMotor(int pwmValue);

	void stopMotor();

private:
	int _pwmPin;
	// Last value written. analogWrite() reconfigures the timer on every call and
	// the master bed re-applies all window outputs after each step ACK.
	int _lastValue = -1;
	void write(int pwmValue);
};

#ifdef Nozzle_Mount_PCB
	extern DCMotor dcMotor;
#endif

#ifdef Stainer_Master_PCB
	extern DCMotor mixdc1;
	extern DCMotor mixdc2;
	extern DCMotor draindc;
	extern DCMotor washdc1;
	extern DCMotor washdc2;
	extern DCMotor suctiondc1;
	extern DCMotor suctiondc2;
#endif

#endif /* DCMOTOR_H_ */