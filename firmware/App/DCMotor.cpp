/*
 * DCMotor.cpp
 *
 *  Created on: Sep 13, 2024
 *      Author: Varalakshmi
 */

#include "DCMotor.h"

#ifdef Nozzle_Mount_PCB
    DCMotor dcMotor(M1_PWM);
#endif

#ifdef Stainer_Master_PCB
    DCMotor mixdc1(M_Motor1);
    DCMotor mixdc2(M_Motor2);
    DCMotor draindc(D_Motor);
    DCMotor washdc1(W_Motor1);
    DCMotor washdc2(W_Motor2);
    DCMotor suctiondc1(S_Motor1);
    DCMotor suctiondc2(S_Motor2);
#endif

DCMotor::DCMotor(int pwmPin) {
    _pwmPin = pwmPin;
}

void DCMotor::setup() {
    pinMode(_pwmPin, OUTPUT);
    _lastValue = -1; // pinMode() detaches any running PWM.
}

void DCMotor::write(int pwmValue) {
    if (pwmValue == _lastValue) return;
    analogWrite(_pwmPin, pwmValue);
    _lastValue = pwmValue;
}

void DCMotor::runMotor(int pwmValue) {
    write(pwmValue);
}

void DCMotor::stopMotor() {
    write(0);
}

DCMotor::~DCMotor() {
	// TODO Auto-generated destructor stub
}