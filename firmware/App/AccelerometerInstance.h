/*
* AccelerometerInstance.h
*
* Created on: Jan 22, 2026
* Author: Varalakshmi
*/

#ifndef ACCELEROMETER_INSTANCE_H_
#define ACCELEROMETER_INSTANCE_H_

#include <Arduino.h>
#include <Wire.h>
#include "Constants.h"

class AccelerometerInstance {
private:
    TwoWire& _wire;
    uint8_t _addr = 0x68;
    bool continuousRead = false;
    bool available_ = false;
    bool lastReadOk_ = false;

    int16_t _accel[3];      // Accel Data to store
    
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    
    bool writeReg(uint8_t reg, uint8_t val);
    bool readBytes(uint8_t startReg, uint8_t* dst, size_t len);

public:
    AccelerometerInstance(TwoWire& wire); 
    void setup(int sda, int scl, uint8_t address, uint32_t i2cHz = 400000);
    int16_t* readAcceldata();
    bool readAccelSample(int16_t out[3]);
    bool isAvailable() const { return available_; }
    bool lastReadSucceeded() const { return lastReadOk_; }
    void loop();
    void readAccelContinuous(bool enable);
    virtual ~AccelerometerInstance();
};

#ifdef Master
    #ifdef Stainer_Gantry_PCB
        extern AccelerometerInstance acc1;
        extern AccelerometerInstance acc2;
    #endif
    #ifdef Stainer_Master_PCB
        extern AccelerometerInstance acc1;
    #endif
#endif

#endif /* ACCELEROMETER_INSTANCE_H_ */
