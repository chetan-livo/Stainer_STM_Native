/*
 * AccelerometerInstance.cpp
 *
 *  Created on: Jan 22, 2026
 *      Author: Varalakshmi
 */

#include "AccelerometerInstance.h"
#include "SensorFault.h"

AccelerometerInstance::AccelerometerInstance(TwoWire& wire) : _wire(wire) {
    _accel[0] = _accel[1] =_accel[2] = 0;
}

void AccelerometerInstance::setup(int sda, int scl, uint8_t address, uint32_t i2cHz) {
    _addr = address;
    available_ = false;
    lastReadOk_ = false;

    // I2C bus recovery: if SDA is stuck LOW (e.g. MPU6050 held bus from previous reset),
    // toggle SCL up to 9 times as GPIO to free the slave before handing control to Wire.
    pinMode(scl, OUTPUT);
    pinMode(sda, INPUT_PULLUP);
    for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
        digitalWrite(scl, HIGH); delayMicroseconds(5);
        digitalWrite(scl, LOW);  delayMicroseconds(5);
    }
    // Generate STOP condition (SDA LOW→HIGH while SCL HIGH)
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
    digitalWrite(sda, HIGH); delayMicroseconds(5);

    _wire.setSDA(sda);
    _wire.setSCL(scl);
    _wire.begin();
    _wire.setClock(i2cHz);

    // Probe: check device is present before writing — avoids deadlock when absent.
    _wire.beginTransmission(_addr);
    uint8_t err = _wire.endTransmission(true);
    if (err != 0) {
        SensorFault::report("LIVO-SEN-005",_addr==0x68?"MPU68":"MPU69",err,"probe_nack");
        Serial.print("MPU6050 0x"); Serial.print(_addr, HEX);
        Serial.print(" not found (I2C err "); Serial.print(err); Serial.println("), skipping.");
        return;
    }

    // Wake up MPU6050
    if (!writeReg(REG_PWR_MGMT_1, 0)) {
        SensorFault::report("LIVO-SEN-005",_addr==0x68?"MPU68":"MPU69",-1,"wake_failed");
        Serial.print("MPU6050 0x"); Serial.print(_addr, HEX);
        Serial.println(" wake-up failed.");
        return;
    }
    delay(10);
    available_ = true;
    Serial.print("0x");
    Serial.print(_addr, HEX);
    Serial.println(" MPU6050 Initialized.");
}

int16_t* AccelerometerInstance::readAcceldata() {
    if (readAccelSample(_accel)) {
        Serial.print(_accel[0]); Serial.print(", ");
        Serial.print(_accel[1]); Serial.print(", ");
        Serial.print(_accel[2]);
    }

    return lastReadOk_ ? _accel : nullptr;
}

bool AccelerometerInstance::readAccelSample(int16_t out[3]) {
    if (out == nullptr || !available_) {
        SensorFault::report("LIVO-SEN-005",_addr==0x68?"MPU68":"MPU69",-1,"unavailable");
        lastReadOk_ = false;
        return false;
    }

    uint8_t buf[6];
    if (!readBytes(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        SensorFault::report("LIVO-SEN-006",_addr==0x68?"MPU68":"MPU69",-1,"bus_read_failed");
        lastReadOk_ = false;
        return false;
    }

    _accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    _accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    _accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    if (!SensorFaultPolicy::accelValid(_accel[0],_accel[1],_accel[2])) {
        lastReadOk_=false;
        SensorFault::report("LIVO-SEN-008",_addr==0x68?"MPU68":"MPU69",_accel[2],"zero_or_saturated_sample");
        return false;
    }
    out[0] = _accel[0];
    out[1] = _accel[1];
    out[2] = _accel[2];
    lastReadOk_ = true;
    return true;
}

bool AccelerometerInstance::writeReg(uint8_t reg, uint8_t val)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    return (_wire.endTransmission(true) == 0);
}

bool AccelerometerInstance::readBytes(uint8_t startReg, uint8_t* buf, size_t len)
{
    _wire.beginTransmission(_addr);
    _wire.write(startReg);
    if (_wire.endTransmission(false) != 0) {   
        Serial.print("I2C endTransmission error = ");
        return false;
    }

    size_t got = _wire.requestFrom((int)_addr, (int)len, (int)true);
    if (got != len) return false;

    for (size_t i = 0; i < len; i++) {
        buf[i] = _wire.read();
    }
    return true;
}

AccelerometerInstance::~AccelerometerInstance() {
	// TODO Auto-generated destructor stub
}

void AccelerometerInstance::readAccelContinuous(bool enable) {
    continuousRead = enable;
}

void AccelerometerInstance::loop() {
    if (continuousRead) {
        readAcceldata();
        Serial.println();
    }
}
