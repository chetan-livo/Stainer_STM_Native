/*
 * I2CInstance.h
 *
 *  Created on: 21-Apr-2025
 *      Author: Varalakshmi
 */

#ifndef I2CINSTANCE_H_
#define I2CINSTANCE_H_

#include <Wire.h>
#include "Stm32SerialCompat.h"
#include "Constants.h"
#include "HeaderPCB.h"
#include "LivoCommunication.h"

class I2CInstance {
public:
    I2CInstance(TwoWire& wire); 
    I2CInstance(LivoHardwareSerial& serial);
    void setup(int sda, int scl);
    void setup(int sda, int scl, uint8_t address);
    void setupUART(uint32_t baud = MAGAZINE_UART_BAUD);
    void loop();

    void markNextPacketAsRouted();

    void handleReceive(int howMany);

    bool requestSensorData(uint8_t address, uint8_t command, uint8_t length);
    bool writeCommandData(uint8_t address, uint8_t command, const uint8_t* data, uint8_t length);
    bool hasNewData() const;
    const uint8_t* getData();
    uint8_t getDataLength() const;
    void clearPendingData();

    bool available();
    uint8_t read();

    virtual ~I2CInstance();

    uint8_t getLastResponseTarget() const { return lastResponseTarget; }
private:
    static const uint8_t transportBufferSize = 64;
    static const uint8_t uartFrameSize = transportBufferSize + 5;
    TwoWire* _wire;
    LivoHardwareSerial* _serial = nullptr;
    bool uartMode = false;
    uint8_t uartFrame[uartFrameSize] = {};
    uint8_t uartFrameLength = 0;
    uint32_t uartLastByteMs = 0;

    bool routeNextPacket = false;

    enum State { IDLE, WRITE_COMMAND, REQUESTED, RECEIVING };
    State state = IDLE;
    uint8_t commandByte = 0;

    uint32_t READ_TIMEOUT_MS = 1000;
    uint32_t stateStartMs      = 0;
    
    uint8_t targetAddress = 0;
    uint8_t bytesToRead = 0;
    uint8_t dataBuffer[transportBufferSize] = {0};
    uint8_t bytesRead = 0;
    bool newDataAvailable = false;

    uint8_t lastResponseTarget = 0;
};

#ifdef Master
    extern I2CInstance i2c1;
    extern I2CInstance i2c2;
#endif

#endif /* I2CINSTANCE_H_ */
 
