/*
* I2CInstance_Slave.h
*
*  Created on: 22-Apr-2025
*      Author: Varalakshmi
*/

#ifndef I2CINSTANCE_SLAVE_H_
#define I2CINSTANCE_SLAVE_H_

#include <Wire.h>
#include "Stm32SerialCompat.h"
#include "Constants.h"
#include "HeaderPCB.h"
#include "LivoCommunication.h"
#include "AnalogSensorReader.h"

using Callback = void (*)(const uint8_t* data, uint8_t length);

class I2CInstanceSlave {
public:
    I2CInstanceSlave(TwoWire& wire); // Pass which I2C to use (Wire, Wire1, etc.)
    I2CInstanceSlave(LivoHardwareSerial& serial);
    void setup(int sda, int scl, uint8_t slaveAddress);
    void setupUART(uint8_t slaveAddress, uint32_t baud = MAGAZINE_UART_BAUD);
    void loop();

    void setReplyData(const uint8_t* data, uint8_t length);
    bool hasNewRequest() const;
    const uint8_t* getReceivedData() const;
    uint8_t getReceivedLength() const;

    void setReceiveCallback(Callback cb);
    void handleReceive(int howMany);
    void handleRequest();

    template<typename ModelType>
    void readAndSendMappedPosition(ModelType& model);

    virtual ~I2CInstanceSlave();
    
    int* getSmoothedSensorValues();
    bool captureMagazineLedSnapshot(int outValues[24]);
    bool isMagazineLedBusIdle() const;
private:
    bool isMagazineMhCommand(uint8_t command) const;
    void setMagazineMhTrigger(bool state);
    void finishMagazineMhCommand();
    void updateLocalMagazineDetection(const int values[24]);
    void setMagazineAlert(bool asserted);

    TwoWire* _wire;
    LivoHardwareSerial* _serial = nullptr;
    bool uartMode_ = false;
    static const uint8_t uartFrameSize = 69;
    uint8_t uartFrame_[uartFrameSize] = {};
    uint8_t uartFrameLength_ = 0;
    uint32_t uartLastByteMs_ = 0;
    void serviceUART();

    uint8_t currentCommand;
    // Wire remains limited to its legacy 32-byte transactions. The explicit
    // UART targets can carry all 24 IR values or all 20 calibration values.
    static const uint8_t maxBufferSize = MAGAZINE_UART_TEST ? 64 : 32;
    uint8_t receiveBuffer[maxBufferSize] = {0};
    uint8_t receiveLength = 0;
    bool newRequest = false;

    uint8_t replyBuffer[maxBufferSize] = {0};
    uint8_t replyLength = 0;

    static Callback onReceiveCallback;

    AnalogSensorReader* sensorReader_;

    // Members for EMA Filter
    float* filteredSensorValues_;    // Array to store the smoothed values
    float alpha_;                     // Smoothing factor (0.0 to 1.0)
    bool isFilterInitialized_;        // Flag to check if the filter has been initialized

    bool lockMagFlag = false;
    bool unlockMagFlag = false;
    bool magazineMhCommandActive_ = false;
    bool magazineMhCommandNeedsLoopCompletion_ = false;
    bool magazineManualTriggerMode_ = false;
    uint8_t stableMagazineType_ = 0;
    uint8_t candidateMagazineType_ = 0;
    uint8_t candidateMagazineSamples_ = 0;
    volatile uint8_t pendingMagazineEvent_ = MAG_EVENT_NONE;
    volatile uint8_t pendingPreviousMagazineType_ = 0;
    bool magazineDetectionInitialized_ = false;
    uint16_t pendingLedThresholds_[20] = {};
    uint8_t pendingLedThresholdType_ = 0;
    uint8_t pendingLedThresholdParts_ = 0;
    bool ledThresholdsValid_ = false;

    volatile uint8_t magFwStatus_ =
        #if defined(MAGAZINE_BOOTLOADER_BUILD)
            MAGFW_STATUS_READY;
        #else
            MAGFW_STATUS_IDLE;
        #endif
    volatile uint8_t magFwError_ = MAGFW_ERR_NONE;
    volatile uint32_t magFwExpectedSize_ = 0;
    volatile uint32_t magFwReceived_ = 0;
    volatile bool magFwRebootRequested_ = false;

    #if defined(MAGAZINE_BOOTLOADER_BUILD)
    volatile uint32_t magFwExpectedCrc_ = 0;
    volatile uint16_t magFwExpectedSeq_ = 0;
    volatile bool magFwUpdateActive_ = false;
    uint32_t magFwLastActivityMs_ = 0;
    uint8_t magFwVectorGate_[8] = {};
    bool magFwVectorGateBuffered_ = false;
    volatile bool magFwCommitPending_ = false;
    volatile uint32_t magFwCommitMagic_ = 0;

    uint8_t magFwCrc8(const uint8_t* data, uint8_t len) const;
    uint32_t magFwReadU32(const uint8_t* data) const;
    uint16_t magFwReadU16(const uint8_t* data) const;
    uint32_t magFwCrc32Update(uint32_t crc, const uint8_t* data, uint32_t len) const;
    bool magFwAppLooksValid() const;
    bool magFwBufferedVectorLooksValid() const;
    bool magFwEraseApp(uint32_t size);
    bool magFwWriteFlash(uint32_t offset, const uint8_t* data, uint8_t len);
    uint32_t magFwAppCrcWithBufferedVector() const;
    uint32_t magFwAppCrc(uint32_t len) const;
    bool magFwWriteBufferedVector();
    void magFwJumpToApp();
    void magFwHandleBegin();
    void magFwHandleErase();
    void magFwHandleData();
    void magFwHandleVerify();
    void magFwHandleCommit();
    #endif

    // On-board calibration state
    bool     calibrationRequested_       = false;
    uint8_t  calibrationStatus_          = CAL_STATUS_IDLE;
    uint8_t  calibrationMagType_         = 0;
    uint16_t calibrationResults_[20]     = {};
    
};

#endif /* I2CINSTANCE_SLAVE_H_ */

