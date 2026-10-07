#include "Constants.h"

// Keep Magazine Holder IR PCB 1/2 on the supplied stable class definition while
// leaving the current API available to every other slave target.
#if defined(Magazine_PCB_V2)
#include "I2CInstance_Slave_StableHeader.inc"
#else

// Retained only for the unchanged non-magazine slave implementation below.
#ifndef CMD_MAGFW_BEGIN
#define CMD_MAGFW_BEGIN          0x20
#define CMD_MAGFW_DATA           0x21
#define CMD_MAGFW_STATUS         0x22
#define CMD_MAGFW_VERIFY         0x23
#define CMD_MAGFW_COMMIT         0x24
#define CMD_MAGFW_ABORT          0x25
#define CMD_MAGFW_REBOOT         0x26
#define CMD_MAGFW_ERASE          0x27
#define MAGFW_APP_BASE_ADDRESS   0x08020000UL
#define MAGFW_STATUS_IDLE        0
#define MAGFW_STATUS_READY       1
#define MAGFW_STATUS_RECEIVING   2
#define MAGFW_STATUS_VERIFYING   3
#define MAGFW_STATUS_COMPLETE    4
#define MAGFW_STATUS_ERROR       5
#define MAGFW_STATUS_UNSUPPORTED 6
#define MAGFW_STATUS_ERASING     7
#define MAGFW_ERR_NONE           0
#define MAGFW_ERR_BAD_STATE      1
#define MAGFW_ERR_BAD_ARGS       2
#define MAGFW_ERR_BAD_SEQ        3
#define MAGFW_ERR_CRC            4
#define MAGFW_ERR_FLASH          5
#define MAGFW_ERR_VERIFY         6
#define MAGFW_ERR_UNSUPPORTED    7
#endif

/*
* I2CInstance_Slave.h
*
*  Created on: 22-Apr-2025
*      Author: Varalakshmi
*/

#ifndef I2CINSTANCE_SLAVE_H_
#define I2CINSTANCE_SLAVE_H_

#include <Wire.h>
#include "Constants.h"
#include "HeaderPCB.h"
#include "LivoCommunication.h"
#include "AnalogSensorReader.h"

using Callback = void (*)(const uint8_t* data, uint8_t length);

class I2CInstanceSlave {
public:
    I2CInstanceSlave(TwoWire& wire); // Pass which I2C to use (Wire, Wire1, etc.)
    void setup(int sda, int scl, uint8_t slaveAddress);
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
    const int* getCachedSensorValues() const;
    bool hasCachedSensorValues() const;
    bool isBusIdle() const;
    #ifdef Gantry_Z_Hall_PCB
    void queueZHallDiagnostic();
    #endif
private:
    void startBus();
    void clearBusTransactionState();
    void serviceBusWatchdog();
    void recoverBus();
    int getCachedReplySensorValue(uint8_t sensorIndex, bool filtered) const;
    bool isMagazineMhCommand(uint8_t command) const;
    void setMagazineMhTrigger(bool state);
    void finishMagazineMhCommand();
    void updateLocalMagazineDetection();
    void setMagazineAlert(bool asserted);

    TwoWire& _wire;
    int sdaPin_ = -1;
    int sclPin_ = -1;
    uint8_t busAddress_ = 0;
    bool busStarted_ = false;
    bool busLowTiming_ = false;
    unsigned long busLowSinceMs_ = 0;

    volatile uint8_t currentCommand = 0;
    static const uint8_t maxBufferSize = 32;
    static_assert(maxBufferSize >= 29, "MAGFW DATA packet must fit the I2C slave buffer");
    uint8_t receiveBuffer[maxBufferSize] = {0};
    volatile uint8_t receiveLength = 0;
    volatile bool newRequest = false;
    volatile bool replyPending_ = false;
    volatile unsigned long commandReceivedMs_ = 0;

    uint8_t replyBuffer[maxBufferSize] = {0};
    uint8_t replyLength = 0;

    static Callback onReceiveCallback;

    AnalogSensorReader* sensorReader_;

    // Members for EMA Filter
    float* filteredSensorValues_;    // Array to store the smoothed values
    float alpha_;                     // Smoothing factor (0.0 to 1.0)
    bool isFilterInitialized_;        // Flag to check if the filter has been initialized
    int smoothedSensorValues_[24] = {};
    int cachedSensorValues_[24] = {};
    bool cachedSensorValuesReady_ = false;
    unsigned long localSensorSampleMs_ = 0;
    uint8_t stableMagazineType_ = 0;
    uint8_t candidateMagazineType_ = 0;
    uint8_t candidateMagazineSamples_ = 0;
    volatile uint8_t pendingMagazineEvent_ = MAG_EVENT_NONE;
    volatile uint8_t pendingPreviousMagazineType_ = 0;
    bool magazineDetectionInitialized_ = false;

    volatile bool lockMagFlag = false;
    volatile bool unlockMagFlag = false;
    volatile bool magazineMhCommandActive_ = false;
    volatile bool magazineMhCommandNeedsLoopCompletion_ = false;
    volatile bool magazineManualTriggerMode_ = false;

    // On-board calibration state
    volatile bool     calibrationRequested_ = false;
    volatile uint8_t  calibrationStatus_    = CAL_STATUS_IDLE;
    uint8_t  calibrationMagType_         = 0;
    uint16_t calibrationResults_[20]     = {};

    #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
    volatile uint8_t magFwStatus_        = MAGFW_STATUS_READY;
    #else
    volatile uint8_t magFwStatus_        = MAGFW_STATUS_IDLE;
    #endif
    volatile uint8_t  magFwError_        = MAGFW_ERR_NONE;
    volatile uint32_t magFwExpectedSize_ = 0;
    volatile uint32_t magFwReceived_     = 0;
    volatile bool magFwRebootRequested_  = false;

    #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
    volatile uint32_t magFwExpectedCrc_  = 0;
    volatile uint16_t magFwExpectedSeq_  = 0;
    volatile bool     magFwUpdateActive_ = false;
    uint32_t magFwBootStartedMs_         = 0;
    volatile uint32_t magFwLastActivityMs_ = 0;
    uint8_t  magFwVectorGate_[8]         = {};
    bool     magFwVectorGateBuffered_    = false;

    // Latched by handleReceive() (I2C ISR) so a COMMIT request survives even if a
    // later transaction overwrites the shared receive buffer before loop() runs.
    volatile bool     magFwCommitPending_ = false;
    volatile bool     magFwCommitArgsOk_  = false;
    volatile uint32_t magFwCommitMagic_   = 0;

    uint8_t  magFwCrc8(const uint8_t* data, uint8_t len) const;
    uint32_t magFwReadU32(const uint8_t* p) const;
    uint16_t magFwReadU16(const uint8_t* p) const;
    bool     magFwAppLooksValid() const;
    bool     magFwBufferedVectorLooksValid() const;
    uint32_t magFwCrc32Update(uint32_t crc, const uint8_t* data, uint32_t len) const;
    uint32_t magFwAppCrc(uint32_t len) const;
    uint32_t magFwAppCrcWithBufferedVector() const;
    uint32_t magFwSectorForAddress(uint32_t address) const;
    bool     magFwEraseApp(uint32_t size);
    bool     magFwWriteFlash(uint32_t offset, const uint8_t* data, uint8_t len);
    bool     magFwWriteBufferedVector();
    void     magFwJumpToApp();
    void     magFwHandleBegin();
    void     magFwHandleErase();
    void     magFwHandleData();
    void     magFwHandleVerify();
    void     magFwHandleCommit(uint32_t magic, bool argsOk);
    #endif
    
};

#endif /* I2CINSTANCE_SLAVE_H_ */

#endif // Magazine_PCB_V2 stable header selection
