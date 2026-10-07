/*
* I2CInstance_Slave.cpp
*
*  Created on: 22-Apr-2025
*      Author: Varalakshmi
*/

#include "Constants.h"

// Magazine Holder IR PCB 1/2 use the implementation from the supplied stable
// firmware. Other slave targets retain their current implementation.
#if defined(Magazine_PCB_V2)
#include "I2CInstance_Slave_Stable.inc"
#else

#include "I2CInstance_Slave.h"
#ifdef Gantry_Z_Hall_PCB
#include "ZHallCalibrationData.h"
#include "ZHallSignalFilter.h"
#include "ZHallPositionWindow.h"
#include "ZHallDiagnostic.h"
namespace {
ZHallSignal::Filter<> zHallSignalFilter;
uint32_t zHallSampleSequence = 0;
bool zHallSignalReady = false;
char zHallDiagnosticBuffer[256];
size_t zHallDiagnosticLength = 0;
uint32_t zHallDiagnosticQueuedMs = 0;
ZHallSignal::PositionWindow zHallPositionWindow;
volatile uint16_t zHallPositionWindowSpanMs = 0;
volatile uint8_t zHallPositionWindowSamples = 0;
volatile uint8_t zHallPositionStatus = ZHallPosition::Uncalibrated;
volatile int32_t zHallPositionSteps = INT32_MIN;
volatile uint16_t zHallUncertaintySteps = UINT16_MAX;
volatile uint16_t zHallPositionSequence = 0;
volatile unsigned long zHallPositionMs = 0;
}
#endif

#ifdef Magazine_PCB_V2
#include <IWatchdog.h>
#endif

#if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
#include "stm32f4xx_hal.h"

static constexpr uint32_t MAGFW_APP_BASE = MAGFW_APP_BASE_ADDRESS;
// Sector 7 (0x08060000..0x0807FFFF) stores holder-local LED calibration.
static constexpr uint32_t MAGFW_FLASH_END = 0x08060000UL;
static constexpr uint32_t MAGFW_COMMIT_MAGIC = 0x4D465743UL; // "MFWC"
// A holder with no valid app remains in the bootloader indefinitely.
static constexpr uint32_t MAGFW_BOOT_WINDOW_MS = 10000UL;
static constexpr bool MAGFW_STAY_IN_BOOTLOADER_FOR_TEST = false;
static constexpr uint8_t MAGFW_VECTOR_GATE_SIZE = 8;

__attribute__((noreturn, noinline))
static void magFwBranchToApp(uint32_t appStack, uint32_t appResetAddress) {
    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // External IRQ state and pending system exceptions belong to the bootloader.
    // None may reach the application before its Reset_Handler initializes RAM.
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    HAL_RCC_DeInit();
    HAL_DeInit();

    SCB->VTOR = MAGFW_APP_BASE;
    __set_CONTROL(0);
    __set_BASEPRI(0);
    __set_FAULTMASK(0);
    __DSB();
    __ISB();

    // Do not execute compiler-generated C/C++ code after changing MSP. The
    // application Reset_Handler reloads MSP too, but this mirrors a real reset.
    __asm volatile(
        "msr msp, %0\n"
        "dsb\n"
        "isb\n"
        "cpsie i\n"
        "bx %1\n"
        :
        : "r" (appStack), "r" (appResetAddress)
        : "memory"
    );
    __builtin_unreachable();
}
#endif

#ifdef Slave
I2CInstanceSlave* activeSlaveInstance = nullptr;
Callback I2CInstanceSlave::onReceiveCallback = nullptr;

#ifdef Gantry_Z_Hall_PCB
bool zHallDiagnosticCommand(const String& command) {
    if(command!="ZHREAD")return false;
    if(activeSlaveInstance)activeSlaveInstance->queueZHallDiagnostic();
    return true;
}
void I2CInstanceSlave::queueZHallDiagnostic() {
    // Called only in main context. Never perform ADC reads or I2C transactions.
    if(zHallDiagnosticLength)return;
    const uint32_t age=millis()-localSensorSampleMs_;
    zHallDiagnosticLength=ZHallDiagnostic::format(zHallDiagnosticBuffer,sizeof(zHallDiagnosticBuffer),
        zHallSampleSequence,localSensorSampleMs_,age,
        cachedSensorValuesReady_ && zHallSignalReady && age<50UL,
        cachedSensorValues_,zHallSignalFilter.output());
    zHallDiagnosticQueuedMs=millis();
}
#endif

#ifdef Gantry_X_Hall_PCB
    XYStagePositionModel gantryxstagepositionModel;
#endif

int numSensors = sizeof(sensorPins) / sizeof(sensorPins[0]);

// Global handler functions (required by Wire lib)
void onReceiveHandler(int howMany) {
    if (activeSlaveInstance) {
        activeSlaveInstance->handleReceive(howMany);
    }
}

void onRequestHandler() {
    if (activeSlaveInstance) {
        activeSlaveInstance->handleRequest();
    }
}

I2CInstanceSlave::I2CInstanceSlave(TwoWire& wire) 
:   _wire(wire),
    sensorReader_(nullptr),
    filteredSensorValues_(nullptr),
    alpha_(0.02f),
    isFilterInitialized_(false)
{
    activeSlaveInstance = this;
	// TODO Auto-generated constructor stub
}

I2CInstanceSlave::~I2CInstanceSlave() {
    if (sensorReader_) {
        delete sensorReader_;
        sensorReader_ = nullptr;
    }
    // Clean up filter memory
    if (filteredSensorValues_) {
        delete[] filteredSensorValues_;
        filteredSensorValues_ = nullptr;
    }
}

bool I2CInstanceSlave::isMagazineMhCommand(uint8_t command) const {
    #ifdef Magazine_PCB_V2
        switch (command) {
            case CMD_HALL_DATA:
            case CMD_1_HALL_DATA:
            case CMD_2_HALL_DATA:
            case CMD_FILTERED_HALL_DATA:
            case CMD_1_FILTERED_HALL_DATA:
            case CMD_2_FILTERED_HALL_DATA:
            case CMD_SLIDE_STATUS:
            case CMD_MAGZINE_STATUS:
            case CMD_LOCK_MAGZINE:
            case CMD_UNLOCK_MAGZINE:
            case CMD_SOFT_RESET:
            case CMD_CALIBRATE:
            case CMD_CALIBRATE_RESULT_1:
            case CMD_CALIBRATE_RESULT_2:
            case CMD_MAGAZINE_EVENT:
            case CMD_MAGFW_BEGIN:
            case CMD_MAGFW_DATA:
            case CMD_MAGFW_STATUS:
            case CMD_MAGFW_VERIFY:
            case CMD_MAGFW_COMMIT:
            case CMD_MAGFW_ABORT:
            case CMD_MAGFW_REBOOT:
            case CMD_MAGFW_ERASE:
                return true;
            default:
                return false;
        }
    #else
        (void)command;
        return false;
    #endif
}

void I2CInstanceSlave::setMagazineMhTrigger(bool state) {
    #ifdef Magazine_PCB_V2
        digitalWrite(IR_1_23_5V_EN, state ? HIGH : LOW);
    #else
        (void)state;
    #endif
}

void I2CInstanceSlave::setMagazineAlert(bool asserted) {
    #ifdef Magazine_PCB_V2
        // Active-low, open-drain interrupt. Gantry supplies the pull-up.
        digitalWrite(I2C1_INT, asserted ? LOW : HIGH);
    #else
        (void)asserted;
    #endif
}

void I2CInstanceSlave::updateLocalMagazineDetection() {
    #ifdef Magazine_PCB_V2
        if (!cachedSensorValuesReady_ || cachedSensorValues_[20]<0 || cachedSensorValues_[21]<0 || cachedSensorValues_[22]<0) return;

        const uint8_t detectedType =
            ((cachedSensorValues_[22] < MAG_TYPE_S23_THRESHOLD) ? 4U : 0U) |
            ((cachedSensorValues_[21] < MAG_TYPE_S22_THRESHOLD) ? 2U : 0U) |
            ((cachedSensorValues_[20] < MAG_TYPE_S21_THRESHOLD) ? 1U : 0U);

        if (detectedType != candidateMagazineType_) {
            candidateMagazineType_ = detectedType;
            candidateMagazineSamples_ = 1;
            return;
        }
        if (candidateMagazineSamples_ < 3) ++candidateMagazineSamples_;
        if (candidateMagazineSamples_ < 3) return;

        if (!magazineDetectionInitialized_) {
            magazineDetectionInitialized_ = true;
            stableMagazineType_ = detectedType;
            if (detectedType != 0) {
                pendingPreviousMagazineType_ = 0;
                pendingMagazineEvent_ = MAG_EVENT_INSERTED;
                setMagazineAlert(true);
            }
            return;
        }
        if (detectedType == stableMagazineType_) return;

        const uint8_t previousType = stableMagazineType_;
        stableMagazineType_ = detectedType;
        pendingPreviousMagazineType_ = previousType;
        pendingMagazineEvent_ = (detectedType == 0)
            ? MAG_EVENT_REMOVED
            : ((previousType == 0) ? MAG_EVENT_INSERTED : MAG_EVENT_SWAPPED);
        setMagazineAlert(true);
    #endif
}

void I2CInstanceSlave::finishMagazineMhCommand() {
    if (!magazineMhCommandActive_ || magazineMhCommandNeedsLoopCompletion_) {
        return;
    }

    setMagazineMhTrigger(false);
    magazineMhCommandActive_ = false;
}

#if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
uint8_t I2CInstanceSlave::magFwCrc8(const uint8_t* data, uint8_t len) const {
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= data[i];
    return c;
}

uint32_t I2CInstanceSlave::magFwReadU32(const uint8_t* p) const {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

uint16_t I2CInstanceSlave::magFwReadU16(const uint8_t* p) const {
    return ((uint16_t)p[0] << 8) | p[1];
}

bool I2CInstanceSlave::magFwAppLooksValid() const {
    uint32_t sp = *(uint32_t*)MAGFW_APP_BASE;
    uint32_t rv = *(uint32_t*)(MAGFW_APP_BASE + 4);
    return (sp >= 0x20000000UL && sp < 0x20040000UL &&
            rv >= MAGFW_APP_BASE && rv < MAGFW_FLASH_END);
}

bool I2CInstanceSlave::magFwBufferedVectorLooksValid() const {
    if (!magFwVectorGateBuffered_) return false;
    uint32_t sp = (uint32_t)magFwVectorGate_[0] |
                  ((uint32_t)magFwVectorGate_[1] << 8) |
                  ((uint32_t)magFwVectorGate_[2] << 16) |
                  ((uint32_t)magFwVectorGate_[3] << 24);
    uint32_t rv = (uint32_t)magFwVectorGate_[4] |
                  ((uint32_t)magFwVectorGate_[5] << 8) |
                  ((uint32_t)magFwVectorGate_[6] << 16) |
                  ((uint32_t)magFwVectorGate_[7] << 24);
    return (sp >= 0x20000000UL && sp < 0x20040000UL &&
            rv >= MAGFW_APP_BASE && rv < MAGFW_FLASH_END);
}

uint32_t I2CInstanceSlave::magFwCrc32Update(uint32_t crc, const uint8_t* data, uint32_t len) const {
    crc = ~crc;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
        }
    }
    return ~crc;
}

uint32_t I2CInstanceSlave::magFwAppCrc(uint32_t len) const {
    return magFwCrc32Update(0, (const uint8_t*)MAGFW_APP_BASE, len);
}

uint32_t I2CInstanceSlave::magFwAppCrcWithBufferedVector() const {
    if (!magFwVectorGateBuffered_ || magFwExpectedSize_ < MAGFW_VECTOR_GATE_SIZE) return 0;
    uint32_t crc = magFwCrc32Update(0, magFwVectorGate_, MAGFW_VECTOR_GATE_SIZE);
    if (magFwExpectedSize_ > MAGFW_VECTOR_GATE_SIZE) {
        crc = magFwCrc32Update(crc,
                               (const uint8_t*)(MAGFW_APP_BASE + MAGFW_VECTOR_GATE_SIZE),
                               magFwExpectedSize_ - MAGFW_VECTOR_GATE_SIZE);
    }
    return crc;
}

uint32_t I2CInstanceSlave::magFwSectorForAddress(uint32_t address) const {
    if (address < 0x08004000UL) return FLASH_SECTOR_0;
    if (address < 0x08008000UL) return FLASH_SECTOR_1;
    if (address < 0x0800C000UL) return FLASH_SECTOR_2;
    if (address < 0x08010000UL) return FLASH_SECTOR_3;
    if (address < 0x08020000UL) return FLASH_SECTOR_4;
    if (address < 0x08040000UL) return FLASH_SECTOR_5;
    if (address < 0x08060000UL) return FLASH_SECTOR_6;
    return FLASH_SECTOR_7;
}

bool I2CInstanceSlave::magFwEraseApp(uint32_t size) {
    if (size == 0 || MAGFW_APP_BASE + size > MAGFW_FLASH_END) return false;
    uint32_t first = magFwSectorForAddress(MAGFW_APP_BASE);
    uint32_t last = magFwSectorForAddress(MAGFW_APP_BASE + size - 1);
    FLASH_EraseInitTypeDef erase = {};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_OK;
    for (uint32_t sector = first; sector <= last; ++sector) {
        uint32_t sectorError = 0;
        erase.Sector = sector;
        IWatchdog.reload();
        st = HAL_FLASHEx_Erase(&erase, &sectorError);
        IWatchdog.reload();
        if (st != HAL_OK) break;
    }
    HAL_FLASH_Lock();
    return st == HAL_OK;
}

bool I2CInstanceSlave::magFwWriteFlash(uint32_t offset, const uint8_t* data, uint8_t len) {
    if ((offset % 4) != 0 || (len % 4) != 0) return false;
    if (MAGFW_APP_BASE + offset + len > MAGFW_FLASH_END) return false;
    HAL_FLASH_Unlock();
    for (uint8_t i = 0; i < len; i += 4) {
        uint32_t word = (uint32_t)data[i] | ((uint32_t)data[i + 1] << 8) |
                        ((uint32_t)data[i + 2] << 16) | ((uint32_t)data[i + 3] << 24);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGFW_APP_BASE + offset + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        if (*(uint32_t*)(MAGFW_APP_BASE + offset + i) != word) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

bool I2CInstanceSlave::magFwWriteBufferedVector() {
    if (!magFwBufferedVectorLooksValid()) return false;
    const uint32_t sp = (uint32_t)magFwVectorGate_[0] |
                        ((uint32_t)magFwVectorGate_[1] << 8) |
                        ((uint32_t)magFwVectorGate_[2] << 16) |
                        ((uint32_t)magFwVectorGate_[3] << 24);
    const uint32_t rv = (uint32_t)magFwVectorGate_[4] |
                        ((uint32_t)magFwVectorGate_[5] << 8) |
                        ((uint32_t)magFwVectorGate_[6] << 16) |
                        ((uint32_t)magFwVectorGate_[7] << 24);

    HAL_FLASH_Unlock();
    // Program RESET first and SP last. Until the final SP word succeeds,
    // appLooksValid() remains false after any reset or power interruption.
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGFW_APP_BASE + 4, rv) != HAL_OK ||
        *(uint32_t*)(MAGFW_APP_BASE + 4) != rv) {
        HAL_FLASH_Lock();
        return false;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGFW_APP_BASE, sp) != HAL_OK ||
        *(uint32_t*)MAGFW_APP_BASE != sp) {
        HAL_FLASH_Lock();
        return false;
    }
    HAL_FLASH_Lock();
    return true;
}

void I2CInstanceSlave::magFwJumpToApp() {
    if (!magFwAppLooksValid()) return;
    const uint32_t appStack = *(uint32_t*)MAGFW_APP_BASE;
    const uint32_t appResetAddress = *(uint32_t*)(MAGFW_APP_BASE + 4);
    _wire.end();
    magFwBranchToApp(appStack, appResetAddress);
}

void I2CInstanceSlave::magFwHandleBegin() {
    if (receiveLength < 13) { magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_ARGS; return; }
    magFwExpectedSize_ = magFwReadU32(&receiveBuffer[1]);
    magFwExpectedCrc_ = magFwReadU32(&receiveBuffer[5]);
    uint32_t base = magFwReadU32(&receiveBuffer[9]);
    if (base != MAGFW_APP_BASE || magFwExpectedSize_ < MAGFW_VECTOR_GATE_SIZE ||
        MAGFW_APP_BASE + magFwExpectedSize_ > MAGFW_FLASH_END) {
        magFwStatus_ = MAGFW_STATUS_ERROR;
        magFwError_ = MAGFW_ERR_BAD_ARGS;
        return;
    }
    memset(magFwVectorGate_, 0xFF, sizeof(magFwVectorGate_));
    magFwVectorGateBuffered_ = false;
    magFwReceived_ = 0;
    magFwExpectedSeq_ = 0;
    magFwUpdateActive_ = false;
    magFwStatus_ = MAGFW_STATUS_READY;
    magFwError_ = MAGFW_ERR_NONE;
}

void I2CInstanceSlave::magFwHandleErase() {
    if (magFwExpectedSize_ == 0 || magFwStatus_ != MAGFW_STATUS_READY) {
        magFwStatus_ = MAGFW_STATUS_ERROR;
        magFwError_ = MAGFW_ERR_BAD_STATE;
        return;
    }
    magFwReceived_ = 0;
    magFwExpectedSeq_ = 0;
    magFwUpdateActive_ = false;
    memset(magFwVectorGate_, 0xFF, sizeof(magFwVectorGate_));
    magFwVectorGateBuffered_ = false;
    magFwStatus_ = MAGFW_STATUS_ERASING;
    magFwError_ = MAGFW_ERR_NONE;
    if (!magFwEraseApp(magFwExpectedSize_)) {
        magFwUpdateActive_ = false;
        magFwStatus_ = MAGFW_STATUS_ERROR;
        magFwError_ = MAGFW_ERR_FLASH;
        return;
    }
    magFwUpdateActive_ = true;
    magFwStatus_ = MAGFW_STATUS_RECEIVING;
    magFwError_ = MAGFW_ERR_NONE;
}

void I2CInstanceSlave::magFwHandleData() {
    if (!magFwUpdateActive_ || receiveLength < 10) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_STATE; return;
    }
    uint8_t packetCrc = receiveBuffer[receiveLength - 1];
    if (magFwCrc8(receiveBuffer, receiveLength - 1) != packetCrc) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_CRC; return;
    }
    uint16_t seq = magFwReadU16(&receiveBuffer[1]);
    uint32_t offset = magFwReadU32(&receiveBuffer[3]);
    uint8_t len = receiveBuffer[7];
    if (seq != magFwExpectedSeq_ || len == 0 || len > 20 || receiveLength != (uint8_t)(9 + len)) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_ARGS; return;
    }
    if (offset != magFwReceived_ || offset + len > magFwExpectedSize_) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_SEQ; return;
    }
    bool writeOk = false;
    if (offset == 0) {
        if (len < MAGFW_VECTOR_GATE_SIZE) {
            magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_ARGS; return;
        }
        memcpy(magFwVectorGate_, &receiveBuffer[8], MAGFW_VECTOR_GATE_SIZE);
        magFwVectorGateBuffered_ = true;
        writeOk = (len == MAGFW_VECTOR_GATE_SIZE) ||
                  magFwWriteFlash(MAGFW_VECTOR_GATE_SIZE,
                                  &receiveBuffer[8 + MAGFW_VECTOR_GATE_SIZE],
                                  len - MAGFW_VECTOR_GATE_SIZE);
    } else {
        writeOk = magFwWriteFlash(offset, &receiveBuffer[8], len);
    }
    if (!writeOk) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_FLASH; return;
    }
    magFwReceived_ += len;
    magFwExpectedSeq_++;
    // Stay in RECEIVING even once every byte is in — COMPLETE is reserved for a
    // holder that has actually run CRC verification. Otherwise a dropped VERIFY
    // command would leave this stale COMPLETE looking like a real pass.
    magFwStatus_ = MAGFW_STATUS_RECEIVING;
    magFwError_ = MAGFW_ERR_NONE;
}

void I2CInstanceSlave::magFwHandleVerify() {
    // The receive phase is over the moment VERIFY is asked for, one way or another.
    // Clearing this here (rather than only on success) lets a failed verify still
    // be treated as an idle bootloader, so the boot-window fallback in loop() can
    // eventually jump to whatever app is actually valid instead of wedging forever.
    magFwUpdateActive_ = false;

    if (magFwExpectedSize_ == 0 || magFwReceived_ < magFwExpectedSize_) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_VERIFY; return;
    }
    if (magFwAppCrcWithBufferedVector() != magFwExpectedCrc_ ||
        !magFwBufferedVectorLooksValid()) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_VERIFY; return;
    }
    if (!magFwWriteBufferedVector()) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_FLASH; return;
    }
    if (magFwAppCrc(magFwExpectedSize_) != magFwExpectedCrc_ || !magFwAppLooksValid()) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_VERIFY; return;
    }
    magFwStatus_ = MAGFW_STATUS_COMPLETE;
    magFwError_ = MAGFW_ERR_NONE;
}

// magic/argsOk are captured by handleReceive() at the moment the COMMIT packet
// arrives, not re-read from the shared receive buffer here — that buffer can
// already belong to a later transaction (e.g. a STATUS poll) by the time loop()
// gets around to processing this.
void I2CInstanceSlave::magFwHandleCommit(uint32_t magic, bool argsOk) {
    if (argsOk && magic == MAGFW_COMMIT_MAGIC && magFwStatus_ == MAGFW_STATUS_COMPLETE) {
        magFwError_ = MAGFW_ERR_NONE;
        if (!MAGFW_STAY_IN_BOOTLOADER_FOR_TEST) {
            delay(20);
            magFwJumpToApp();
            // Only reached if the app image no longer looks valid.
            magFwStatus_ = MAGFW_STATUS_ERROR;
            magFwError_ = MAGFW_ERR_VERIFY;
        }
    } else {
        magFwStatus_ = MAGFW_STATUS_ERROR;
        magFwError_ = MAGFW_ERR_BAD_STATE;
    }
}
#endif

void I2CInstanceSlave::startBus() {
    _wire.setSDA(sdaPin_);
    _wire.setSCL(sclPin_);
    _wire.begin(busAddress_, false, false);
    _wire.onRequest(onRequestHandler);
    _wire.onReceive(onReceiveHandler);
    busStarted_ = true;
    busLowTiming_ = false;
}

void I2CInstanceSlave::clearBusTransactionState() {
    setMagazineMhTrigger(false);
    currentCommand = 0;
    newRequest = false;
    replyPending_ = false;
    receiveLength = 0;
    replyLength = 0;
    magazineMhCommandActive_ = false;
    magazineMhCommandNeedsLoopCompletion_ = false;
    magazineManualTriggerMode_ = false;
}

bool I2CInstanceSlave::isBusIdle() const {
    if (!busStarted_ || sdaPin_ < 0 || sclPin_ < 0) return false;
    if (newRequest || replyPending_ || magazineMhCommandActive_) return false;

    noInterrupts();
    const bool busIdle = digitalRead(sdaPin_) == HIGH && digitalRead(sclPin_) == HIGH;
    const bool transactionIdle = !newRequest && !replyPending_ && !magazineMhCommandActive_;
    interrupts();
    return busIdle && transactionIdle;
}

void I2CInstanceSlave::recoverBus() {
    if (!busStarted_) return;

    _wire.end();
    busStarted_ = false;
    clearBusTransactionState();

    // Release both lines before restarting the slave peripheral. The holder is
    // never allowed to drive SCL pulses because Gantry is the I2C bus master.
    pinMode(sdaPin_, INPUT_PULLUP);
    pinMode(sclPin_, INPUT_PULLUP);
    delayMicroseconds(50);
    startBus();
}

void I2CInstanceSlave::serviceBusWatchdog() {
    #if defined(Magazine_PCB_V2) || defined(Gantry_Z_Hall_PCB)
        if (!busStarted_ || sdaPin_ < 0 || sclPin_ < 0) {
            busLowTiming_ = false;
            return;
        }

        const unsigned long now = millis();
        if (replyPending_ && now - commandReceivedMs_ >= 500UL &&
            digitalRead(sdaPin_) == HIGH && digitalRead(sclPin_) == HIGH) {
            noInterrupts();
            if (replyPending_ && now - commandReceivedMs_ >= 500UL) {
                clearBusTransactionState();
            }
            interrupts();
        }

        const bool lineLow = digitalRead(sdaPin_) == LOW || digitalRead(sclPin_) == LOW;
        if (!lineLow) {
            busLowTiming_ = false;
            return;
        }

        if (!busLowTiming_) {
            busLowTiming_ = true;
            busLowSinceMs_ = now;
        } else if (now - busLowSinceMs_ >= 250UL) {
            recoverBus();
        }
    #endif
}

void I2CInstanceSlave::setup(int sda, int scl, uint8_t slaveAddress) {
    sdaPin_ = sda;
    sclPin_ = scl;
    busAddress_ = (slaveAddress > 0x7F) ? (slaveAddress & 0x7F) : slaveAddress;

    #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
        pinMode(IR_1_23_5V_EN, OUTPUT);
        digitalWrite(IR_1_23_5V_EN, LOW);
        pinMode(I2C1_INT, OUTPUT_OPEN_DRAIN);
        setMagazineAlert(false);
        pinMode(solenoid1, OUTPUT);
        pinMode(solenoid2, OUTPUT);
        digitalWrite(solenoid1, LOW);
        digitalWrite(solenoid2, LOW);
        magFwBootStartedMs_ = millis();
        magFwLastActivityMs_ = magFwBootStartedMs_;
        startBus();
        Serial.print("I2C slave ready at 0x");
        if (busAddress_ < 0x10) Serial.print('0');
        Serial.println(busAddress_, HEX);
        return;
    #endif

    sensorReader_ = new AnalogSensorReader(sensorPins, numSensors);

    if (sensorReader_) {
        sensorReader_->setup();
    }

    // Allocate and initialize memory for the filter
    filteredSensorValues_ = new float[numSensors];
    for (int i = 0; i < numSensors; ++i) {
        filteredSensorValues_[i] = 0.0f;
    }
    isFilterInitialized_ = false;

    #ifdef Magazine_PCB_V2
        pinMode(IR_1_23_5V_EN, OUTPUT);
        digitalWrite(IR_1_23_5V_EN, LOW);
        pinMode(I2C1_INT, OUTPUT_OPEN_DRAIN);
        setMagazineAlert(false);
        pinMode(solenoid1, OUTPUT);
        pinMode(solenoid2, OUTPUT);
        digitalWrite(solenoid1, LOW);
        digitalWrite(solenoid2, LOW);

        // Publish the slave address only after every object used by the I2C
        // callbacks exists and a complete sensor snapshot is available.
        setMagazineMhTrigger(true);
        delay(5);
        (void)getSmoothedSensorValues();
        setMagazineMhTrigger(false);
    #endif

    startBus();
    Serial.print("I2C slave ready at 0x");
    if (busAddress_ < 0x10) Serial.print('0');
    Serial.println(busAddress_, HEX);
}

// Set data to reply when master requests
void I2CInstanceSlave::setReplyData(const uint8_t* data, uint8_t length) {
    replyLength = min(length, maxBufferSize);
    memcpy(replyBuffer, data, replyLength);
}

int I2CInstanceSlave::getCachedReplySensorValue(uint8_t sensorIndex, bool filtered) const {
    if (sensorIndex >= numSensors || !cachedSensorValuesReady_) return -1;
    return filtered ? smoothedSensorValues_[sensorIndex] : cachedSensorValues_[sensorIndex];
}

template<typename ModelType>
void I2CInstanceSlave::readAndSendMappedPosition(ModelType& model) {
    int raw[ModelType::NUM_FEATURES];
    int* smoothedValues = getSmoothedSensorValues();
    for (uint8_t i = 0; i < ModelType::NUM_FEATURES; ++i) {
        raw[i] = smoothedValues[i];
    }

    Serial.print("Raw Hall sensor values: ");
    for (uint8_t i = 0; i < ModelType::NUM_FEATURES; ++i) {
        Serial.print(raw[i]);
        Serial.print(i < ModelType::NUM_FEATURES - 1 ? ", " : "\n");
    }

    int32_t mappedPosition = model.predictFromRaw(raw);

    replyBuffer[replyLength++] = (mappedPosition >> 24) & 0xFF;
    replyBuffer[replyLength++] = (mappedPosition >> 16) & 0xFF;
    replyBuffer[replyLength++] = (mappedPosition >> 8) & 0xFF;
    replyBuffer[replyLength++] = mappedPosition & 0xFF;

    Serial.print("Sending mapped position: ");
    Serial.println(mappedPosition);
}

void I2CInstanceSlave::handleRequest() {
    replyLength = 0;

    #ifdef Gantry_Z_Hall_PCB
    // The ISR only serializes a complete task-context snapshot. ADC reads,
    // filtering, prediction and USB writes must never run in this callback.
    const bool fresh = cachedSensorValuesReady_ && millis() - localSensorSampleMs_ < 100UL;
    if (currentCommand == CMD_Z_HALL_FAST_SNAPSHOT) {
        const unsigned long age=millis()-zHallPositionMs;
        const uint16_t age16=age>UINT16_MAX ? UINT16_MAX : age;
        const int32_t position=fresh && age<100UL ? zHallPositionSteps : INT32_MIN;
        replyBuffer[replyLength++]=1;
        replyBuffer[replyLength++]=fresh && age<100UL ? zHallPositionStatus : ZHallPosition::InvalidAdc;
        replyBuffer[replyLength++]=zHallPositionSequence>>8;
        replyBuffer[replyLength++]=zHallPositionSequence&0xFF;
        replyBuffer[replyLength++]=age16>>8;
        replyBuffer[replyLength++]=age16&0xFF;
        for(int shift=24;shift>=0;shift-=8) replyBuffer[replyLength++]=((uint32_t)position>>shift)&0xFF;
        replyBuffer[replyLength++]=zHallUncertaintySteps>>8;
        replyBuffer[replyLength++]=zHallUncertaintySteps&0xFF;
        replyBuffer[replyLength++]=zHallPositionWindowSpanMs>>8;
        replyBuffer[replyLength++]=zHallPositionWindowSpanMs&0xFF;
        for(int shift=24;shift>=0;shift-=8) replyBuffer[replyLength++]=(ZHallCalibrationData::id>>shift)&0xFF;
        replyBuffer[replyLength++]=zHallPositionWindowSamples;
    } else if (currentCommand == CMD_Z_HALL_SNAPSHOT) {
        const unsigned long age = millis() - zHallPositionMs;
        const uint16_t age16 = age > UINT16_MAX ? UINT16_MAX : age;
        const int32_t position = fresh && age < 250UL ? zHallPositionSteps : INT32_MIN;
        replyBuffer[replyLength++] = fresh && age < 250UL ? zHallPositionStatus : ZHallPosition::InvalidAdc;
        replyBuffer[replyLength++] = zHallPositionSequence >> 8;
        replyBuffer[replyLength++] = zHallPositionSequence & 0xFF;
        replyBuffer[replyLength++] = age16 >> 8;
        replyBuffer[replyLength++] = age16 & 0xFF;
        for (int shift = 24; shift >= 0; shift -= 8) replyBuffer[replyLength++] = ((uint32_t)position >> shift) & 0xFF;
        replyBuffer[replyLength++] = zHallUncertaintySteps >> 8;
        replyBuffer[replyLength++] = zHallUncertaintySteps & 0xFF;
        for (uint8_t i = 0; i < 10; ++i) {
            const int value = fresh ? cachedSensorValues_[i] : -1;
            replyBuffer[replyLength++] = (value >> 8) & 0xFF;
            replyBuffer[replyLength++] = value & 0xFF;
        }
    } else if (currentCommand == CMD_Z_HALL_MODEL_INFO) {
        const auto& table = ZHallCalibrationData::table;
        replyBuffer[replyLength++] = 1; // protocol version
        replyBuffer[replyLength++] = table.validated ? 1 : 0;
        const uint32_t fields[] = {ZHallCalibrationData::id,
            table.count ? (uint32_t)table.knots[0].steps : 0,
            table.count ? (uint32_t)table.knots[table.count-1].steps : 0};
        for (uint8_t i = 0; i < 3; ++i)
            for (int shift = 24; shift >= 0; shift -= 8) replyBuffer[replyLength++] = (fields[i] >> shift) & 0xFF;
        replyBuffer[replyLength++] = ZHallCalibrationData::microsteps >> 8;
        replyBuffer[replyLength++] = ZHallCalibrationData::microsteps & 0xFF;
        replyBuffer[replyLength++] = table.count >> 8;
        replyBuffer[replyLength++] = table.count & 0xFF;
    } else if (currentCommand == CMD_HALL_DATA || currentCommand == CMD_FILTERED_HALL_DATA ||
        currentCommand == CMD_1_HALL_DATA || currentCommand == CMD_2_HALL_DATA ||
        currentCommand == CMD_1_FILTERED_HALL_DATA || currentCommand == CMD_2_FILTERED_HALL_DATA) {
        const bool filtered = currentCommand == CMD_FILTERED_HALL_DATA ||
            currentCommand == CMD_1_FILTERED_HALL_DATA || currentCommand == CMD_2_FILTERED_HALL_DATA;
        const uint8_t first = (currentCommand == CMD_2_HALL_DATA || currentCommand == CMD_2_FILTERED_HALL_DATA) ? 5 : 0;
        const uint8_t end = (currentCommand == CMD_1_HALL_DATA || currentCommand == CMD_1_FILTERED_HALL_DATA) ? 5 : 10;
        for (uint8_t i = first; i < end; ++i) {
            const int value = fresh ? (filtered ? smoothedSensorValues_[i] : cachedSensorValues_[i]) : -1;
            replyBuffer[replyLength++] = (value >> 8) & 0xFF;
            replyBuffer[replyLength++] = value & 0xFF;
        }
    } else if (currentCommand == CMD_POSITION_MAPPED) {
        const int32_t position = fresh && millis() - zHallPositionMs < 250UL ? zHallPositionSteps : INT32_MIN;
        for (int shift = 24; shift >= 0; shift -= 8) replyBuffer[replyLength++] = ((uint32_t)position >> shift) & 0xFF;
    } else {
        replyBuffer[replyLength++] = 0xFF;
    }
    const byte zChecksum = livoCommunication.calculateCheckSum8bit(replyBuffer, replyLength);
    replyBuffer[replyLength++] = zChecksum;
    _wire.write(replyBuffer, replyLength);
    replyPending_ = false;
    return;
    #endif

    if (currentCommand == CMD_MAGAZINE_TRIGGER_ON) {
        #ifdef Magazine_PCB_V2
            magazineManualTriggerMode_ = true;
            setMagazineMhTrigger(true);
        #endif
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_MAGAZINE_TRIGGER_OFF) {
        #ifdef Magazine_PCB_V2
            magazineManualTriggerMode_ = false;
            setMagazineMhTrigger(false);
        #endif
        replyBuffer[replyLength++] = 0;
    }
    else if (currentCommand == CMD_HALL_DATA) {
        #ifndef Magazine_PCB_V2
            Serial.print("Sending Hall sensor values: ");
        #endif
        for (uint8_t i = 0; i < numSensors && replyLength + 2 < maxBufferSize; i++) {
            #ifdef Magazine_PCB_V2
                const int sensorValue = getCachedReplySensorValue(i, false);
            #else
                const int sensorValue = analogRead(sensorPins[i]);
                Serial.print(sensorValue);
                Serial.print(" ");
            #endif
            replyBuffer[replyLength++] = (sensorValue >> 8) & 0xFF;
            replyBuffer[replyLength++] = sensorValue & 0xFF;
        }
        #ifndef Magazine_PCB_V2
            Serial.println("");
        #endif

    } else  if (currentCommand == CMD_1_HALL_DATA) {
        #ifndef Magazine_PCB_V2
            Serial.print("Sending First Half Hall sensor values: ");
        #endif
        for (uint8_t i = 0; i < numSensors/2 && replyLength + 2 < maxBufferSize; i++) {
            #ifdef Magazine_PCB_V2
                const int sensorValue = getCachedReplySensorValue(i, false);
            #else
                const int sensorValue = analogRead(sensorPins[i]);
                Serial.print(sensorValue);
                Serial.print(" ");
            #endif
            replyBuffer[replyLength++] = (sensorValue >> 8) & 0xFF;
            replyBuffer[replyLength++] = sensorValue & 0xFF;
        }
        #ifndef Magazine_PCB_V2
            Serial.println("");
        #endif
    } else  if (currentCommand == CMD_2_HALL_DATA) {
        #ifndef Magazine_PCB_V2
            Serial.print("Sending Second Half Hall sensor values: ");
        #endif
        for (uint8_t i = numSensors/2; i < numSensors && replyLength + 2 < maxBufferSize; i++) {
            #ifdef Magazine_PCB_V2
                const int sensorValue = getCachedReplySensorValue(i, false);
            #else
                const int sensorValue = analogRead(sensorPins[i]);
                Serial.print(sensorValue);
                Serial.print(" ");
            #endif
            replyBuffer[replyLength++] = (sensorValue >> 8) & 0xFF;
            replyBuffer[replyLength++] = sensorValue & 0xFF;
        }
        #ifndef Magazine_PCB_V2
            Serial.println("");
        #endif
    } 
    else if (currentCommand == CMD_FILTERED_HALL_DATA) {
        #ifndef Magazine_PCB_V2
            Serial.print("Sending filtered Hall sensor values: ");
            int* smoothedValues = getSmoothedSensorValues();
        #endif
        for (uint8_t i = 0; i < numSensors && replyLength + 2 < maxBufferSize; i++) {
            #ifdef Magazine_PCB_V2
                const int sensorValue = getCachedReplySensorValue(i, true);
            #else
                const int sensorValue = smoothedValues[i];
                Serial.print(sensorValue);
                Serial.print(" ");
            #endif
            replyBuffer[replyLength++] = (sensorValue >> 8) & 0xFF;
            replyBuffer[replyLength++] = sensorValue & 0xFF;
        }
        #ifndef Magazine_PCB_V2
            Serial.println();
        #endif

    } 
    else if (currentCommand == CMD_1_FILTERED_HALL_DATA) {
        #ifndef Magazine_PCB_V2
            Serial.print("Sending First half filtered Hall sensor values: ");
            int* smoothedValues = getSmoothedSensorValues();
        #endif
        for (uint8_t i = 0; i < numSensors/2 && replyLength + 2 < maxBufferSize; i++) {
            #ifdef Magazine_PCB_V2
                const int sensorValue = getCachedReplySensorValue(i, true);
            #else
                const int sensorValue = smoothedValues[i];
                Serial.print(sensorValue);
                Serial.print(" ");
            #endif
            replyBuffer[replyLength++] = (sensorValue >> 8) & 0xFF;
            replyBuffer[replyLength++] = sensorValue & 0xFF;
        }
        #ifndef Magazine_PCB_V2
            Serial.println();
        #endif

    } 
    else if (currentCommand == CMD_2_FILTERED_HALL_DATA) {
        #ifndef Magazine_PCB_V2
            Serial.print("Sending Second half filtered Hall sensor values: ");
            int* smoothedValues = getSmoothedSensorValues();
        #endif
        for (uint8_t i = numSensors/2; i < numSensors && replyLength + 2 < maxBufferSize; i++) {
            #ifdef Magazine_PCB_V2
                const int sensorValue = getCachedReplySensorValue(i, true);
            #else
                const int sensorValue = smoothedValues[i];
                Serial.print(sensorValue);
                Serial.print(" ");
            #endif
            replyBuffer[replyLength++] = (sensorValue >> 8) & 0xFF;
            replyBuffer[replyLength++] = sensorValue & 0xFF;
        }
        #ifndef Magazine_PCB_V2
            Serial.println();
        #endif

    } 
    else if (currentCommand == CMD_POSITION_MAPPED) {
        #ifdef Gantry_X_Hall_PCB
        // GantryXPositionModel mapping
            readAndSendMappedPosition(gantryxstagepositionModel);
        #endif
        #ifdef Magazine_PCB_V2
            replyBuffer[replyLength++] = 0x00;
            replyBuffer[replyLength++] = 0x00;
            replyBuffer[replyLength++] = 0x00;
            replyBuffer[replyLength++] = 0x00;
        #endif
    } 
    else if (currentCommand == CMD_SLIDE_STATUS) {
        #ifdef Magazine_PCB_V2
            const int s21 = getCachedReplySensorValue(20, false);
            const int s22 = getCachedReplySensorValue(21, false);
            const int s23 = getCachedReplySensorValue(22, false);
            const int magCode = ((s23 < MAG_TYPE_S23_THRESHOLD) ? 4 : 0)
                              | ((s22 < MAG_TYPE_S22_THRESHOLD) ? 2 : 0)
                              | ((s21 < MAG_TYPE_S21_THRESHOLD) ? 1 : 0);
            const int magType = magCode;   // 0..7, 0 = no magazine

            if (magType == 0) {
                for (uint8_t i = 0; i < numSensors-4; i++) {
                    replyBuffer[replyLength++] = 0;
                }
            } else {
                // Debug-only slot status. Real slot detection is done on gantry side
                // using ESP32-stored per-type/per-holder thresholds (SETCAL). A rough
                // single threshold is plenty for diagnostic visibility here.
                const int DEBUG_SLOT_THRESHOLD = 3000;
                for (uint8_t i = 0; i < numSensors-4; i++) {
                    replyBuffer[replyLength++] =
                        (getCachedReplySensorValue(i, true) < DEBUG_SLOT_THRESHOLD) ? 1 : 0;
                }
            }
        #endif
    }
    else if (currentCommand == CMD_MAGZINE_STATUS) {
        #ifdef Magazine_PCB_V2
            const int tom1 = getCachedReplySensorValue(20, false);
            const int tom2 = getCachedReplySensorValue(21, false);
            const int tom3 = getCachedReplySensorValue(22, false);
        if (tom1 < 3300 || tom2 < 2800 || tom3 < 2000) {
            replyBuffer[replyLength++] = 1;
        } else {
            replyBuffer[replyLength++] = 0;
        }
        #else
            // Hall boards have only 10 channels; magazine channels 20..22
            // do not exist. Report unsupported instead of reading past the array.
            replyBuffer[replyLength++] = 0xFF;
        #endif
    }
    else if (currentCommand == CMD_MAGAZINE_EVENT) {
        #ifdef Magazine_PCB_V2
            replyBuffer[replyLength++] = pendingMagazineEvent_;
            replyBuffer[replyLength++] = stableMagazineType_;
            replyBuffer[replyLength++] = pendingPreviousMagazineType_;
            pendingMagazineEvent_ = MAG_EVENT_NONE;
            pendingPreviousMagazineType_ = stableMagazineType_;
            setMagazineAlert(false);
        #else
            replyBuffer[replyLength++] = MAG_EVENT_NONE;
            replyBuffer[replyLength++] = 0;
            replyBuffer[replyLength++] = 0;
        #endif
    }
    else if (currentCommand == CMD_LOCK_MAGZINE) {
        lockMagFlag = true;
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_UNLOCK_MAGZINE) {
        unlockMagFlag = true;
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_SOFT_RESET) {
        #ifdef Magazine_PCB_V2
            isFilterInitialized_ = false;
            for (int i = 0; i < numSensors; ++i) filteredSensorValues_[i] = 0.0f;
            lockMagFlag                          = false;
            unlockMagFlag                        = false;
            magazineMhCommandActive_             = false;
            magazineMhCommandNeedsLoopCompletion_ = false;
            magazineManualTriggerMode_           = false;
            calibrationRequested_                = false;
            calibrationStatus_                   = CAL_STATUS_IDLE;
            newRequest                           = false;
            setMagazineMhTrigger(false);
            digitalWrite(solenoid1, LOW);
            digitalWrite(solenoid2, LOW);
        #endif
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_CALIBRATE) {
        replyBuffer[replyLength++] = calibrationStatus_;
    }
    else if (currentCommand == CMD_CALIBRATE_RESULT_1) {
        #ifdef Magazine_PCB_V2
            replyBuffer[replyLength++] = calibrationStatus_;
            if (calibrationStatus_ == CAL_STATUS_COMPLETE) {
                replyBuffer[replyLength++] = calibrationMagType_;
                for (uint8_t s = 0; s < 10; s++) {
                    replyBuffer[replyLength++] = (calibrationResults_[s] >> 8) & 0xFF;
                    replyBuffer[replyLength++] =  calibrationResults_[s]       & 0xFF;
                }
            }
        #else
            replyBuffer[replyLength++] = CAL_STATUS_ERROR;
        #endif
    }
    else if (currentCommand == CMD_CALIBRATE_RESULT_2) {
        #ifdef Magazine_PCB_V2
            if (calibrationStatus_ == CAL_STATUS_COMPLETE) {
                for (uint8_t s = 10; s < 20; s++) {
                    replyBuffer[replyLength++] = (calibrationResults_[s] >> 8) & 0xFF;
                    replyBuffer[replyLength++] =  calibrationResults_[s]       & 0xFF;
                }
            } else {
                replyBuffer[replyLength++] = 0;
            }
        #else
            replyBuffer[replyLength++] = 0;
        #endif
    }
    else if (currentCommand == CMD_MAGFW_STATUS || currentCommand == CMD_MAGFW_VERIFY) {
        #ifdef Magazine_PCB_V2
            replyBuffer[replyLength++] = magFwStatus_;
            replyBuffer[replyLength++] = magFwError_;
            replyBuffer[replyLength++] = (magFwReceived_ >> 24) & 0xFF;
            replyBuffer[replyLength++] = (magFwReceived_ >> 16) & 0xFF;
            replyBuffer[replyLength++] = (magFwReceived_ >> 8)  & 0xFF;
            replyBuffer[replyLength++] =  magFwReceived_        & 0xFF;
            replyBuffer[replyLength++] = (magFwExpectedSize_ >> 24) & 0xFF;
            replyBuffer[replyLength++] = (magFwExpectedSize_ >> 16) & 0xFF;
            replyBuffer[replyLength++] = (magFwExpectedSize_ >> 8)  & 0xFF;
            replyBuffer[replyLength++] =  magFwExpectedSize_        & 0xFF;
        #else
            replyBuffer[replyLength++] = MAGFW_STATUS_UNSUPPORTED;
            replyBuffer[replyLength++] = MAGFW_ERR_UNSUPPORTED;
        #endif
    }
    if (replyLength >= maxBufferSize) replyLength = maxBufferSize - 1;
    byte checksum = livoCommunication.calculateCheckSum8bit(replyBuffer, replyLength);
    replyBuffer[replyLength++] = checksum;
    _wire.write(replyBuffer, replyLength);
    replyPending_ = false;

    finishMagazineMhCommand();
}

int* I2CInstanceSlave::getSmoothedSensorValues() {
    if (!sensorReader_ || !filteredSensorValues_) return smoothedSensorValues_;

    sensorReader_->readAllSensors();
    int* rawValues = sensorReader_->getAllSensorValues();
    for (int i = 0; i < numSensors; ++i) {
        cachedSensorValues_[i] = rawValues[i];
    }
    cachedSensorValuesReady_ = true;

    if (!isFilterInitialized_) {
        for (int i = 0; i < numSensors; ++i) {
            filteredSensorValues_[i] = static_cast<float>(rawValues[i]);
        }
        isFilterInitialized_ = true;
    } else {
        for (int i = 0; i < numSensors; ++i) {
            if(rawValues[i]<0) filteredSensorValues_[i]=-1;
            else if(filteredSensorValues_[i]<0) filteredSensorValues_[i]=rawValues[i];
            else filteredSensorValues_[i] = (alpha_ * static_cast<float>(rawValues[i])) +
                                      ((1.0f - alpha_) * filteredSensorValues_[i]);
        }
    }
    
    // Cache a stable local snapshot for the holder LED renderer.
    for (int i = 0; i < numSensors; ++i) {
        if(rawValues[i]<0) {smoothedSensorValues_[i]=-1;filteredSensorValues_[i]=-1;}
        else {if(filteredSensorValues_[i]<0) filteredSensorValues_[i]=rawValues[i];
            smoothedSensorValues_[i] = static_cast<int>(round(filteredSensorValues_[i]));}
    }
    return smoothedSensorValues_;
}

const int* I2CInstanceSlave::getCachedSensorValues() const {
    return cachedSensorValues_;
}

bool I2CInstanceSlave::hasCachedSensorValues() const {
    return cachedSensorValuesReady_;
}

void I2CInstanceSlave::handleReceive(int howMany) {
    receiveLength = 0;
    if (_wire.available() && howMany >= 1) {
        currentCommand = _wire.read();  // First byte = command
        receiveBuffer[0] = currentCommand;
        receiveLength = 1;
        replyPending_ = true;
        commandReceivedMs_ = millis();
        #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
            // Any transaction while in the bootloader counts as OTA activity —
            // it is the only thing this build talks to a master about.
            magFwLastActivityMs_ = commandReceivedMs_;
        #endif

        if (currentCommand == CMD_MAGAZINE_TRIGGER_ON) {
            magazineManualTriggerMode_ = true;
            setMagazineMhTrigger(true);
            magazineMhCommandActive_ = false;
            magazineMhCommandNeedsLoopCompletion_ = false;
        } else if (currentCommand == CMD_MAGAZINE_TRIGGER_OFF) {
            magazineManualTriggerMode_ = false;
            setMagazineMhTrigger(false);
            magazineMhCommandActive_ = false;
            magazineMhCommandNeedsLoopCompletion_ = false;
        } else if (currentCommand == CMD_CALIBRATE) {
            if (calibrationStatus_ == CAL_STATUS_IDLE) {
                calibrationRequested_ = true;
            }
            // RUNNING: ignore — don't restart mid-calibration
            // COMPLETE/ERROR: ignore — status preserved until CMD_SOFT_RESET clears it
        } else if (currentCommand == CMD_MAGFW_REBOOT) {
            #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
                // Already in bootloader: treat REBOOT as a harmless keep-alive.
                // Gantry sends this before BEGIN to kick an app into bootloader;
                // in bootloader mode we must remain READY, not fall back to IDLE.
                magFwStatus_ = MAGFW_STATUS_READY;
                magFwError_ = MAGFW_ERR_NONE;
            #else
            magFwRebootRequested_ = true;
            magFwStatus_ = MAGFW_STATUS_READY;
            magFwError_ = MAGFW_ERR_NONE;
            #endif
        } else if (currentCommand == CMD_MAGFW_BEGIN ||
                   currentCommand == CMD_MAGFW_DATA ||
                   currentCommand == CMD_MAGFW_COMMIT ||
                   currentCommand == CMD_MAGFW_ERASE) {
            #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
                // Payload bytes are copied below, then processed after the receive buffer is complete.
            #else
                magFwStatus_ = MAGFW_STATUS_UNSUPPORTED;
                magFwError_ = MAGFW_ERR_UNSUPPORTED;
                magFwExpectedSize_ = 0;
                magFwReceived_ = 0;
            #endif
        } else if (currentCommand == CMD_MAGFW_VERIFY) {
            #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
                // Process after receive buffer copy.
            #else
                magFwStatus_ = MAGFW_STATUS_UNSUPPORTED;
                magFwError_ = MAGFW_ERR_UNSUPPORTED;
            #endif
        } else if (currentCommand == CMD_MAGFW_ABORT) {
            magFwStatus_ = MAGFW_STATUS_IDLE;
            magFwError_ = MAGFW_ERR_NONE;
            magFwExpectedSize_ = 0;
            magFwReceived_ = 0;
            #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
                magFwExpectedCrc_ = 0;
                magFwExpectedSeq_ = 0;
                magFwUpdateActive_ = false;
                magFwCommitPending_ = false;
            #endif
        #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
        } else if (currentCommand == CMD_MAGFW_STATUS) {
            // MAGFW status is handled entirely by handleRequest(); do not enable sensors.
        } else if (isMagazineMhCommand(currentCommand) && !magazineManualTriggerMode_ &&
                   currentCommand < CMD_MAGFW_BEGIN) {
        #else
        } else if (isMagazineMhCommand(currentCommand) && !magazineManualTriggerMode_) {
        #endif
            setMagazineMhTrigger(true);
            magazineMhCommandActive_ = true;
            magazineMhCommandNeedsLoopCompletion_ =
                (currentCommand == CMD_LOCK_MAGZINE || currentCommand == CMD_UNLOCK_MAGZINE);
        }
    }

    while (_wire.available() && receiveLength < maxBufferSize) {
        receiveBuffer[receiveLength++] = _wire.read();  // if future expansion needs more data
    }

    newRequest = true;

    #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
        // BEGIN is intentionally non-destructive, so it is safe to process here.
        // Doing it immediately prevents a following STATUS request from overwriting
        // currentCommand before the main loop sees the BEGIN packet.
        if (currentCommand == CMD_MAGFW_BEGIN) {
            magFwHandleBegin();
            newRequest = false;
        } else if (currentCommand == CMD_MAGFW_COMMIT) {
            // Latch the COMMIT request (and its magic bytes) right here in the ISR,
            // while receiveBuffer still definitely holds this transaction. loop()
            // may not run again before the master starts a STATUS poll, which would
            // otherwise overwrite currentCommand/receiveBuffer out from under a
            // COMMIT that hadn't been processed yet.
            magFwCommitArgsOk_ = (receiveLength >= 5);
            magFwCommitMagic_ = magFwCommitArgsOk_ ? magFwReadU32(&receiveBuffer[1]) : 0;
            magFwCommitPending_ = true;
            newRequest = false;
        }
    #endif

}

bool I2CInstanceSlave::hasNewRequest() const {
    return newRequest;
}

const uint8_t* I2CInstanceSlave::getReceivedData() const {
    return receiveBuffer;
}

uint8_t I2CInstanceSlave::getReceivedLength() const {
    return receiveLength;
}

void I2CInstanceSlave::setReceiveCallback(Callback cb) {
    onReceiveCallback = cb;
}

void I2CInstanceSlave::loop() {
    #ifdef Gantry_Z_Hall_PCB
    serviceBusWatchdog();
    if (!cachedSensorValuesReady_ || millis() - localSensorSampleMs_ >= 5UL) {
        int raw[10], filtered[10];
        for (uint8_t i = 0; i < 10; ++i) {
            raw[i] = analogRead(sensorPins[i]);
            if (raw[i] < 0 || raw[i] > 4095) raw[i] = -1;
        }
        const uint32_t sampleMs=millis();
        const bool signalReady = zHallSignalFilter.update(raw, sampleMs);
        zHallSignalReady = signalReady;
        ++zHallSampleSequence;
        const float* signal = zHallSignalFilter.output();
        for (uint8_t i = 0; i < 10; ++i) {
            filteredSensorValues_[i] = signalReady ? signal[i] : -1;
            filtered[i] = signalReady ? (int)lroundf(signal[i]) : -1;
        }
        ZHallPosition::Result positionResult;
        bool publishPosition = false;
        uint32_t positionSpan=0;
        if (!signalReady) {
            zHallPositionWindow.reset();
            positionResult.status = ZHallPosition::InvalidAdc;
            publishPosition = true;
        } else {
            // Keep the position window in float; rounding is only for legacy
            // filtered ADC packets. Raw packets above remain diagnostic truth.
            float mean[10];
            if (zHallPositionWindow.update(signal,sampleMs,mean,positionSpan)) {
                positionResult = ZHallPosition::predict(mean, ZHallCalibrationData::table);
                publishPosition = true;
            }
        }
        const uint32_t interruptMask = __get_PRIMASK();
        __disable_irq();
        for (uint8_t i = 0; i < 10; ++i) {
            cachedSensorValues_[i] = raw[i];
            smoothedSensorValues_[i] = filtered[i];
        }
        localSensorSampleMs_ = millis();
        cachedSensorValuesReady_ = true;
        isFilterInitialized_ = true;
        if (publishPosition) {
            zHallPositionStatus = positionResult.status;
            zHallPositionSteps = positionResult.status == ZHallPosition::Valid ? (int32_t)lroundf(positionResult.steps) : INT32_MIN;
            zHallUncertaintySteps = positionResult.status == ZHallPosition::Valid ? (uint16_t)fminf(UINT16_MAX, ceilf(3 * positionResult.sigmaSteps)) : UINT16_MAX;
            ++zHallPositionSequence;
            zHallPositionMs = sampleMs;
            zHallPositionWindowSpanMs=positionSpan>UINT16_MAX ? UINT16_MAX : positionSpan;
            zHallPositionWindowSamples=signalReady ? 32 : 0;
        }
        __set_PRIMASK(interruptMask);
    }
    // USB diagnostics never wait for a slow/disconnected host. The complete
    // line fits the CDC queue; only enqueue when all bytes fit, else expire.
    if(zHallDiagnosticLength) {
        if(millis()-zHallDiagnosticQueuedMs>=200UL)zHallDiagnosticLength=0;
        else if(Serial.availableForWrite()>=(int)zHallDiagnosticLength) {
            const size_t sent=Serial.write((const uint8_t*)zHallDiagnosticBuffer,zHallDiagnosticLength);
            if(sent)zHallDiagnosticLength=0;
        }
    }
    #endif
    #ifdef Magazine_PCB_V2
        IWatchdog.reload();
        serviceBusWatchdog();
    #endif

    if (magFwRebootRequested_) {
        delay(20);
        NVIC_SystemReset();
    }

    #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
        if (magFwCommitPending_) {
            magFwCommitPending_ = false;
            magFwHandleCommit(magFwCommitMagic_, magFwCommitArgsOk_);
            return;
        }
    #endif

    if (newRequest) {
        newRequest = false;

        #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
            if (currentCommand == CMD_MAGFW_BEGIN) {
                magFwHandleBegin();
                return;
            } else if (currentCommand == CMD_MAGFW_DATA) {
                magFwHandleData();
                return;
            } else if (currentCommand == CMD_MAGFW_ERASE) {
                magFwHandleErase();
                return;
            } else if (currentCommand == CMD_MAGFW_STATUS) {
                return;
            } else if (currentCommand == CMD_MAGFW_VERIFY) {
                magFwHandleVerify();
                return;
            } else if (currentCommand == CMD_MAGFW_ABORT || currentCommand == CMD_MAGFW_REBOOT) {
                return;
            }
            // CMD_MAGFW_COMMIT is handled above via magFwCommitPending_, latched
            // directly from the ISR — it never reaches this dispatch.
        #endif

        // Optional: Print or handle internally
        if (onReceiveCallback) {
            onReceiveCallback(receiveBuffer, receiveLength);
        } else {
            // Default behavior: echo back
            setReplyData(receiveBuffer, receiveLength);
        }
    }

    #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
        // Fall back to the app once the bus has been quiet for the boot window,
        // as long as no erase/data-receive session is actually in flight. This is
        // keyed off last activity, not boot time, and off magFwUpdateActive_ rather
        // than magFwExpectedSize_ (which BEGIN sets once and is never cleared for
        // the rest of this power cycle) — otherwise a single OTA attempt, whether
        // it fails or fully succeeds through a lost COMMIT, permanently disables
        // this fallback and strands the holder in the bootloader. jumpToApp() is a
        // no-op if the app region does not look valid, so this is safe to retry.
        if (!MAGFW_STAY_IN_BOOTLOADER_FOR_TEST && !magFwUpdateActive_ &&
            millis() - magFwLastActivityMs_ >= MAGFW_BOOT_WINDOW_MS) {
            magFwJumpToApp();
        }
        return;
    #endif

    #ifdef Magazine_PCB_V2
    if (calibrationRequested_) {
        calibrationRequested_ = false;
        calibrationStatus_    = CAL_STATUS_RUNNING;
        isFilterInitialized_  = false;   // EMA will re-seed from raw after calibration

        Serial.println("CAL: starting on-board calibration");

        // Detect magazine type from S21/S22/S23 with trigger ON
        digitalWrite(IR_1_23_5V_EN, HIGH);
        delay(5);
        const int s21 = analogRead(sensorPins[20]);
        const int s22 = analogRead(sensorPins[21]);
        const int s23 = analogRead(sensorPins[22]);
        digitalWrite(IR_1_23_5V_EN, LOW);

        const int magCode = ((s23 < MAG_TYPE_S23_THRESHOLD) ? 4 : 0)
                          | ((s22 < MAG_TYPE_S22_THRESHOLD) ? 2 : 0)
                          | ((s21 < MAG_TYPE_S21_THRESHOLD) ? 1 : 0);
        // Accept any 3-bit code 1..7 (S23/S22/S21). 0 = no magazine.
        //   1=MAG001, 2=MAG010, 3=MAG011, 4=MAG100, 5=MAG101, 6=MAG110, 7=MAG111
        const uint8_t magType = (uint8_t)magCode;

        if (magType == 0) {
            Serial.println("CAL Error: no magazine detected");
            calibrationStatus_ = CAL_STATUS_ERROR;
        } else {
            Serial.print("CAL: Magazine 00"); Serial.print(magType);
            Serial.print(", Holder ");
            Serial.println((DEVICE_ID == Stainer_Magzine1_PCB_ID) ? 1 : 2);

            const int NUM_RUNS    = 2;
            const int NUM_BATCHES = 5;
            const int BATCH_SIZE  = 100;
            const unsigned long SAMPLE_INTERVAL_MS = 10;

            long finalSum[20] = {0};

            for (int run = 0; run < NUM_RUNS; run++) {
                Serial.print("CAL: run "); Serial.println(run + 1);
                long runSum[20] = {0};

                for (int batch = 0; batch < NUM_BATCHES; batch++) {
                    long batchSum[20] = {0};

                    digitalWrite(IR_1_23_5V_EN, HIGH);
                    delay(2);

                    for (int sample = 0; sample < BATCH_SIZE; sample++) {
                        const unsigned long t = millis();
                        for (int s = 0; s < 20; s++) {
                            batchSum[s] += analogRead(sensorPins[s]);
                        }
                        while (millis() - t < SAMPLE_INTERVAL_MS) {}
                        IWatchdog.reload();
                    }

                    digitalWrite(IR_1_23_5V_EN, LOW);
                    delay(2);

                    for (int s = 0; s < 20; s++) {
                        runSum[s] += batchSum[s] / BATCH_SIZE;
                    }
                }
                for (int s = 0; s < 20; s++) {
                    finalSum[s] += runSum[s] / NUM_BATCHES;
                }
            }

            for (int s = 0; s < 20; s++) {
                const long avg = finalSum[s] / NUM_RUNS;
                calibrationResults_[s] = (uint16_t)(avg > 65535 ? 65535 : avg < 0 ? 0 : avg);
            }

            calibrationMagType_ = magType;
            calibrationStatus_  = CAL_STATUS_COMPLETE;  // flag-last: ISR reads this safely
            Serial.println("CAL: complete");
        }
    }
    #endif

    if (calibrationStatus_ != CAL_STATUS_RUNNING) {
        #ifdef Magazine_PCB_V2
            // Local holder sensing: briefly power the sensor rail, refresh the
            // 24-channel EMA, then turn it back off. Explicit Gantry trigger mode
            // owns the rail and is never overridden here.
            const unsigned long now = millis();
            if (!magazineMhCommandActive_ && now - localSensorSampleMs_ >= 100) {
                const bool localTrigger = !magazineManualTriggerMode_;
                if (localTrigger) setMagazineMhTrigger(true);
                delay(5);
                (void)getSmoothedSensorValues();
                updateLocalMagazineDetection();
                localSensorSampleMs_ = now;
                if (localTrigger && !magazineManualTriggerMode_ && !magazineMhCommandActive_) {
                    setMagazineMhTrigger(false);
                }
            }
        #else
            (void)getSmoothedSensorValues();
        #endif
    }

    #ifdef Magazine_PCB_V2
    if (lockMagFlag) {
        for (int i = 0; i<3; i++) {
            digitalWrite(solenoid1, HIGH);
            delay(5);
            digitalWrite(solenoid1, LOW);
            delay(10);
        }
        lockMagFlag = false;
        if (magazineMhCommandNeedsLoopCompletion_) {
            setMagazineMhTrigger(false);
            magazineMhCommandNeedsLoopCompletion_ = false;
            magazineMhCommandActive_ = false;
        }
        Serial.println("Locked");
    }
    if (unlockMagFlag) {
        for (int i = 0; i<3; i++) {
            digitalWrite(solenoid2, HIGH);
            delay(5);
            digitalWrite(solenoid2, LOW);
            delay(10);
        }
        unlockMagFlag = false;
        if (magazineMhCommandNeedsLoopCompletion_) {
            setMagazineMhTrigger(false);
            magazineMhCommandNeedsLoopCompletion_ = false;
            magazineMhCommandActive_ = false;
        }
        Serial.println("Unlocked");
    }
    #endif
}
#endif

#endif // Magazine_PCB_V2 stable implementation selection
