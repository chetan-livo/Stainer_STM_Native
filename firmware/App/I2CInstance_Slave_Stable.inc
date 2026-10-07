#include "SensorFaultPolicy.h"
/*
* I2CInstance_Slave.cpp
*
*  Created on: 22-Apr-2025
*      Author: Varalakshmi
*/

#include "I2CInstance_Slave.h"
#include "MagazineLED.h"

#if defined(MAGAZINE_BOOTLOADER_BUILD)
#include <IWatchdog.h>
#include "stm32f4xx_hal.h"
static constexpr uint32_t MAGFW_APP_BASE = MAGFW_APP_BASE_ADDRESS;
static constexpr uint32_t MAGFW_FLASH_END = 0x08060000UL;
static constexpr uint32_t MAGFW_COMMIT_MAGIC = 0x4D465743UL;
static constexpr uint32_t MAGFW_BOOT_WINDOW_MS = 10000UL;
static constexpr uint8_t MAGFW_VECTOR_GATE_SIZE = 8;

__attribute__((noreturn, noinline))
static void magazineBranchToApp(uint32_t appStack, uint32_t appResetAddress) {
    __disable_irq();
    SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    HAL_RCC_DeInit(); HAL_DeInit();
    SCB->VTOR = MAGFW_APP_BASE;
    __set_CONTROL(0); __set_BASEPRI(0); __set_FAULTMASK(0);
    __DSB(); __ISB();
    __asm volatile("msr msp, %0\n dsb\n isb\n cpsie i\n bx %1\n"
        : : "r" (appStack), "r" (appResetAddress) : "memory");
    __builtin_unreachable();
}
#endif

#ifdef Slave
I2CInstanceSlave* activeSlaveInstance = nullptr;
Callback I2CInstanceSlave::onReceiveCallback = nullptr;

#ifdef Gantry_X_Hall_PCB
    XYStagePositionModel gantryxstagepositionModel;
#endif
#ifdef Gantry_Z_Hall_PCB
    XYStagePositionModel gantryzstagepositionModel;
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
:   _wire(&wire),
    sensorReader_(nullptr),
    filteredSensorValues_(nullptr),
    // Fast local LED response. At the 20 ms sample period, 0.02 caused
    // second-scale lag; 0.50 follows a real slot transition within a few samples.
    alpha_(0.50f),
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

#if defined(MAGAZINE_BOOTLOADER_BUILD)
uint8_t I2CInstanceSlave::magFwCrc8(const uint8_t* data, uint8_t len) const {
    uint8_t crc = 0; for (uint8_t i = 0; i < len; ++i) crc ^= data[i]; return crc;
}
uint32_t I2CInstanceSlave::magFwReadU32(const uint8_t* p) const {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
uint16_t I2CInstanceSlave::magFwReadU16(const uint8_t* p) const {
    return ((uint16_t)p[0] << 8) | p[1];
}
uint32_t I2CInstanceSlave::magFwCrc32Update(uint32_t crc, const uint8_t* data, uint32_t len) const {
    crc = ~crc;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
    }
    return ~crc;
}
bool I2CInstanceSlave::magFwAppLooksValid() const {
    const uint32_t sp = *(uint32_t*)MAGFW_APP_BASE;
    const uint32_t rv = *(uint32_t*)(MAGFW_APP_BASE + 4);
    return sp >= 0x20000000UL && sp < 0x20040000UL && rv >= MAGFW_APP_BASE && rv < MAGFW_FLASH_END;
}
bool I2CInstanceSlave::magFwBufferedVectorLooksValid() const {
    if (!magFwVectorGateBuffered_) return false;
    const uint32_t sp = (uint32_t)magFwVectorGate_[0] | ((uint32_t)magFwVectorGate_[1] << 8) |
        ((uint32_t)magFwVectorGate_[2] << 16) | ((uint32_t)magFwVectorGate_[3] << 24);
    const uint32_t rv = (uint32_t)magFwVectorGate_[4] | ((uint32_t)magFwVectorGate_[5] << 8) |
        ((uint32_t)magFwVectorGate_[6] << 16) | ((uint32_t)magFwVectorGate_[7] << 24);
    return sp >= 0x20000000UL && sp < 0x20040000UL && rv >= MAGFW_APP_BASE && rv < MAGFW_FLASH_END;
}
uint32_t I2CInstanceSlave::magFwAppCrc(uint32_t len) const {
    return magFwCrc32Update(0, (const uint8_t*)MAGFW_APP_BASE, len);
}
uint32_t I2CInstanceSlave::magFwAppCrcWithBufferedVector() const {
    if (!magFwVectorGateBuffered_ || magFwExpectedSize_ < 8) return 0;
    uint32_t crc = magFwCrc32Update(0, magFwVectorGate_, 8);
    if (magFwExpectedSize_ > 8) crc = magFwCrc32Update(crc,
        (const uint8_t*)(MAGFW_APP_BASE + 8), magFwExpectedSize_ - 8);
    return crc;
}
bool I2CInstanceSlave::magFwEraseApp(uint32_t size) {
    if (!size || MAGFW_APP_BASE + size > MAGFW_FLASH_END) return false;
    const uint32_t lastSector = (MAGFW_APP_BASE + size - 1 < 0x08040000UL) ? FLASH_SECTOR_5 : FLASH_SECTOR_6;
    FLASH_EraseInitTypeDef erase = {};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS; erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.NbSectors = 1;
    HAL_FLASH_Unlock(); HAL_StatusTypeDef result = HAL_OK;
    for (uint32_t sector = FLASH_SECTOR_5; sector <= lastSector; ++sector) {
        uint32_t error = 0; erase.Sector = sector; IWatchdog.reload();
        result = HAL_FLASHEx_Erase(&erase, &error); IWatchdog.reload();
        if (result != HAL_OK) break;
    }
    HAL_FLASH_Lock(); return result == HAL_OK;
}
bool I2CInstanceSlave::magFwWriteFlash(uint32_t offset, const uint8_t* data, uint8_t len) {
    if ((offset & 3U) || (len & 3U) || MAGFW_APP_BASE + offset + len > MAGFW_FLASH_END) return false;
    HAL_FLASH_Unlock();
    for (uint8_t i = 0; i < len; i += 4) {
        const uint32_t word = (uint32_t)data[i] | ((uint32_t)data[i + 1] << 8) |
            ((uint32_t)data[i + 2] << 16) | ((uint32_t)data[i + 3] << 24);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGFW_APP_BASE + offset + i, word) != HAL_OK ||
            *(uint32_t*)(MAGFW_APP_BASE + offset + i) != word) { HAL_FLASH_Lock(); return false; }
    }
    HAL_FLASH_Lock(); return true;
}
bool I2CInstanceSlave::magFwWriteBufferedVector() {
    if (!magFwBufferedVectorLooksValid()) return false;
    const uint32_t sp = (uint32_t)magFwVectorGate_[0] | ((uint32_t)magFwVectorGate_[1] << 8) |
        ((uint32_t)magFwVectorGate_[2] << 16) | ((uint32_t)magFwVectorGate_[3] << 24);
    const uint32_t rv = (uint32_t)magFwVectorGate_[4] | ((uint32_t)magFwVectorGate_[5] << 8) |
        ((uint32_t)magFwVectorGate_[6] << 16) | ((uint32_t)magFwVectorGate_[7] << 24);
    HAL_FLASH_Unlock();
    const bool ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGFW_APP_BASE + 4, rv) == HAL_OK &&
        *(uint32_t*)(MAGFW_APP_BASE + 4) == rv &&
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGFW_APP_BASE, sp) == HAL_OK &&
        *(uint32_t*)MAGFW_APP_BASE == sp;
    HAL_FLASH_Lock(); return ok;
}
void I2CInstanceSlave::magFwJumpToApp() {
    if (!magFwAppLooksValid()) return;
    if (_serial) _serial->end();
    magazineBranchToApp(*(uint32_t*)MAGFW_APP_BASE, *(uint32_t*)(MAGFW_APP_BASE + 4));
}
void I2CInstanceSlave::magFwHandleBegin() {
    if (receiveLength < 13) { magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_ARGS; return; }
    magFwExpectedSize_ = magFwReadU32(receiveBuffer + 1);
    magFwExpectedCrc_ = magFwReadU32(receiveBuffer + 5);
    if (magFwReadU32(receiveBuffer + 9) != MAGFW_APP_BASE || magFwExpectedSize_ < 8 ||
        MAGFW_APP_BASE + magFwExpectedSize_ > MAGFW_FLASH_END || (magFwExpectedSize_ & 3U)) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_ARGS; return;
    }
    memset(magFwVectorGate_, 0xFF, sizeof(magFwVectorGate_));
    magFwVectorGateBuffered_ = false; magFwReceived_ = 0; magFwExpectedSeq_ = 0;
    magFwUpdateActive_ = false; magFwStatus_ = MAGFW_STATUS_READY; magFwError_ = MAGFW_ERR_NONE;
}
void I2CInstanceSlave::magFwHandleErase() {
    if (!magFwExpectedSize_ || magFwStatus_ != MAGFW_STATUS_READY) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_STATE; return;
    }
    magFwStatus_ = MAGFW_STATUS_ERASING; magFwError_ = MAGFW_ERR_NONE;
    magFwReceived_ = 0; magFwExpectedSeq_ = 0; magFwVectorGateBuffered_ = false;
    if (!magFwEraseApp(magFwExpectedSize_)) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_FLASH; return;
    }
    magFwUpdateActive_ = true; magFwStatus_ = MAGFW_STATUS_RECEIVING;
}
void I2CInstanceSlave::magFwHandleData() {
    if (!magFwUpdateActive_ || receiveLength < 10) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_STATE; return;
    }
    const uint8_t len = receiveBuffer[7];
    if (magFwCrc8(receiveBuffer, receiveLength - 1) != receiveBuffer[receiveLength - 1]) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_CRC; return;
    }
    if (magFwReadU16(receiveBuffer + 1) != magFwExpectedSeq_ || !len || len > 20 ||
        receiveLength != (uint8_t)(9 + len) || magFwReadU32(receiveBuffer + 3) != magFwReceived_ ||
        magFwReceived_ + len > magFwExpectedSize_) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_SEQ; return;
    }
    bool ok = false;
    if (magFwReceived_ == 0) {
        if (len < 8) { magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_ARGS; return; }
        memcpy(magFwVectorGate_, receiveBuffer + 8, 8); magFwVectorGateBuffered_ = true;
        ok = len == 8 || magFwWriteFlash(8, receiveBuffer + 16, len - 8);
    } else ok = magFwWriteFlash(magFwReceived_, receiveBuffer + 8, len);
    if (!ok) { magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_FLASH; return; }
    magFwReceived_ += len; ++magFwExpectedSeq_; magFwStatus_ = MAGFW_STATUS_RECEIVING;
}
void I2CInstanceSlave::magFwHandleVerify() {
    magFwUpdateActive_ = false; magFwStatus_ = MAGFW_STATUS_VERIFYING;
    if (magFwReceived_ != magFwExpectedSize_ || magFwAppCrcWithBufferedVector() != magFwExpectedCrc_ ||
        !magFwBufferedVectorLooksValid()) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_VERIFY; return;
    }
    if (!magFwWriteBufferedVector() || magFwAppCrc(magFwExpectedSize_) != magFwExpectedCrc_ ||
        !magFwAppLooksValid()) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_VERIFY; return;
    }
    magFwStatus_ = MAGFW_STATUS_COMPLETE; magFwError_ = MAGFW_ERR_NONE;
}
void I2CInstanceSlave::magFwHandleCommit() {
    if (magFwCommitMagic_ != MAGFW_COMMIT_MAGIC || magFwStatus_ != MAGFW_STATUS_COMPLETE) {
        magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_BAD_STATE; return;
    }
    delay(20); magFwJumpToApp();
    magFwStatus_ = MAGFW_STATUS_ERROR; magFwError_ = MAGFW_ERR_VERIFY;
}
#endif

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
            case CMD_CALIBRATE_RESULT_ALL:
            case CMD_MAGAZINE_EVENT:
            case CMD_FIRMWARE_VERSION:
            case CMD_FIRMWARE_IDENTITY:
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

I2CInstanceSlave::I2CInstanceSlave(LivoHardwareSerial& serial)
:   _wire(nullptr), _serial(&serial), uartMode_(true),
    sensorReader_(nullptr), filteredSensorValues_(nullptr), alpha_(0.50f),
    isFilterInitialized_(false)
{
    activeSlaveInstance = this;
}

void I2CInstanceSlave::setMagazineAlert(bool asserted) {
    #ifdef Magazine_PCB_V2
        digitalWrite(I2C1_INT, asserted ? LOW : HIGH);
    #else
        (void)asserted;
    #endif
}

void I2CInstanceSlave::updateLocalMagazineDetection(const int values[24]) {
    #ifdef Magazine_PCB_V2
        if (values == nullptr) return;
        const uint8_t detectedType =
            ((values[22] < MAG_TYPE_S23_THRESHOLD) ? 4U : 0U) |
            ((values[21] < MAG_TYPE_S22_THRESHOLD) ? 2U : 0U) |
            ((values[20] < MAG_TYPE_S21_THRESHOLD) ? 1U : 0U);

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
    #else
        (void)values;
    #endif
}

void I2CInstanceSlave::finishMagazineMhCommand() {
    if (!magazineMhCommandActive_ || magazineMhCommandNeedsLoopCompletion_) {
        return;
    }

    setMagazineMhTrigger(false);
    magazineMhCommandActive_ = false;
}

void I2CInstanceSlave::setup(int sda, int scl, uint8_t slaveAddress) {
    _wire->setSDA(sda);
    _wire->setSCL(scl);
    _wire->begin(slaveAddress, false, false);

    _wire->onRequest(onRequestHandler);
    _wire->onReceive(onReceiveHandler);

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
    #endif
}

void I2CInstanceSlave::setupUART(uint8_t slaveAddress, uint32_t baud) {
    (void)slaveAddress;
    _serial->begin(baud);
    #if defined(MAGAZINE_BOOTLOADER_BUILD)
    pinMode(IR_1_23_5V_EN, OUTPUT); digitalWrite(IR_1_23_5V_EN, LOW);
    pinMode(I2C1_INT, OUTPUT_OPEN_DRAIN); setMagazineAlert(false);
    pinMode(solenoid1, OUTPUT); pinMode(solenoid2, OUTPUT);
    digitalWrite(solenoid1, LOW); digitalWrite(solenoid2, LOW);
    magFwLastActivityMs_ = millis();
    return;
    #endif
    sensorReader_ = new AnalogSensorReader(sensorPins, numSensors);
    if (sensorReader_) sensorReader_->setup();
    filteredSensorValues_ = new float[numSensors]();
    isFilterInitialized_ = false;
    pinMode(IR_1_23_5V_EN, OUTPUT);
    pinMode(I2C1_INT, OUTPUT_OPEN_DRAIN);
    setMagazineAlert(false);
    pinMode(solenoid1, OUTPUT); pinMode(solenoid2, OUTPUT);
    digitalWrite(IR_1_23_5V_EN, LOW);
    digitalWrite(solenoid1, LOW); digitalWrite(solenoid2, LOW);
}

// Set data to reply when master requests
void I2CInstanceSlave::setReplyData(const uint8_t* data, uint8_t length) {
    replyLength = min(length, maxBufferSize);
    memcpy(replyBuffer, data, replyLength);
}

template<typename ModelType>
void I2CInstanceSlave::readAndSendMappedPosition(ModelType& model) {
    int raw[ModelType::NUM_FEATURES];
    int* smoothedValues = getSmoothedSensorValues();
    for (uint8_t i = 0; i < ModelType::NUM_FEATURES; ++i) {
        raw[i] = smoothedValues[i];
    }

    Serial.print("Raw IR sensor values: ");
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

    if (currentCommand == CMD_MAGAZINE_TRIGGER_ON) {
        #ifdef Magazine_PCB_V2
            magazineManualTriggerMode_ = true;
            setMagazineMhTrigger(true);
            Serial.println("Magazine trigger ON");
        #endif
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_MAGAZINE_TRIGGER_OFF) {
        #ifdef Magazine_PCB_V2
            magazineManualTriggerMode_ = false;
            setMagazineMhTrigger(false);
            Serial.println("Magazine trigger OFF");
        #endif
        replyBuffer[replyLength++] = 0;
    }
    else if (currentCommand == CMD_HALL_DATA) {
        Serial.print("Sending IR sensor values: ");
        for (uint8_t i = 0; i < numSensors; i++) {
            int sensorValue = analogRead(sensorPins[i]);  // 10-bit value

            Serial.print(sensorValue);
            Serial.print(" ");

            byte msb = (sensorValue >> 8) & 0xFF;
            byte lsb = sensorValue & 0xFF;

            replyBuffer[replyLength++] = msb;
            replyBuffer[replyLength++] = lsb;
        }
        Serial.println("");

    } else  if (currentCommand == CMD_1_HALL_DATA) {
        Serial.print("Sending first-half IR sensor values: ");
        for (uint8_t i = 0; i < numSensors/2; i++) {
            int sensorValue = analogRead(sensorPins[i]);  // 10-bit value

            Serial.print(sensorValue);
            Serial.print(" ");

            byte msb = (sensorValue >> 8) & 0xFF;
            byte lsb = sensorValue & 0xFF;

            replyBuffer[replyLength++] = msb;
            replyBuffer[replyLength++] = lsb;
        }
        Serial.println("");
    } else  if (currentCommand == CMD_2_HALL_DATA) {
        Serial.print("Sending second-half IR sensor values: ");
        for (uint8_t i = numSensors/2; i < numSensors; i++) {
            int sensorValue = analogRead(sensorPins[i]);  // 10-bit value

            Serial.print(sensorValue);
            Serial.print(" ");

            byte msb = (sensorValue >> 8) & 0xFF;
            byte lsb = sensorValue & 0xFF;

            replyBuffer[replyLength++] = msb;
            replyBuffer[replyLength++] = lsb;
        }
        Serial.println("");
    } 
    else if (currentCommand == CMD_FILTERED_HALL_DATA) {
        Serial.print("Sending filtered IR sensor values: ");

        int* smoothedValues = getSmoothedSensorValues();

        for (uint8_t i = 0; i < numSensors; i++) {
            Serial.print(smoothedValues[i]);
            Serial.print(" ");

            byte msb = (smoothedValues[i] >> 8) & 0xFF;
            byte lsb = smoothedValues[i] & 0xFF;

            replyBuffer[replyLength++] = msb;
            replyBuffer[replyLength++] = lsb;
        }
        Serial.println();

    } 
    else if (currentCommand == CMD_1_FILTERED_HALL_DATA) {
        Serial.print("Sending first-half filtered IR sensor values: ");

        int* smoothedValues = getSmoothedSensorValues();

        for (uint8_t i = 0; i < numSensors/2; i++) {
            Serial.print(smoothedValues[i]);
            Serial.print(" ");

            byte msb = (smoothedValues[i] >> 8) & 0xFF;
            byte lsb = smoothedValues[i] & 0xFF;

            replyBuffer[replyLength++] = msb;
            replyBuffer[replyLength++] = lsb;
        }
        Serial.println();

    } 
    else if (currentCommand == CMD_2_FILTERED_HALL_DATA) {
        Serial.print("Sending second-half filtered IR sensor values: ");

        int* smoothedValues = getSmoothedSensorValues();

        for (uint8_t i = numSensors/2; i < numSensors; i++) {
            Serial.print(smoothedValues[i]);
            Serial.print(" ");

            byte msb = (smoothedValues[i] >> 8) & 0xFF;
            byte lsb = smoothedValues[i] & 0xFF;

            replyBuffer[replyLength++] = msb;
            replyBuffer[replyLength++] = lsb;
        }
        Serial.println();

    } 
    else if (currentCommand == CMD_POSITION_MAPPED) {
        #ifdef Gantry_X_Hall_PCB
        // GantryXPositionModel mapping
            readAndSendMappedPosition(gantryxstagepositionModel);
        #endif
        #ifdef Gantry_Z_Hall_PCB
        // GantryYPositionModel mapping
            readAndSendMappedPosition(gantryzstagepositionModel);
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
            // The holder PCB is authoritative for slide presence. Average ten
            // fresh readings so a single noisy live sample cannot flip a slot.
            static const uint8_t SAMPLE_COUNT = 10;
            uint32_t sums[23] = {}; bool invalid[23]={};
            for (uint8_t sample = 0; sample < SAMPLE_COUNT; ++sample) {
                for (uint8_t sensor = 0; sensor < 23; ++sensor) {
                    const int value=analogRead(sensorPins[sensor]);
                    invalid[sensor] |= !SensorFaultPolicy::adcValid(value);
                    if(SensorFaultPolicy::adcValid(value)) sums[sensor] += value;
                }
            }

            int averages[24] = {};
            for (uint8_t sensor = 0; sensor < 23; ++sensor) {
                averages[sensor] = invalid[sensor] ? -1 : (int)(sums[sensor] / SAMPLE_COUNT);
            }
            const bool typeValid=!invalid[20]&&!invalid[21]&&!invalid[22];
            if(typeValid) updateLocalMagazineDetection(averages);

            const uint8_t magType =
                ((averages[22] < MAG_TYPE_S23_THRESHOLD) ? 4U : 0U) |
                ((averages[21] < MAG_TYPE_S22_THRESHOLD) ? 2U : 0U) |
                ((averages[20] < MAG_TYPE_S21_THRESHOLD) ? 1U : 0U);
            uint8_t slots[20] = {};
            const bool statusValid = magType != 0 &&
                magazineLED.classifySlots(magType, averages, slots);

            // Byte 0: 0=no magazine, 1=valid occupancy, 2=calibration unavailable.
            // Byte 1: magazine type. Bytes 2..21: holder-classified slots.
            // Byte 22: valid calibration types; bit 0=type 1 ... bit 6=type 7.
            bool sensorError=false;for(uint8_t i=0;i<23;++i)sensorError|=invalid[i];
            // State 3 means invalid measurements; never declare empty or occupied.
            replyBuffer[replyLength++] = sensorError ? 3 : (magType == 0) ? 0 : (statusValid ? 1 : 2);
            replyBuffer[replyLength++] = magType;
            for (uint8_t slot = 0; slot < 20; ++slot) {
                replyBuffer[replyLength++] = invalid[slot] ? 3 : statusValid ? slots[slot] : 0;
            }
            replyBuffer[replyLength++] = magazineLED.getThresholdValidMask();
        #endif
    }
    else if (currentCommand == CMD_MAGZINE_STATUS) {
        // Keep the legacy one-byte response consistent with CMD_SLIDE_STATUS.
        // Ten samples prevent a noisy type input from reporting a false removal.
        static const uint8_t SAMPLE_COUNT = 10;
        uint32_t sums[3] = {}; bool valid=true;
        for (uint8_t sample = 0; sample < SAMPLE_COUNT; ++sample) {
            for(int channel=0;channel<3;++channel) {
                const int value=analogRead(sensorPins[20+channel]);
                valid &= SensorFaultPolicy::adcValid(value);
                if(SensorFaultPolicy::adcValid(value)) sums[channel]+=value;
            }
        }
        const int s21 = (int)(sums[0] / SAMPLE_COUNT);
        const int s22 = (int)(sums[1] / SAMPLE_COUNT);
        const int s23 = (int)(sums[2] / SAMPLE_COUNT);
        const uint8_t magType =
            ((s23 < MAG_TYPE_S23_THRESHOLD) ? 4U : 0U) |
            ((s22 < MAG_TYPE_S22_THRESHOLD) ? 2U : 0U) |
            ((s21 < MAG_TYPE_S21_THRESHOLD) ? 1U : 0U);
        replyBuffer[replyLength++] = valid ? (magType != 0 ? 1 : 0) : 0xff;
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
        // Lock Magzine
        Serial.println("Locking Magzine");
        lockMagFlag = true;
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_UNLOCK_MAGZINE) {
        // Unlock Magzine
        Serial.println("Unlocking Magzine");
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
            Serial.println("SOFT RESET");
        #endif
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_CALIBRATE) {
        replyBuffer[replyLength++] = calibrationStatus_;
    }
    else if (currentCommand == CMD_UART_LINK_TEST) {
        replyBuffer[replyLength++] = DEVICE_ID;
        replyBuffer[replyLength++] = 0x55; // UART transport marker
        replyBuffer[replyLength++] = 0x01; // framed UART protocol version
    }
    else if (currentCommand == CMD_FIRMWARE_VERSION) {
        int firstDot = FWverNo.indexOf('.');
        int secondDot = FWverNo.indexOf('.', firstDot + 1);
        replyBuffer[replyLength++] = DEVICE_ID;
        replyBuffer[replyLength++] = (uint8_t)FWverNo.substring(1, firstDot).toInt();
        replyBuffer[replyLength++] = (uint8_t)FWverNo.substring(firstDot + 1, secondDot).toInt();
        replyBuffer[replyLength++] = (uint8_t)FWverNo.substring(secondDot + 1).toInt();
    }
    else if (currentCommand == CMD_FIRMWARE_IDENTITY) {
        // Fixed 48-byte payload: ID + semantic version + 20-byte build stamp
        // + the STM32 factory 96-bit UID as 24 uppercase hexadecimal bytes.
        int firstDot = FWverNo.indexOf('.');
        int secondDot = FWverNo.indexOf('.', firstDot + 1);
        replyBuffer[replyLength++] = DEVICE_ID;
        replyBuffer[replyLength++] = (uint8_t)FWverNo.substring(1, firstDot).toInt();
        replyBuffer[replyLength++] = (uint8_t)FWverNo.substring(firstDot + 1, secondDot).toInt();
        replyBuffer[replyLength++] = (uint8_t)FWverNo.substring(secondDot + 1).toInt();
        #if MAGAZINE_UART_TEST
        for (uint8_t i = 0; i < 20; ++i) {
            replyBuffer[replyLength++] = i < FWbuildNo.length() ? FWbuildNo[i] : ' ';
        }
        static const char hex[] = "0123456789ABCDEF";
        const volatile uint32_t* uid = reinterpret_cast<const volatile uint32_t*>(0x1FFF7A10UL);
        for (uint8_t word = 0; word < 3; ++word) {
            const uint32_t value = uid[word];
            for (int8_t shift = 28; shift >= 0; shift -= 4) {
                replyBuffer[replyLength++] = hex[(value >> shift) & 0x0F];
            }
        }
        #endif
    }
    else if (currentCommand == CMD_MAGFW_STATUS) {
        replyBuffer[replyLength++] = magFwStatus_;
        replyBuffer[replyLength++] = magFwError_;
        replyBuffer[replyLength++] = (uint8_t)(magFwReceived_ >> 24);
        replyBuffer[replyLength++] = (uint8_t)(magFwReceived_ >> 16);
        replyBuffer[replyLength++] = (uint8_t)(magFwReceived_ >> 8);
        replyBuffer[replyLength++] = (uint8_t)magFwReceived_;
        replyBuffer[replyLength++] = (uint8_t)(magFwExpectedSize_ >> 24);
        replyBuffer[replyLength++] = (uint8_t)(magFwExpectedSize_ >> 16);
        replyBuffer[replyLength++] = (uint8_t)(magFwExpectedSize_ >> 8);
        replyBuffer[replyLength++] = (uint8_t)magFwExpectedSize_;
    }
    else if (currentCommand >= CMD_MAGFW_BEGIN && currentCommand <= CMD_MAGFW_ERASE) {
        // Framed UART write acknowledgement. Destructive work is performed by
        // loop() only after this response has left the UART.
        replyBuffer[replyLength++] = 1;
    }
    else if (currentCommand == CMD_SET_LED_CAL_1 ||
             currentCommand == CMD_SET_LED_CAL_2 ||
             currentCommand == CMD_SET_LED_CAL_ALL) {
        // UART write acknowledgement. I2C writes retain their original
        // endTransmission ACK behavior and never enter handleRequest().
        replyBuffer[replyLength++] = 1;
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
    else if (currentCommand == CMD_CALIBRATE_RESULT_ALL) {
        #if defined(Magazine_PCB_V2) && MAGAZINE_UART_TEST
            replyBuffer[replyLength++] = calibrationStatus_;
            if (calibrationStatus_ == CAL_STATUS_COMPLETE) {
                replyBuffer[replyLength++] = calibrationMagType_;
                for (uint8_t s = 0; s < 20; ++s) {
                    replyBuffer[replyLength++] = (calibrationResults_[s] >> 8) & 0xFF;
                    replyBuffer[replyLength++] = calibrationResults_[s] & 0xFF;
                }
            }
        #else
            replyBuffer[replyLength++] = CAL_STATUS_ERROR;
        #endif
    }
    byte checksum = livoCommunication.calculateCheckSum8bit(replyBuffer, replyLength);
    replyBuffer[replyLength++] = checksum;
    if (uartMode_) {
        uint8_t frame[uartFrameSize] = {0x5A, DEVICE_ID, currentCommand, replyLength};
        memcpy(frame + 4, replyBuffer, replyLength);
        frame[4 + replyLength] = livoCommunication.calculateCheckSum8bit(frame, 4 + replyLength);
        _serial->write(frame, replyLength + 5);
    } else {
        _wire->write(replyBuffer, replyLength);
    }

    finishMagazineMhCommand();
}

int* I2CInstanceSlave::getSmoothedSensorValues() {
    sensorReader_->readAllSensors();
    int* rawValues = sensorReader_->getAllSensorValues();

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
    
    // Create and return new array with smoothed integer values
    static int smoothedValues[24];          // Max 24 sensors 
    for (int i = 0; i < numSensors; ++i) {
        smoothedValues[i] = rawValues[i]<0 ? -1 : static_cast<int>(round(filteredSensorValues_[i]));
    }
    
    return smoothedValues;
}

bool I2CInstanceSlave::isMagazineLedBusIdle() const {
    return !newRequest
        && !magazineMhCommandActive_
        && !lockMagFlag
        && !unlockMagFlag
        && calibrationStatus_ != CAL_STATUS_RUNNING;
}

bool I2CInstanceSlave::captureMagazineLedSnapshot(int outValues[24]) {
    if (outValues == nullptr || !isMagazineLedBusIdle()) return false;

    const bool ownsTrigger = !magazineManualTriggerMode_;
    if (ownsTrigger) {
        setMagazineMhTrigger(true);
        delay(2);
    }

    // A master request may have arrived while the emitters were settling.
    if (!isMagazineLedBusIdle()) {
        if (ownsTrigger && !magazineManualTriggerMode_ && !magazineMhCommandActive_) {
            setMagazineMhTrigger(false);
        }
        return false;
    }

    int* values = getSmoothedSensorValues();
    for (uint8_t i = 0; i < 24; ++i) outValues[i] = values[i];
    updateLocalMagazineDetection(values);

    if (ownsTrigger && !magazineManualTriggerMode_ && !magazineMhCommandActive_) {
        setMagazineMhTrigger(false);
    }
    return true;
}

void I2CInstanceSlave::handleReceive(int howMany) {
    receiveLength = 0;
    if (_wire->available() && howMany >= 1) {
        currentCommand = _wire->read();  // First byte = command
        receiveLength = 1;

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
        } else if (isMagazineMhCommand(currentCommand) && !magazineManualTriggerMode_) {
            setMagazineMhTrigger(true);
            magazineMhCommandActive_ = true;
            magazineMhCommandNeedsLoopCompletion_ =
                (currentCommand == CMD_LOCK_MAGZINE || currentCommand == CMD_UNLOCK_MAGZINE);
        }
    }

    while (_wire->available() && receiveLength < maxBufferSize) {
        receiveBuffer[receiveLength++] = _wire->read();  // if future expansion needs more data
    }

    newRequest = true;
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

void I2CInstanceSlave::serviceUART() {
    static const uint32_t UART_INTERBYTE_TIMEOUT_MS = 25;
    if (uartFrameLength_ && millis() - uartLastByteMs_ > UART_INTERBYTE_TIMEOUT_MS) {
        uartFrameLength_ = 0;
    }
    while (_serial->available()) {
        const uint8_t b = _serial->read();
        uartLastByteMs_ = millis();
        if (!uartFrameLength_ && b != 0xA5) continue;
        if (uartFrameLength_ >= sizeof(uartFrame_)) { uartFrameLength_ = 0; continue; }
        uartFrame_[uartFrameLength_++] = b;
        if (uartFrameLength_ < 5) continue;
        const uint8_t payloadLength = uartFrame_[3];
        const uint16_t frameLength = (uint16_t)payloadLength + 5U;
        // receiveBuffer stores the command byte in addition to the payload.
        if (frameLength > sizeof(uartFrame_) ||
            (size_t)payloadLength + 1U > sizeof(receiveBuffer)) {
            uartFrameLength_ = 0; continue;
        }
        if (uartFrameLength_ != frameLength) continue;
        const bool valid = uartFrame_[frameLength - 1] ==
            livoCommunication.calculateCheckSum8bit(uartFrame_, frameLength - 1);
        if (valid && uartFrame_[1] == DEVICE_ID) {
            currentCommand = uartFrame_[2];
            receiveLength = payloadLength + 1;
            receiveBuffer[0] = currentCommand;
            if (payloadLength) memcpy(receiveBuffer + 1, uartFrame_ + 4, payloadLength);
            #if defined(MAGAZINE_BOOTLOADER_BUILD)
            magFwLastActivityMs_ = millis();
            #endif
            if (currentCommand == CMD_MAGAZINE_TRIGGER_ON) {
                magazineManualTriggerMode_ = true; setMagazineMhTrigger(true);
            } else if (currentCommand == CMD_MAGAZINE_TRIGGER_OFF) {
                magazineManualTriggerMode_ = false; setMagazineMhTrigger(false);
            } else if (currentCommand == CMD_CALIBRATE && calibrationStatus_ == CAL_STATUS_IDLE) {
                calibrationRequested_ = true;
            } else if (currentCommand == CMD_MAGFW_REBOOT) {
                #if defined(MAGAZINE_BOOTLOADER_BUILD)
                magFwStatus_ = MAGFW_STATUS_READY;
                #else
                magFwRebootRequested_ = true;
                #endif
            } else if (isMagazineMhCommand(currentCommand) &&
                       currentCommand != CMD_CALIBRATE &&
                       !magazineManualTriggerMode_) {
                setMagazineMhTrigger(true); magazineMhCommandActive_ = true;
                magazineMhCommandNeedsLoopCompletion_ =
                    currentCommand == CMD_LOCK_MAGZINE || currentCommand == CMD_UNLOCK_MAGZINE;
            }
            newRequest = true;
            const bool magWrite = currentCommand >= CMD_MAGFW_BEGIN &&
                                  currentCommand <= CMD_MAGFW_ERASE &&
                                  currentCommand != CMD_MAGFW_STATUS;
            if (currentCommand != CMD_SET_LED_CAL_1 &&
                currentCommand != CMD_SET_LED_CAL_2 &&
                currentCommand != CMD_SET_LED_CAL_ALL) {
                handleRequest();
                // MAGFW writes need an immediate transport ACK, then remain
                // pending for loop() to perform erase/program/verify safely.
                newRequest = magWrite;
            }
        }
        uartFrameLength_ = 0;
    }
}

void I2CInstanceSlave::loop() {
    if (uartMode_) serviceUART();
    if (magFwRebootRequested_) {
        if (_serial) _serial->flush();
        delay(20);
        NVIC_SystemReset();
    }
    if (newRequest) {
        newRequest = false;

        #if defined(MAGAZINE_BOOTLOADER_BUILD)
        if (_serial && currentCommand >= CMD_MAGFW_BEGIN && currentCommand <= CMD_MAGFW_ERASE) {
            _serial->flush();
        }
        if (currentCommand == CMD_MAGFW_BEGIN) { magFwHandleBegin(); return; }
        if (currentCommand == CMD_MAGFW_ERASE) { magFwHandleErase(); return; }
        if (currentCommand == CMD_MAGFW_DATA) { magFwHandleData(); return; }
        if (currentCommand == CMD_MAGFW_VERIFY) { magFwHandleVerify(); return; }
        if (currentCommand == CMD_MAGFW_COMMIT) {
            if (receiveLength >= 5) magFwCommitMagic_ = magFwReadU32(receiveBuffer + 1);
            magFwHandleCommit(); return;
        }
        if (currentCommand == CMD_MAGFW_ABORT) {
            magFwUpdateActive_ = false; magFwExpectedSize_ = 0; magFwReceived_ = 0;
            magFwStatus_ = MAGFW_STATUS_READY; magFwError_ = MAGFW_ERR_NONE; return;
        }
        if (currentCommand == CMD_MAGFW_REBOOT || currentCommand == CMD_MAGFW_STATUS) return;
        #else
        if (currentCommand >= CMD_MAGFW_BEGIN && currentCommand <= CMD_MAGFW_ERASE) {
            if (currentCommand != CMD_MAGFW_REBOOT && currentCommand != CMD_MAGFW_STATUS) {
                magFwStatus_ = MAGFW_STATUS_UNSUPPORTED;
                magFwError_ = MAGFW_ERR_UNSUPPORTED;
            }
            return;
        }
        #endif

        if ((currentCommand == CMD_SET_LED_CAL_1 || currentCommand == CMD_SET_LED_CAL_2) &&
            receiveLength == 22) {
            const uint8_t magazineType = receiveBuffer[1];
            if (magazineType >= 1 && magazineType <= 7) {
                if (currentCommand == CMD_SET_LED_CAL_1) {
                    ledThresholdsValid_ = false;
                }
                if (pendingLedThresholdType_ != magazineType) {
                    pendingLedThresholdType_ = magazineType;
                    pendingLedThresholdParts_ = 0;
                }
                const uint8_t offset = (currentCommand == CMD_SET_LED_CAL_1) ? 0 : 10;
                for (uint8_t i = 0; i < 10; ++i) {
                    pendingLedThresholds_[offset + i] =
                        ((uint16_t)receiveBuffer[2 + i * 2] << 8) |
                        receiveBuffer[3 + i * 2];
                }
                pendingLedThresholdParts_ |= (currentCommand == CMD_SET_LED_CAL_1) ? 0x01 : 0x02;
                if (pendingLedThresholdParts_ == 0x03) {
                    magazineLED.setThresholds(magazineType, pendingLedThresholds_);
                    ledThresholdsValid_ = true;
                    pendingLedThresholdParts_ = 0;
                    Serial.print("LED CAL applied type ");
                    Serial.println(magazineType);
                }
            }
            if (uartMode_) handleRequest();
            return;
        }

        if (currentCommand == CMD_SET_LED_CAL_ALL && receiveLength == 42) {
            const uint8_t magazineType = receiveBuffer[1];
            if (magazineType >= 1 && magazineType <= 7) {
                pendingLedThresholdType_ = magazineType;
                for (uint8_t i = 0; i < 20; ++i) {
                    pendingLedThresholds_[i] =
                        ((uint16_t)receiveBuffer[2 + i * 2] << 8) |
                        receiveBuffer[3 + i * 2];
                }
                magazineLED.setThresholds(magazineType, pendingLedThresholds_);
                ledThresholdsValid_ = true;
                pendingLedThresholdParts_ = 0;
                Serial.print("LED CAL applied type ");
                Serial.println(magazineType);
            }
            if (uartMode_) handleRequest();
            return;
        }

        // Optional: Print or handle internally
        if (onReceiveCallback) {
            onReceiveCallback(receiveBuffer, receiveLength);
        } else {
            // Default behavior: echo back
            setReplyData(receiveBuffer, receiveLength);
        }
    }
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

        if (magType == 0 || !SensorFaultPolicy::adcValid(s21) || !SensorFaultPolicy::adcValid(s22) || !SensorFaultPolicy::adcValid(s23)) {
            Serial.println("CAL Error: no magazine detected or type sensors invalid");
            calibrationStatus_ = CAL_STATUS_ERROR;
        } else {
            Serial.print("CAL: Magazine 00"); Serial.print(magType);
            Serial.print(", Holder ");
            Serial.println((DEVICE_ID == Stainer_Magzine1_PCB_ID) ? 1 : 2);

            const int NUM_RUNS    = 2;
            const int NUM_BATCHES = 5;
            const int BATCH_SIZE  = 100;
            const unsigned long SAMPLE_INTERVAL_MS = 10;

            long finalSum[20] = {0}; bool calibrationSamplesValid=true;

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
                            const int value=analogRead(sensorPins[s]);
                            calibrationSamplesValid &= SensorFaultPolicy::adcValid(value);
                            if(SensorFaultPolicy::adcValid(value)) batchSum[s] += value;
                        }
                        // I2C status reads were served by an interrupt while this
                        // calibration loop ran. UART has no receive interrupt
                        // callback, so service framed status requests here.
                        while (millis() - t < SAMPLE_INTERVAL_MS) {
                            if (uartMode_) serviceUART();
                        }
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
            calibrationStatus_ = calibrationSamplesValid ? CAL_STATUS_COMPLETE : CAL_STATUS_ERROR;
            Serial.println(calibrationSamplesValid?"CAL: complete":"CAL: FAILED invalid sensor samples");
        }
    }
    #endif

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

    #if defined(MAGAZINE_BOOTLOADER_BUILD)
    if (!magFwUpdateActive_ && millis() - magFwLastActivityMs_ >= MAGFW_BOOT_WINDOW_MS) {
        magFwJumpToApp();
    }
    return;
    #endif
}
#endif

