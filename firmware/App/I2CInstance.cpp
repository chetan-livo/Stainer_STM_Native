#include "Stm32SerialCompat.h"
#include "SensorFault.h"
#include "I2CInstance.h"

I2CInstance* Instance = nullptr;

namespace {
constexpr uint8_t UART_REQUEST_SOF = 0xA5;
constexpr uint8_t UART_RESPONSE_SOF = 0x5A;
constexpr uint32_t UART_INTERBYTE_TIMEOUT_MS = 25;

uint8_t wireMasterAddress(uint8_t address) {
    return address > 0x7F ? address & 0x7F : address;
}
}

I2CInstance::I2CInstance(TwoWire& wire) : _wire(&wire) { Instance = this; }
I2CInstance::I2CInstance(LivoHardwareSerial& serial)
    : _wire(nullptr), _serial(&serial), uartMode(true) {}

void onReceiveHandle(int count) {
    if (Instance) Instance->handleReceive(count);
}

void I2CInstance::handleReceive(int) {
    while (_wire && _wire->available()) (void)_wire->read();
}

void I2CInstance::setup(int sda, int scl) {
    _wire->setSDA(sda);
    _wire->setSCL(scl);
    _wire->begin();
}

void I2CInstance::setup(int sda, int scl, uint8_t address) {
    _wire->setSDA(sda);
    _wire->setSCL(scl);
    // Explicit slave-mode arguments avoid the core 3 ArduinoCore-API overload.
    _wire->begin(address, false, false);
    _wire->onReceive(onReceiveHandle);
}

void I2CInstance::setupUART(uint32_t baud) { _serial->begin(baud); }

bool I2CInstance::requestSensorData(uint8_t address, uint8_t command, uint8_t length) {
    if (state != IDLE || length + 1U > sizeof(dataBuffer)) return false;
    targetAddress = address;
    commandByte = command;
    bytesToRead = length + 1; // legacy payload checksum remains part of the reply
    bytesRead = 0;
    newDataAvailable = false;
    memset(dataBuffer, 0, sizeof(dataBuffer));
    state = WRITE_COMMAND;
    return true;
}

bool I2CInstance::writeCommandData(uint8_t address, uint8_t command,
                                   const uint8_t* data, uint8_t length) {
    if (state != IDLE || newDataAvailable || (length && !data)) return false;

    if (!uartMode) {
        if (length > 31) return false;
        _wire->beginTransmission(wireMasterAddress(address));
        _wire->write(command);
        if (length) _wire->write(data, length);
        return _wire->endTransmission(true) == 0;
    }

    if (length + 1U > transportBufferSize) return false;
    uint8_t frame[uartFrameSize] = {UART_REQUEST_SOF, address, command, length};
    if (length) memcpy(frame + 4, data, length);
    frame[4 + length] = livoCommunication.calculateCheckSum8bit(frame, 4 + length);

    while (_serial->available()) (void)_serial->read();
    if (_serial->write(frame, length + 5U) != length + 5U) return false;

    // Serial.write() only proves that bytes entered the local TX queue. Wait for
    // the holder ACK so calibration writes can detect a missing/rejected frame.
    uint8_t ack[uartFrameSize] = {};
    uint8_t ackLength = 0;
    uint32_t lastByteMs = millis();
    const uint32_t startedMs = lastByteMs;
    while (millis() - startedMs <= READ_TIMEOUT_MS) {
        if (ackLength && millis() - lastByteMs > UART_INTERBYTE_TIMEOUT_MS) ackLength = 0;
        while (_serial->available()) {
            const uint8_t value = _serial->read();
            lastByteMs = millis();
            if (!ackLength && value != UART_RESPONSE_SOF) continue;
            if (ackLength >= sizeof(ack)) {
                ackLength = 0;
                continue;
            }
            ack[ackLength++] = value;
            if (ackLength < 5) continue;
            const uint8_t payloadLength = ack[3];
            const uint16_t fullLength = (uint16_t)payloadLength + 5U;
            if (fullLength > sizeof(ack)) {
                ackLength = 0;
                continue;
            }
            if (ackLength != fullLength) continue;

            const bool outerChecksumOk = ack[fullLength - 1] ==
                livoCommunication.calculateCheckSum8bit(ack, fullLength - 1);
            const bool addressedReply = ack[1] == address && ack[2] == command;
            const bool ackPayloadOk = payloadLength == 2 && ack[4] == 1 &&
                ack[5] == livoCommunication.calculateCheckSum8bit(ack + 4, 1);
            if (outerChecksumOk && addressedReply && ackPayloadOk) return true;
            ackLength = 0;
        }
        delay(1);
    }
    return false;
}

void I2CInstance::markNextPacketAsRouted() { routeNextPacket = true; }

void I2CInstance::loop() {
    if (uartMode) {
        if (state == WRITE_COMMAND) {
            uint8_t frame[5] = {UART_REQUEST_SOF, targetAddress, commandByte, 0, 0};
            frame[4] = livoCommunication.calculateCheckSum8bit(frame, 4);
            while (_serial->available()) (void)_serial->read();
            _serial->write(frame, sizeof(frame));
            uartFrameLength = 0;
            uartLastByteMs = millis();
            stateStartMs = millis();
            state = RECEIVING;
        }

        if (uartFrameLength && millis() - uartLastByteMs > UART_INTERBYTE_TIMEOUT_MS) {
            uartFrameLength = 0;
        }

        while (_serial->available()) {
            const uint8_t value = _serial->read();
            uartLastByteMs = millis();
            if (!uartFrameLength && value != UART_RESPONSE_SOF) continue;
            if (uartFrameLength >= sizeof(uartFrame)) {
                uartFrameLength = 0;
                continue;
            }
            uartFrame[uartFrameLength++] = value;
            if (uartFrameLength < 5) continue;

            const uint8_t payloadLength = uartFrame[3];
            const uint16_t fullLength = (uint16_t)payloadLength + 5U;
            if (fullLength > sizeof(uartFrame)) {
                uartFrameLength = 0;
                continue;
            }
            if (uartFrameLength != fullLength) continue;

            const bool checksumOk = uartFrame[fullLength - 1] ==
                livoCommunication.calculateCheckSum8bit(uartFrame, fullLength - 1);
            const bool expectedReply = state == RECEIVING && payloadLength > 0 &&
                uartFrame[1] == targetAddress &&
                uartFrame[2] == commandByte && payloadLength == bytesToRead;
            // A completed background poll may arrive after an explicit request
            // replaced it. Ignore that valid stale frame and keep waiting for
            // this request; its timeout still reports a missing/wrong board.
            if(!checksumOk && state == RECEIVING) {
                char channel[12];snprintf(channel,sizeof(channel),"BUS-%02X",targetAddress);
                SensorFault::report("LIVO-COM-007",channel,commandByte,"invalid_reply_frame");
            }
            if (checksumOk && expectedReply) {
                memcpy(dataBuffer, uartFrame + 4, payloadLength);
                bytesRead = payloadLength;
                lastResponseTarget = targetAddress;
                newDataAvailable = dataBuffer[payloadLength - 1] ==
                    livoCommunication.calculateCheckSum8bit(dataBuffer, payloadLength - 1);
                state = IDLE;
            }
            uartFrameLength = 0;
        }

        if (state == RECEIVING && millis() - stateStartMs > READ_TIMEOUT_MS) {
            char channel[12];snprintf(channel,sizeof(channel),"BUS-%02X",targetAddress);
            SensorFault::report("LIVO-COM-005",channel,commandByte,"uart_response_timeout");
            uartFrameLength = 0;
            newDataAvailable = false;
            bytesRead = 0;
            state = IDLE;
        }
        return;
    }

    switch (state) {
        case WRITE_COMMAND:
            _wire->beginTransmission(wireMasterAddress(targetAddress));
            _wire->write(commandByte);
            {
                const uint8_t error=_wire->endTransmission(true);
                if(error) {
                    char channel[12];snprintf(channel,sizeof(channel),"BUS-%02X",targetAddress);
                    SensorFault::report("LIVO-COM-010",channel,error,"i2c_command_failed");
                    newDataAvailable=false;state=IDLE;break;
                }
            }
            state = REQUESTED;
            break;
        case REQUESTED: {
            const uint8_t received = _wire->requestFrom(wireMasterAddress(targetAddress), bytesToRead);
            bytesRead = 0;
            if (!received) state = IDLE;
            else {
                stateStartMs = millis();
                state = RECEIVING;
            }
            break;
        }
        case RECEIVING:
            while (_wire->available() && bytesRead < bytesToRead) {
                dataBuffer[bytesRead++] = _wire->read();
            }
            if (bytesRead == bytesToRead) {
                newDataAvailable = dataBuffer[bytesToRead - 1] ==
                    livoCommunication.calculateCheckSum8bit(dataBuffer, bytesToRead - 1);
                if (newDataAvailable) lastResponseTarget = targetAddress;
                else {
                    char channel[12];snprintf(channel,sizeof(channel),"BUS-%02X",targetAddress);
                    SensorFault::report("LIVO-COM-007",channel,commandByte,"i2c_checksum_invalid");
                }
                state = IDLE;
            } else if (millis() - stateStartMs > READ_TIMEOUT_MS) {
                char channel[12];snprintf(channel,sizeof(channel),"BUS-%02X",targetAddress);
                SensorFault::report("LIVO-COM-005",channel,commandByte,"i2c_response_timeout");
                while (_wire->available()) (void)_wire->read();
                newDataAvailable = false;
                bytesRead = bytesToRead = 0;
                state = IDLE;
            }
            break;
        default:
            break;
    }
}

bool I2CInstance::hasNewData() const { return newDataAvailable; }

const uint8_t* I2CInstance::getData() {
    newDataAvailable = false;
    return dataBuffer;
}

uint8_t I2CInstance::getDataLength() const { return bytesToRead; }

void I2CInstance::clearPendingData() {
    if (uartMode) while (_serial->available()) (void)_serial->read();
    else while (_wire->available()) (void)_wire->read();
    newDataAvailable = false;
    bytesRead = bytesToRead = lastResponseTarget = uartFrameLength = 0;
    state = IDLE;
}

bool I2CInstance::available() { return uartMode ? _serial->available() : _wire->available(); }
uint8_t I2CInstance::read() { return uartMode ? _serial->read() : _wire->read(); }
I2CInstance::~I2CInstance() {}
