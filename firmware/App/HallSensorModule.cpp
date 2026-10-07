/*
* HallSensorModule.cpp
*
*  Created on: May 17, 2025
*      Author: Varalakshmi
*/

#include "I2CInstance.h"
#include "HallSensorModule.h"
#include "SensorFault.h"

// Mirror Serial.print(...) on every Master build (no-op on master since master has no hall modules)
#ifdef Master
#define Serial mirroredSerial
#endif

#ifdef Master

#ifdef Stainer_Gantry_PCB
    MagazineIrSensorModule magazine1IrModule(DEVICE_ID, Stainer_Magzine1_PCB_ID, 24, &i2c1);
    MagazineIrSensorModule magazine2IrModule(DEVICE_ID, Stainer_Magzine2_PCB_ID, 24, &i2c2);

    MagazineIrSensorModule& Magazine1Ir = magazine1IrModule;
    MagazineIrSensorModule& Magazine2Ir = magazine2IrModule;
#endif

HallSensorModule::HallSensorModule(uint8_t device, uint8_t pcb, int halls, I2CInstance* i2cInstance)
    : deviceID(device), pcbID(pcb), numberOfHalls(halls), i2cPort(i2cInstance) {}

void HallSensorModule::requestHallPosition() {
    if (pcbID == Stainer_Magzine1_PCB_ID || pcbID == Stainer_Magzine2_PCB_ID) return;
    if (!positionRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        if(i2cPort->requestSensorData(pcbID, CMD_POSITION_MAPPED, 4)){
            positionRequestFlag = true;
        }
    }
}

void HallSensorModule::requestAllHallValues() {
    if (pcbID != Stainer_Magzine1_PCB_ID && pcbID != Stainer_Magzine2_PCB_ID) {
        requestHallValues();
        return;
    }

    if (!hallRequestFlag && !fullHallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
#if MAGAZINE_UART_TEST
        RequestedHallNum = numberOfHalls;
        hallStoreOffset = 0;
        silentMode = true;
        if (i2cPort->requestSensorData(pcbID, CMD_HALL_DATA, numberOfHalls * 2)) {
            hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
        } else {
            silentMode = false;
        }
#else
        RequestedHallNum = numberOfHalls / 2;
        hallStoreOffset = 0;
        silentMode = true;
        if (i2cPort->requestSensorData(pcbID, CMD_1_HALL_DATA, numberOfHalls)) {
            hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            fullHallRequestFlag = true;
        } else {
            silentMode = false;
        }
#endif
    }
}

void HallSensorModule::requestHallValues() {
    if (pcbID == Stainer_Magzine1_PCB_ID || pcbID == Stainer_Magzine2_PCB_ID) {
#if MAGAZINE_UART_TEST
        if (!hallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
            RequestedHallNum = numberOfHalls;
            hallStoreOffset = 0;
            if (i2cPort->requestSensorData(pcbID, CMD_HALL_DATA, numberOfHalls * 2)) {
                hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            }
        }
#else
        requestHalfHallValues(1);
#endif
        return;
    }
    if (!hallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        RequestedHallNum = numberOfHalls;
        hallStoreOffset = 0;
        if(i2cPort->requestSensorData(pcbID, CMD_HALL_DATA, numberOfHalls * 2)) {
            hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
        }
    }
}

void HallSensorModule::requestHalfHallValues(int halfvalue) {
    if (!hallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        RequestedHallNum = numberOfHalls/2;
        hallStoreOffset = (halfvalue == 2) ? numberOfHalls / 2 : 0;
        fullHallRequestFlag = false;
        if (halfvalue == 1) {
            if(i2cPort->requestSensorData(pcbID, CMD_1_HALL_DATA, numberOfHalls)) {
                hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            }
        } else if ( halfvalue == 2) {
            if(i2cPort->requestSensorData(pcbID, CMD_2_HALL_DATA, numberOfHalls)) {
                hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            }
        }
    }
}

void HallSensorModule::requestFilteredHallValues() {
    if (pcbID == Stainer_Magzine1_PCB_ID || pcbID == Stainer_Magzine2_PCB_ID) {
#if MAGAZINE_UART_TEST
        if (!hallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
            RequestedHallNum = numberOfHalls;
            hallStoreOffset = 0;
            if (i2cPort->requestSensorData(pcbID, CMD_FILTERED_HALL_DATA, numberOfHalls * 2)) {
                hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            }
        }
#else
        requestHalfFilteredHallValues(1);
#endif
        return;
    }
    if (!hallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        RequestedHallNum = numberOfHalls;
        hallStoreOffset = 0;
        if(i2cPort->requestSensorData(pcbID, CMD_FILTERED_HALL_DATA, numberOfHalls * 2)) {
            hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
        }
    }
}

void HallSensorModule::requestHalfFilteredHallValues(int halfvalue) {
    if (!hallRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        RequestedHallNum = numberOfHalls/2;
        hallStoreOffset = (halfvalue == 2) ? numberOfHalls / 2 : 0;
        fullHallRequestFlag = false;
        if (halfvalue == 1) {
            if(i2cPort->requestSensorData(pcbID, CMD_1_FILTERED_HALL_DATA, numberOfHalls)) {
                hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            }
        } else if (halfvalue == 2) {
            if(i2cPort->requestSensorData(pcbID, CMD_2_FILTERED_HALL_DATA, numberOfHalls)) {
                hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
            }
        }
    }
}

void HallSensorModule::requestSlideStatus() {
    if (!slideRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        // [state][magazine type][20 holder-determined slots][calibration mask]
        if(i2cPort->requestSensorData(pcbID, CMD_SLIDE_STATUS, (numberOfHalls-4) + 3)) {
            slideRequestFlag = true;
        }
    }
}

void HallSensorModule::requestMagzineStatus() {
    if (!magzineRequestFlag && i2cPort && !i2cPort->hasNewData()) {
        if(i2cPort->requestSensorData(pcbID, CMD_MAGZINE_STATUS, 1)) {
            magzineRequestFlag = true;
        }
    }
}

void HallSensorModule::lockMagzine() {
    if (!lockMagzineFlag && i2cPort && !i2cPort->hasNewData()) {
        if(i2cPort->requestSensorData(pcbID, CMD_LOCK_MAGZINE, 1)) {
            lockMagzineFlag = true;
        }
    }
}

void HallSensorModule::unlockMagzine() {
    if (!unlockMagzineFlag && i2cPort && !i2cPort->hasNewData()) {
        if(i2cPort->requestSensorData(pcbID, CMD_UNLOCK_MAGZINE, 1)) {
            unlockMagzineFlag = true;
        }
    }
}

void HallSensorModule::loop() {
    const char* sensor=pcbID==Stainer_Magzine1_PCB_ID?"HOLDER1":pcbID==Stainer_Magzine2_PCB_ID?"HOLDER2":pcbID==Gantry_X_Hall_PCB_ID?"HALL_X":"HALL_Z";
    if(hallRequestFlag && (uint32_t)(millis()-sensorRequestAt_)>5000UL) {
        SensorFault::report("LIVO-MAG-010",sensor,-1,"response_timeout");
        sensorDataValid_=false; clearPendingRequests();
    }
    // The transport remains request/response for both I2C and UART. A caller
    // explicitly queues the holder data it needs; UART does not imply streaming.

    // Only check for new data if this specific module instance is expecting a response.
    if ((positionRequestFlag || hallRequestFlag || slideRequestFlag || magzineRequestFlag || lockMagzineFlag || unlockMagzineFlag) && i2cPort && i2cPort->hasNewData()) {
        
        // Consume only if this response is for my pcbID
        if (i2cPort->getLastResponseTarget() != pcbID) {
            return; // not for me; let the correct module instance read it
        }

        const uint8_t* data = i2cPort->getData();
        uint8_t len = i2cPort->getDataLength();

        if (positionRequestFlag && len >= 4) {
            position = (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                    ((uint32_t)data[2] << 8) | data[3]);
            if(position==INT32_MIN) {
                SensorFault::report("LIVO-SEN-024",sensor,position,"invalid_position_model_input_or_result");
                positionRequestFlag=false;return;
            }
            if (!suppressResponsePrint_) {
                Serial.print(String(position));
                Serial.print(" CMD");
                Serial.println();
            }
            silentMode = false;
            suppressResponsePrint_ = false;
            positionRequestFlag = false;
        } else if (hallRequestFlag) {
            if (RequestedHallNum<=0 || hallStoreOffset+RequestedHallNum>30 || len<RequestedHallNum*2) {
                SensorFault::report("LIVO-MAG-010",sensor,len,"incomplete_sensor_frame");
                sensorDataValid_=false; clearPendingRequests(); return;
            }
            const int valuesRead = RequestedHallNum;
            for(int i=0;i<valuesRead;++i) {
                long value=(data[2*i]<<8)|data[2*i+1];
                if(!SensorFaultPolicy::adcValid(value)) {
                    char channel[20];snprintf(channel,sizeof(channel),"%s-S%d",sensor,hallStoreOffset+i+1);
                    SensorFault::report("LIVO-SEN-021",channel,value,"remote_adc_invalid");
                    sensorDataValid_=false;clearPendingRequests();return;
                }
            }
            for (int i = 0; i < valuesRead; ++i) {
                hallValues[hallStoreOffset + i] = (data[2 * i] << 8) | data[2 * i + 1];
            }

            // Legacy I2C needs two reads because 24 values exceed Wire's buffer.
            // UART requests all 24 in one frame and never sets fullHallRequestFlag.
            if (fullHallRequestFlag && hallStoreOffset == 0) {
                hallRequestFlag = false;
                RequestedHallNum = numberOfHalls / 2;
                hallStoreOffset = numberOfHalls / 2;

                if (i2cPort->requestSensorData(pcbID, CMD_2_HALL_DATA, numberOfHalls)) {
                    hallRequestFlag = true; sensorRequestAt_=millis(); sensorDataValid_=false;
                    return;
                }

                SensorFault::report("LIVO-MAG-010",sensor,-1,"second_half_request_failed");
                sensorDataValid_=false;clearPendingRequests();return;
            }
            sensorDataValid_=true; ++sensorSequence_;

            const bool shouldPrintFullHallResponse = fullHallRequestFlag && hallStoreOffset != 0;
            if ((!silentMode || shouldPrintFullHallResponse) && !suppressResponsePrint_) {
                const int printStart = fullHallRequestFlag ? 0 : hallStoreOffset;
                const int printCount = fullHallRequestFlag ? numberOfHalls : valuesRead;
                for (int i = 0; i < printCount; ++i) {
                    Serial.print(hallValues[printStart + i]);
                    if (i < printCount - 1) {
                        Serial.print(",");
                    }
                }
                Serial.print(" CMD");
                Serial.println();
            }
            silentMode = false;
            suppressResponsePrint_ = false;
            fullHallRequestFlag = false;
            hallStoreOffset = 0;
            hallRequestFlag = false;
        } else if (slideRequestFlag && len >= (numberOfHalls - 4) + 3) {
            slideStatusState = data[0];
            if(slideStatusState>=3) SensorFault::report("LIVO-MAG-009",sensor,slideStatusState,"holder_sensor_snapshot_invalid");
            slideMagazineType = data[1];
            for (int i = 0; i < numberOfHalls-4; ++i) {
                slideStatus[i] = data[i + 2];
                if(slideStatus[i]>1) {
                    char channel[20];snprintf(channel,sizeof(channel),"%s-S%d",sensor,i+1);
                    SensorFault::report("LIVO-MAG-009",channel,slideStatus[i],"slot_classification_error");
                }
            }
            calibrationValidMask = data[(numberOfHalls - 4) + 2];
            ++slideStatusSequence;
            if (!suppressResponsePrint_) {
                for (int i = 0; i < numberOfHalls-4; ++i) {
                    Serial.print(slideStatus[i]);
                    if (i < numberOfHalls - 5) {
                        Serial.print(",");
                    }
                }
                Serial.print(" CMD");
                Serial.println();
            }
            silentMode = false;
            suppressResponsePrint_ = false;
            slideRequestFlag = false;
        }
        else if (magzineRequestFlag && len >=1) {
            magzineStatus = data[0];
            if(magzineStatus>1) SensorFault::report("LIVO-MAG-009",sensor,magzineStatus,"magazine_type_sensor_invalid");
            if (!suppressResponsePrint_) {
                Serial.print(getPCBLabel(pcbID));
                Serial.print(" MS:");
                Serial.print(magzineStatus);
                Serial.print(" CMD");
                Serial.println();
            }
            silentMode = false;
            suppressResponsePrint_ = false;
            magzineRequestFlag = false;
        }
        else if (lockMagzineFlag && len >=1) {
            lockStatus = data[0];
            if (!suppressResponsePrint_) {
                Serial.print("LS:");
                Serial.print(lockStatus);
                Serial.print(" CMD");
                Serial.println();
            }
            silentMode = false;
            suppressResponsePrint_ = false;
            lockMagzineFlag = false;
        }
        else if (unlockMagzineFlag && len >=1) {
            lockStatus = data[0] == 0;
            if (!suppressResponsePrint_) {
                Serial.print("LS:");
                Serial.print(lockStatus);
                Serial.print(" CMD");
                Serial.println();
            }
            silentMode = false;
            suppressResponsePrint_ = false;
            unlockMagzineFlag = false;
        }
    }
}

long HallSensorModule::getPosition() const {
    return position;
}

const long* HallSensorModule::getHallValues() const {
    return hallValues;
}

const int* HallSensorModule::getSlideStatus() const {
    return slideStatus;
}

const char* HallSensorModule::getPCBLabel(uint8_t id) {
    switch (id) {
        case Stainer_Magzine1_PCB_ID: return "Stainer Magzine 1";
        case Stainer_Magzine2_PCB_ID: return "Stainer Magzine 2";
        case Gantry_X_Hall_PCB_ID: return "Gantry X";
        case Gantry_Z_Hall_PCB_ID: return "Gantry Z";
        default:   return "Unknown";
    }
}

uint8_t HallSensorModule::getPcbID() const {
    return pcbID;
}

int HallSensorModule::getMagzineStatus() const {
    return magzineStatus;
}

void HallSensorModule::clearPendingRequests() {
    positionRequestFlag = false;
    hallRequestFlag = false;
    fullHallRequestFlag = false;
    slideRequestFlag = false;
    magzineRequestFlag = false;
    lockMagzineFlag = false;
    unlockMagzineFlag = false;
    RequestedHallNum = 0;
    hallStoreOffset = 0;
    silentMode = false;
    suppressResponsePrint_ = false;
}

#endif
