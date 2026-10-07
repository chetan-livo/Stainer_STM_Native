/*
 * AckManager.cpp
 *
 *  Created on: May 27, 2025
 *      Author: Faisal
 */

#include "AckManager.h"

// Mirror Serial.print(...) on every Master build:
//   Gantry/Nozzle  → also writes to MasterSerial (uplink to master)
//   Master itself  → also writes to PiSerial    (uplink to ESP32/Pi)
#ifdef Master
#define Serial mirroredSerial
#endif

AckManager ackManager;

AckManager::AckManager() {
    for (int i = 0; i < MAX_PENDING_ACKS; ++i) {
        pendingMoveAcks[i].isActive = false;
        pendingMoveAcks[i].motorInstance = nullptr;
        pendingMoveAcks[i].id[0] = '\0';
        pendingMoveAcks[i].movementStartTimeUs = 0;
    }
}

AckManager::AckRequest* AckManager::findSlotForNewRequest() {
    for (int i = 0; i < MAX_PENDING_ACKS; ++i) {
        if (!pendingMoveAcks[i].isActive) {
            return &pendingMoveAcks[i];
        }
    }
    return nullptr;
}

void AckManager::requestMovementAck(LivoModule* motor, const char* motorId, unsigned long startTimeUs) {
    if (!motor || !motorId || motorId[0] == '\0') return;

    AckRequest* slot = findSlotForNewRequest();
    if (slot) {
        slot->motorInstance = motor;
        strncpy(slot->id, motorId, sizeof(slot->id) - 1);
        slot->id[sizeof(slot->id) - 1] = '\0';
        slot->movementStartTimeUs = startTimeUs;
        slot->isActive = true;
    } else {
        Serial.print("AckManager: Could not track ACK for motor ");
        Serial.println(motorId);
    }
}

void AckManager::loop(bool displayDebug, bool ackAction) {
    for (int i = 0; i < MAX_PENDING_ACKS; ++i) {
        AckRequest& req = pendingMoveAcks[i];

        if (!req.isActive || req.motorInstance == nullptr) continue;

        bool operationConsideredComplete = !req.motorInstance->isMoving();

        if (operationConsideredComplete) {
            const int verified = movementVerifier ? movementVerifier(req.motorInstance) : 1;
            if (verified == 0) continue;
            unsigned long endTimeUs = micros();
            unsigned long timeTakenUs = endTimeUs - req.movementStartTimeUs;
            if (displayDebug) {
                Serial.print(req.id);
                Serial.print("_POS:");
                Serial.print(req.motorInstance->getCurrentPosition());
                Serial.print(" Time:");
                Serial.print(timeTakenUs);
                Serial.print("us ");
            }
            if (ackAction) {
                Serial.print(req.id);
                Serial.println(verified > 0 ? "ACK" : "FAIL");
            }

            req.isActive = false;       // free slot
            req.motorInstance = nullptr;
            req.id[0] = '\0';
            req.movementStartTimeUs = 0;
        }
    }
}
