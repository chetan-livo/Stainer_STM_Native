/*
 * AckManager.h
 *
 *  Created on: May 27, 2025
 *      Author: Faisal
 */

#ifndef ACKMANAGER_H_
#define ACKMANAGER_H_

#include "Arduino.h"
#include "LivoModule.h"

class AckManager {
public:
    AckManager();
    void requestMovementAck(LivoModule* motor, const char* motorId, unsigned long startTimeUs);
    void loop(bool displayDebug, bool ackAction);
    // Optional check: 0 waits, 1 allows ACK, -1 emits FAIL instead.
    void setMovementVerifier(int (*verify)(LivoModule*)) { movementVerifier = verify; }
private:
    int (*movementVerifier)(LivoModule*) = nullptr;
    struct AckRequest {
        LivoModule* motorInstance;
        char id[4];
        unsigned long movementStartTimeUs;
        bool isActive;

        AckRequest() : motorInstance(nullptr), movementStartTimeUs(0), isActive(false) {
            id[0] = '\0';
        }
    };

    static const int MAX_PENDING_ACKS = 20;
    AckRequest pendingMoveAcks[MAX_PENDING_ACKS];

    AckRequest* findSlotForNewRequest();
};

extern AckManager ackManager;

#endif /* ACKMANAGER_H_ */
