/*
* PacketProcessor.h
*
* Created on: Apr 10, 2024
* Author: Faisal
*/

#ifndef PACKETPROCESSOR_H_
#define PACKETPROCESSOR_H_

#include "LivoCommunication.h"
#include "Protocol2019Handler.h"

#include "Execution2019Handler.h"
#include "Constants.h"
#include "TMCModule.h"
#ifdef Master
    #include "I2CInstance.h"
    #include "HallSensorModule.h"
    #include "AccelerometerInstance.h"
#endif
#ifdef Slave
    #include "I2CInstance_Slave.h"
#endif

class PacketProcessor {
private:
    LivoCommunication &livoCommunication;
    
public:
    PacketProcessor(LivoCommunication &comm);
    virtual ~PacketProcessor();
    
    void setup();
    void loop();
        
    void processInputFramePacket();
};

#endif /* PACKETPROCESSOR_H_ */
