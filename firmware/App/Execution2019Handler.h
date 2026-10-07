/*
* Execution2019Handler.h
*
* Created on: June 11, 2025
* Author: Varalakshmi
*/

#ifndef EXECUTION2019HANDLER_H_
#define EXECUTION2019HANDLER_H_

#include "Protocol2019Handler.h"
#include "TMCModule.h"
#include "AckManager.h"
#include "Constants.h"

class Execution2019Handler {
private:
    bool isFirstPrint = true;
public:
    Execution2019Handler();
    virtual ~Execution2019Handler();

    void beginControllerLink();
    void setup();
    void loop();
    
    void displayPacketValues();
    void performPacketActions();
    void performGantryCommands();
    void performNozzleMountCommands();
    void performStainerMasterCommands();

    void displayHelpMenu();
    bool isHallSensorCommand();
    inline void printIfValid(const char* label, int value);

    // Master-side prefix router: routes "G; ...", "N; ...", "M; ..." to the right PCB.
    // No-prefix lines execute locally for backward compatibility. See Execution2019Handler.cpp.
    void serviceMasterCommandRouter();
};

extern Execution2019Handler execution2019Handler;

#endif /* EXECUTION2019HANDLER_H_ */
