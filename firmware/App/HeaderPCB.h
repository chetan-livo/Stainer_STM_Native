/*
* HeaderPCB.h
*
*  Created on: Jan 19, 2026
*      Author: Varalakshmi
*/

#ifndef HEADERPCB_H_
#define HEADERPCB_H_

#include <Arduino.h>
#include "Constants.h"

#ifdef Master
    #ifdef Stainer_Master_PCB
        #include "StainerMasterPCBV1.h"
    #endif
    #ifdef Stainer_Gantry_PCB
        #include "StainerGantryPCBV1.h"
    #endif
    #ifdef Nozzle_Mount_PCB
        #include "NozzleMountPCBV1.h"
    #endif
#endif

#ifdef Slave
    #ifdef Magazine_PCB_V2
        #include "Magazine_PCB_V2.h"
    #endif
    #ifdef Gantry_X_Hall_PCB
        #include "StageXY_Connections.h"
        #include "XYStagePositionModel.h"
    #endif
    #ifdef Gantry_Z_Hall_PCB
        #include "StageXY_Connections.h"
        #include "XYStagePositionModel.h"
    #endif
#endif

#endif /* HEADERPCB_H_ */