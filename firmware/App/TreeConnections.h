/*
* TreeConnections.h
*
*  Created on: May 12, 2025
*      Author: Varalakshmi
*/

#ifndef TREECONNECTIONS_H_
#define TREECONNECTIONS_H_

#include "Constants.h"
#include <stdint.h>
#include <set>

typedef struct {
  const char* interface;
  byte deviceID;
} Connection;

const Connection StainerMaster[] = {
  {"UART1", Stainer_Gantry_PCB_ID},             
  {"UART2", Nozzle_Mount_PCB_ID}
};

const Connection StainerGantry[] = {
  {"UART1", Stainer_Master_PCB_ID},
  {"I2C1_1", Stainer_Magzine1_PCB_ID},
  {"I2C1_2", Gantry_X_Hall_PCB_ID},
  {"I2C2_1", Stainer_Magzine2_PCB_ID},
  {"I2C2_2", Gantry_Z_Hall_PCB_ID},
  {"GXMotor", Gantry_X_MotorID},
  {"GZMotor", Gantry_Z_MotorID},
  {"GRMotor", Gantry_R_Motor_ID},
  {"GYMotor", Gantry_Y_Motor_ID}
};

const Connection NozzleMount[] = {
  {"UART1", Stainer_Master_PCB_ID},
  {"SXMotor", Stain_X_Motor_ID},
  {"SYMotor", Stain_Y_Motor_ID},
  {"WXMotor", Wash_X_Motor_ID},
  {"WYMotor", Wash_Y_Motor_ID},
  {"BXMotor", Buffer_X_Motor_ID},
  {"BYMotor", Buffer_Y_Motor_ID}
};

class TreeConnections {
  public:
    TreeConnections();
    virtual ~TreeConnections();
};

class I2CInstance;

I2CInstance* getI2CBusForInterface(const char* interfaceName);

const char* getInterfaceForDevice(byte currentMasterID, byte targetID);

byte findConnectedID(byte currentPCBID, const char* interfaceName);

#endif /* TREECONNECTIONS_H_ */