/*
* CommunicationConstants.h
*
*  Created on: April 28, 2025
*      Author: Varalakshmi
*/

#ifndef COMMUNICATIONCONSTANTS_H_
#define COMMUNICATIONCONSTANTS_H_

#include <Arduino.h>

// Device IDs
const byte Stainer_PCB_ID               = 0x80;         // 128
const byte A700_PCB_ID                  = 0x81;         // 129
const byte Smearer_PCB_ID               = 0x82;         // 130
const byte A1000_App_ID                 = 0x83;         // 131
const byte A1000_PCB1_ID                = 0x84;         // 132
const byte A1000_PCB2_ID                = 0x85;         // 133
const byte A1000_PCB3_ID                = 0x86;         // 134
const byte RFID_App_ID                  = 0x87;         // 135
const byte MasterSlave_PCB_v10          = 0x88;         // 136
const byte MasterSlave_PCB_v11          = 0x89;         // 137

const byte MasterSlave1_PCB_ID          = 0x8A;         // 138
const byte MasterSlave2_PCB_ID          = 0x8B;         // 139
const byte MasterSlave3_PCB_ID          = 0x8C;         // 140
const byte MasterSlave4_PCB_ID          = 0x8D;         // 141

const byte Stage_X_Hall_PCB_ID          = 0x8E;         // 142
const byte Stage_Y_Hall_PCB_ID          = 0x8F;         // 143
const byte Z_Hall_PCB_ID                = 0x90;         // 144
const byte C_Hall_PCB_ID                = 0x91;         // 145
const byte T_Hall_PCB_ID                = 0x92;         // 146
const byte Magazine_PCB_ID              = 0x93;         // 147
const byte Gantry_X_Hall_PCB_ID         = 0x94;         // 148
const byte Gantry_Z_Hall_PCB_ID         = 0x95;         // 149
const byte Gripper_PCB_ID               = 0x96;         // 150
const byte Slide_Holder_PCB_ID          = 0x97;         // 151

const byte Magazine_MC1_ID              = 0x98;         // 152
const byte Magazine_MC2_ID              = 0x99;         // 153
const byte Magazine_PCB_V2_ID           = 0x9E;         // 158

const byte LivoScanner_App_ID           = 0x9A;         // 154
const byte TEENSY_LED_DRIVER			= 0x9B;			// 155
const byte TMCDriver_Board_X			= 0x9C;			// 156
const byte TMCDriver_Board_Y			= 0x9D;			// 157

const byte Stainer_Master_PCB_ID        = 0x9F;         // 159
const byte Stainer_Gantry_PCB_ID        = 0xA0;         // 160
const byte Nozzle_Mount_PCB_ID          = 0xA1;         // 161
const byte Stainer_Magzine1_PCB_ID      = 0xA2;         // 162
const byte Stainer_Magzine2_PCB_ID      = 0xA3;         // 163

// Function IDs
const byte Communication_ID 		    = 30;
const byte RFID_UID_ID 				    = 31;
const byte RFID_Read_ID 			    = 32;
const byte RFID_Write_ID 			    = 33;
const byte DRVStepper8825_ID            = 34;
const byte Servo_ID                     = 35;
const byte DC_ID                        = 36;
const byte DigitalSensor_ID             = 37;
const byte Analog1Sensor_ID             = 38;
const byte Analog2Sensor_ID             = 39;
const byte GyroSensor_ID                = 40;
const byte LastAction_ID 			    = 41;
const byte TMC2209_ID                   = 42;
const byte TriggerLED_ID                = 43;
const byte ReadHallSensor_ID 		    = 44;

// Stepper Motor Function IDs
const byte Microscope_Z_MotorID 	    = 45;
const byte Gantry_X_MotorID			    = 46;
const byte Gantry_Z_MotorID			    = 47;
const byte Turret_MotorID			    = 48;
const byte Condenser_MotorID		    = 49;
const byte Iris_MotorID 			    = 50;
const byte Autolevel_L_MotorID		    = 51;
const byte Autolevel_R_MotorID		    = 52;
const byte Oil_MotorID				    = 53;
const byte Deoil_MotorID			    = 54;

// Servo Motor Function IDs
const byte X_Axis_MotorID 			    = 55;
const byte Y_Axis_MotorID			    = 56;
const byte StageSlideHolder_MotorID     = 57;
const byte Gripper_MotorID			    = 58;

const byte Copley_X_MotorID			    = 59;
const byte Copley_Y_MotorID			    = 60;
const byte TMC4671_X_MotorID		    = 61;
const byte TMC4671_Y_MotorID		    = 62;

const byte Teensy                       = 63;
const byte FPGA                         = 64;

const byte Encoder                      = 65;

const byte Gantry_R_Motor_ID            = 66;
const byte Gantry_Y_Motor_ID            = 67;
const byte Stain_X_Motor_ID             = 68;
const byte Stain_Y_Motor_ID             = 69;
const byte Wash_X_Motor_ID              = 70;
const byte Wash_Y_Motor_ID              = 71;
const byte Buffer_X_Motor_ID            = 72;
const byte Buffer_Y_Motor_ID            = 73;

// ReadHallSensor Constants
#define CMD_HALL_DATA                   0x01           
#define CMD_POSITION_MAPPED             0x02 
#define CMD_FILTERED_HALL_DATA          0x03
#define CMD_SLIDE_STATUS                0x04
#define CMD_MAGZINE_STATUS              0x05
#define CMD_1_HALL_DATA                 0x06    // First half hall data
#define CMD_1_FILTERED_HALL_DATA        0x07    // First half filtered hall data
#define CMD_2_HALL_DATA                 0x08    // Second half hall data
#define CMD_2_FILTERED_HALL_DATA        0x09    // Second half filtered hall data
#define CMD_LOCK_MAGZINE                0x0A
#define CMD_UNLOCK_MAGZINE              0x0B
#define CMD_UART_LINK_TEST               0x30    // UART test: ID, transport marker, protocol version
#define CMD_FIRMWARE_VERSION             0x31    // Reply: device ID, semantic major, minor, patch
#define CMD_FIRMWARE_IDENTITY            0x32    // UART reply: version + build stamp + 96-bit MCU UID
#define CMD_MAGFW_BEGIN                  0x20
#define CMD_MAGFW_DATA                   0x21
#define CMD_MAGFW_STATUS                 0x22
#define CMD_MAGFW_VERIFY                 0x23
#define CMD_MAGFW_COMMIT                 0x24
#define CMD_MAGFW_ABORT                  0x25
#define CMD_MAGFW_REBOOT                 0x26
#define CMD_MAGFW_ERASE                  0x27
#define CMD_Z_HALL_SNAPSHOT              0x28    // 32-byte status/position/raw snapshot, including checksum
#define CMD_Z_HALL_MODEL_INFO            0x29    // Model identity/range/microsteps, including checksum
#define CMD_Z_HALL_FAST_SNAPSHOT         0x2A    // 20-byte timestamped rolling position, including checksum
#define MAGFW_APP_BASE_ADDRESS           0x08020000UL
#define MAGFW_STATUS_IDLE                0
#define MAGFW_STATUS_READY               1
#define MAGFW_STATUS_RECEIVING           2
#define MAGFW_STATUS_VERIFYING           3
#define MAGFW_STATUS_COMPLETE            4
#define MAGFW_STATUS_ERROR               5
#define MAGFW_STATUS_UNSUPPORTED         6
#define MAGFW_STATUS_ERASING             7
#define MAGFW_ERR_NONE                   0
#define MAGFW_ERR_BAD_STATE              1
#define MAGFW_ERR_BAD_ARGS               2
#define MAGFW_ERR_BAD_SEQ                3
#define MAGFW_ERR_CRC                    4
#define MAGFW_ERR_FLASH                  5
#define MAGFW_ERR_VERIFY                 6
#define MAGFW_ERR_UNSUPPORTED            7
#define CMD_MAGAZINE_TRIGGER_ON         0x0C
#define CMD_MAGAZINE_TRIGGER_OFF        0x0D
#define CMD_SOFT_RESET                  0x0E
#define CMD_CALIBRATE                   0x0F    // Write: start calibration. Read: returns CAL_STATUS_* byte
#define CMD_CALIBRATE_RESULT_1          0x10    // [status][magType][S1..S10 x2B each][cs] = 23 bytes
#define CMD_CALIBRATE_RESULT_2          0x11    // [S11..S20 x2B each][cs] = 21 bytes
#define CMD_MAGAZINE_EVENT              0x12    // [event][newType][previousType][cs]; read clears holder INT
#define CMD_SET_LED_CAL_1                0x13    // Write: [type][S1..S10 x 2B]
#define CMD_SET_LED_CAL_2                0x14    // Write: [type][S11..S20 x 2B]
#define CMD_SET_LED_CAL_ALL              0x15    // UART write: [type][S1..S20 x 2B]
#define CMD_CALIBRATE_RESULT_ALL         0x16    // UART read: [status][magType][S1..S20 x2B]

#define MAG_EVENT_NONE                  0
#define MAG_EVENT_INSERTED              1
#define MAG_EVENT_REMOVED               2
#define MAG_EVENT_SWAPPED               3

// Calibration status byte values (returned by CMD_CALIBRATE and CMD_CALIBRATE_RESULT_1)
#define CAL_STATUS_IDLE                 0
#define CAL_STATUS_RUNNING              1
#define CAL_STATUS_COMPLETE             2
#define CAL_STATUS_ERROR                3

#define ReadType_XY                     0        
#define ReadType_X                      1       
#define ReadType_Y                      2       
#define ReadType_MicroscopeZ            3        
#define ReadType_C                      4        
#define ReadType_T                      5        
#define ReadType_Gantry_X               6        
#define ReadType_Gantry_Z               7  
#define ReadType_Gripper                8
#define ReadType_SlideHolder            9

// Stepper Motor Commands
const byte goHome                       = 0;     
const byte enable                       = 1;   
const byte isEnabled                    = 2;    
const byte getPosition                  = 3; 
const byte getDistancetogo              = 4;
const byte getSpeed                     = 5;  
const byte getAccel                     = 6;  
const byte getDirection                 = 7;
const byte getOffset                    = 8;
const byte isHomed                      = 9; 
const byte moveAbsolute                 = 10;    
const byte moveRelative                 = 11;
const byte stop                         = 12;

const byte Settings                     = 13;  

// Oriental
const byte getFullStep                  = 14; 
const byte getAlarm                     = 15;  
const byte getTime                      = 16;  

// TMC2209
const byte getIRUN                      = 17;     
const byte getIHOLD                     = 18;   
const byte getMicroStep                 = 19;     
const byte enableSprdCycle              = 20;
const byte isenabledSprdCycle           = 21;

// Copley
const byte getDeceleration              = 22;   
const byte getJerk                      = 23;
const byte getHomingVelocityfast        = 24;
const byte getHomingVelocityslow        = 25;
const byte getHomingAcceleration        = 26;

const byte getHomingOffset              = 27;           // COMMON

const byte getPID                       = 28;
const byte resetDrive                   = 29;

const byte MovewithEncoder              = 30;
const byte CalibrateEncoder             = 31;
const byte ExitClosedLoop               = 32;
const byte getSpeedOffsetwithEncoder    = 33;
const byte getCountOffsetwithEncoder    = 34;

const byte getCp                        = 35;
const byte getCi                        = 36;
const byte getVp                        = 37;
const byte getVi                        = 38;
const byte getPp                        = 39;
const byte getPi                        = 40;

// Sub Setting Commands
const byte setPosition                  = 0;
const byte setOffset                    = 1;  
const byte setSpeed                     = 2;    
const byte setAccel                     = 3; 
const byte setDirection                 = 4;

// TMC2209
const byte setMicroStep                 = 5;   
const byte setIRUN                      = 6;   
const byte setIHOLD                     = 7;  

// Oriental
const byte settoFullStep                = 8;

// Copley
const byte setDeceleration              = 9;
const byte setJerk                      = 10;
const byte setHomingVelocityfast        = 11;
const byte setHomingVelocityslow        = 12;
const byte setHomingAcceleration        = 13;

const byte setHomingOffset              = 14;           // COMMON

const byte setPID                       = 15;

// Common sub settings
const byte setSpeedOffsetwithEncoder    = 16;
const byte setCountOffsetwithEncoder    = 17;

// Teensy or FPGA Command Types
const byte FPGA_set                     = 0;
const byte FPGA_get                     = 1;

const byte setCp                        = 18;
const byte setCi                        = 19;
const byte setVp                        = 20;
const byte setVi                        = 21;
const byte setPp                        = 22;
const byte setPi                        = 23;

// Teensy or FPGA Sub Commands
const byte FPGA_OperatingMode           = 0;
const byte FPGA_ArrayDepth              = 1;
const byte FPGA_InitialPosX             = 2;
const byte FPGA_InitialPosY             = 3;
const byte FPGA_InitialPosZ             = 4;
const byte FPGA_StepCountX              = 5;
const byte FPGA_StepCountY              = 6;
const byte FPGA_StepCountZ              = 7;
const byte FPGA_PosX                    = 8;
const byte FPGA_PosY                    = 9;
const byte FPGA_PosZ                    = 10;
const byte FPGA_PulseX                  = 11;
const byte FPGA_PulseY                  = 12;
const byte FPGA_PulseZ                  = 13;
const byte FPGA_TargetX                 = 14;
const byte FPGA_TargetY                 = 15;
const byte FPGA_TargetZ                 = 16;

// Extra FPGA Commands
const byte FPGA_GridX                   = 17;
const byte FPGA_GridY                   = 18;
const byte FPGA_clearGridXY             = 19;
const byte FPGA_clearArrayZ             = 20;
const byte FPGA_clearErrors             = 21;
const byte FPGA_LED_Trigger_Mode        = 22;
const byte FPGA_LED_Pulse               = 23;
const byte FPGA_Camera_Pulse            = 24;
const byte FPGA_Missed_Pulse            = 25;

// Encoder IDs
const byte Autolevel_L_Encoder          = 0;
const byte Autolevel_R_Encoder          = 1;
const byte Microscope_Z_Encoder         = 2;
const byte Iris_Encoder                 = 3;
const byte Gantry_X_Encoder             = 4;
const byte Gantry_Z_Encoder             = 5;

// Encoder Sub Commands
const byte getEncoderCount              = 0;
const byte setEncoderCount              = 1;
const byte getEncoderIndexCount         = 2;
const byte getEncoderDirection          = 3;
const byte setEncoderDirection          = 4;
const byte setautoUpdate                   = 5;

#endif /* COMMUNICATIONCONSTANTS_H_ */


