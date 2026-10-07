#include "Stm32SerialCompat.h"
/*
* TMCModule.h
*
*  Created on: May 21, 2025
*      Author: Varalakshmi
*/

#ifndef TMCMODULE_H_
#define TMCMODULE_H_

#include "LivoModule.h"
#include "Constants.h"
#include "HeaderPCB.h" 
#include "TMCMotor.h"
#include "TMCConstants.h"
#include "LimitSwitch.h"

class TMCModule : public LivoModule{
public:
    TMCModule(const TMCConfig& config, uint8_t stepPin, uint8_t dirPin, uint8_t enPin, LivoHardwareSerial& serial, uint8_t driverAddress);
    virtual ~TMCModule();

    void setup();
    void loop();
    // LOOPSTAT step-gap tracking; off for the bed, which pauses on each barrier.
    void setStepGapTracked(bool tracked) { stepGapTracked = tracked; }
    // Where loop() is deliberately skipped to freeze a motor, the step
    // interrupt must be told explicitly; the next loop() releases it.
    void holdSteps() { motor.holdSteps(); }

    void moveTo(long value);
    void move(long value);
    void stop();
    void homing(); 

    bool isHomed() const;
    long getDistanceToGo();
    void setMaxSpeed(long value); 
    long getMaxSpeed() const;
    void setAcceleration(long acceleration);
    long getAcceleration() const;
    void setCurrentPosition(long position);
    long getCurrentPosition() const;
    bool isMoving() const;
    void enableMotor(bool enable);
    bool isMotorEnabled() const;
    void setRMSCurrentIRUN(long value);
    long getRMSCurrentIRUN() const;
    void setRMSCurrentIHOLD(long value); 
    long getRMSCurrentIHOLD() const;

    void stopAtIndexPulse(bool isAtIndexPulse); 
    long getindexPulseAtStep() const; 

    void setMicrosteps(int value); 
    int getMicrosteps() const;

    long getOffset() const override;
    void setOffset(long position) override;

    long getHomingOffset() const override;
    void setHomingOffset(long offset) override;

    void markActionSuccess(bool status);
    bool wasLastActionSuccessful() const;
    
    long getDefaultMaxSpeed() const override;
    long getDefaultMaxAcceleration() const override;
    
    TMCMotor motor; 

private:
    uint8_t driverAddress;
    uint8_t stepPin, dirPin, enPin;
    bool actionSuccessFlag = false;
    bool Enabled = false; 
    long indexPulseAtStep = 0; 
    bool ishomed = false;
    bool isHoming = false;
    bool ignoreIndex = false;
    int microsteps;
    long speed;
    long acceleration;
    long rms_current_I_RUN;
    long rms_current_I_HOLD;
    uint32_t stepServiceCycles = 0, stepServiceGeneration = 0; // LOOPSTAT
    bool stepGapTracked = true;

    byte motorId;
};

#ifdef Stainer_Gantry_PCB
    extern TMCModule GXMotor;
    extern TMCModule GZMotor;
    extern TMCModule GRMotor;
    extern TMCModule GYMotor;
    // Extra Motor
    extern TMCModule XMotor;
#endif

#ifdef Nozzle_Mount_PCB
    extern TMCModule SXMotor;
    extern TMCModule SYMotor;
    extern TMCModule WXMotor;
    extern TMCModule WYMotor;
    extern TMCModule BXMotor;
    extern TMCModule BYMotor;
#endif

#ifdef Stainer_Master_PCB
    extern TMCModule ZMotor;
    extern TMCModule TMotor;
    extern TMCModule M14Motor;
    extern TMCModule M15Motor;
    extern TMCModule XMotor;
    extern TMCModule YMotor;
    extern TMCModule M17Motor;
    extern TMCModule M18Motor;
#endif

#endif /* TMCMODULE_H_ */
