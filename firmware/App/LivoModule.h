/*
* LivoModule.h
*
*  Created on: May 26, 2025
*      Author: Varalakshmi
*/

#ifndef LIVOMODULE_H_
#define LIVOMODULE_H_

#include "Constants.h"

class LivoModule {
public:
    LivoModule();
    virtual void move(long value) {}
    virtual void moveTo(long value) {}
    virtual void stop() {}
    virtual void homing() {}
    virtual bool isHomed() const { return false; } // Made const
    virtual long getDistanceToGo() { return 0; }
    virtual void setMaxSpeed(long value) {}
    virtual long getMaxSpeed() const { return 0; }
    virtual void setAcceleration(long value) {}
    virtual long getAcceleration() const { return 0; } 
    virtual void setCurrentPosition(long value) {}
    virtual void getCopleyCurrentPosition() {}
    virtual long getCurrentPosition() const { return 0; } // Made const
    virtual bool isMoving() const { return false; }
    virtual void enableMotor(bool enable) {}
    virtual bool isMotorEnabled() const { return false; } // Made const
    virtual void setRMSCurrentIRUN(long value) {}
    virtual long getRMSCurrentIRUN() const { return 0; } // Made const
    virtual void setRMSCurrentIHOLD(long value) {}
    virtual long getRMSCurrentIHOLD() const { return 0; } // Made const
    virtual void setMicrosteps(int value) {}
    virtual int getMicrosteps() const { return 0; } // Made const
    virtual void setToFullsteps(int fullstepSetting)  {}
    virtual int getFullsteps() { return 0; }
    virtual void stopAtIndexPulse(bool value) {}
    virtual long getindexPulseAtStep() { return 0; }

    virtual long getOffset() const { return 0; }
    virtual void setOffset(long position) {}
    
    virtual void markActionSuccess(bool status) {}
    virtual bool wasLastActionSuccessful() const { return true; }

    virtual long getDefaultMaxSpeed() const = 0; // Pure virtual function for default max speed
    virtual long getDefaultMaxAcceleration() const = 0; // Pure virtual function for default max acceleration
    virtual long getHomingOffset() const = 0;
    virtual void setHomingOffset(long offset) = 0;

    virtual ~LivoModule() = default;

    virtual void setcloseLoopMaxSpeed(long value){}
    virtual long getcloseLoopMaxSpeed() const { return 12345; }

    virtual void loop() = 0;

    virtual byte getmotorId() { return 0; }

private:

};

#endif /* LIVOMODULE_H_ */
