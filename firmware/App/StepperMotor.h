#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include <AccelStepper.h>
#include "Constants.h" // For general constants, ensure pin definitions are handled appropriately
#include "StepEngine.h"

class StepperMotor {
public:
    // Constructor: takes AccelStepper interface type, step pin, dir pin
    StepperMotor(uint8_t interface, uint8_t stepPin, uint8_t dirPin);
    virtual ~StepperMotor();

    // Pure virtual function: must be implemented by derived classes
    virtual void setupMotor() = 0;
    virtual void homing() = 0; // Homing routine specific to the motor type
    virtual bool ishomed() = 0;

    // Common AccelStepper wrapper functions (virtual so they can be overridden if needed)
    virtual void loopMotor(); // Typically calls accelStepper.run()
    virtual void moveTo(long absolute);
    virtual void move(long relative);
    virtual void setMaxSpeed(float speed); // Setter, remains non-const
    virtual float maxSpeed() const; // Added const
    virtual void setAcceleration(float acceleration); // Setter, remains non-const
    virtual float acceleration() const; // Added const
    virtual void setCurrentPosition(long position);
    virtual long currentPosition() const; // Added const
    virtual long distanceToGo() const; // Added const
    virtual bool isRunning() const; // Added const
    virtual void stop();
    virtual void run();
    virtual void runSpeed();
    virtual void setSpeed(float speed);
    virtual void setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert);

    // Motor enable/disable (can be overridden for specific hardware control)
    virtual void enableOutputs();
    virtual void disableOutputs();
    
    virtual long getOffset() const;
    virtual void setOffset(long position);

    void setHomingOffset(long offset);
    long getHomingOffset() const;

    void setJerk(float value);
    float getJerk();

    void startSCurveMove(long targetSteps, float vMax, float aMax, float jMax);
    bool isSCurveActive() const;

    // Timer-driven stepping (StepEngine.h); polled AccelStepper otherwise.
    // Settings go to both planners so an idle motor can switch either way.
    bool enableInterruptStepping();     // setup, motor idle
    void disableInterruptStepping();    // permanently polled (bed T motor)
    bool setInterruptStepping(bool on); // idle only
    bool interruptStepping() const { return isrMode; }
    void holdSteps() { isr.hold(); }    // freeze until the next loopMotor()
    static bool setAllInterruptStepping(bool on); // STEPISR command
    static uint8_t interruptSteppingCount();
protected:
    void setStepOutputsEnabled(bool enabled) { isr.setDriverEnabled(enabled); }
    IsrStepper isr;
    bool isrCapable = false, isrMode = false;
    mutable AccelStepper accelStepper;
    uint8_t stepPin; 
    uint8_t dirPin; 

    long offset = 0; 
    long homingOffset = 1000; 
    bool isHomed=false;

    float jerk = 10000;

    bool sCurveActive = false;

    long  sStartPos         = 0;    
    long  sTargetPos        = 0;     
    float sDir              = 1.0f;  

    float s_vMax            = 0.0f;  
    float s_aMax            = 0.0f;  
    float s_jMax            = 0.0f;  

    float s_t[7]            = {0};
    float s_tCum[7]         = {0};
    float s_totalTime       = 0.0f;

    float s_vBoundary[7]    = {0};

    unsigned long s_tStartMicros = 0;
    unsigned long s_lastPrintMicros = 0;  

    void updateSCurve();

    float sCurveVelocityAt(float t) const;

    void computeUsableProfile(long distanceSteps, float vReq, float aReq, float jReq, float &vUse, float &aUse, float &jUse);
};

#endif /* STEPPERMOTOR_H_ */