#include "Stm32SerialCompat.h"
#ifndef TMCMOTOR_H_
#define TMCMOTOR_H_

#include "StepperMotor.h"
#include <TMCStepper.h>            // https://github.com/teemuatlut/TMCStepper
#include "Arduino.h"

#define DIR_POSITIVE HIGH         // Set direction pin high for forward
#define DIR_NEGATIVE LOW          // Set direction pin low for reverse

class TMC2209Stepper; // Example for TMC2209

class TMCMotor : public StepperMotor {
public:
    // Constructor might take Serial port for UART, CS pin for SPI, enable pin, driver address, R_SENSE
    TMCMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
             LivoHardwareSerial& serialPort, uint8_t driverAddress, float rSense);
    virtual ~TMCMotor();

    // Implement pure virtual functions
    void setupMotor() override;
    void homing() override;
    bool ishomed() override; // Implementation for the pure virtual function

    // TMC-specific methods
    void setRMSCurrentIRUN(uint16_t mA);
    uint16_t getRMSCurrentIRUN();
    void setRMSCurrentIHOLD(uint16_t mA); // Often a percentage of IRUN or direct setting
    uint16_t getRMSCurrentIHOLD();
    void setMicrostepsTMC(int microsteps); // TMCs have software configurable microstepping

    // Method to set pin inversions for AccelStepper
    void setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert) override;

    // Override enable/disable for TMC specific control (e.g., ENN pin)
    void enableOutputs() override;
    void disableOutputs() override;

    void activateRunCurrent();
    void activateHoldCurrent();
    bool diagnosticStatus(uint32_t& status);

private:
    // Pointer to your specific TMC driver object
    TMC2209Stepper* tmcDriver; // Example, replace with your actual driver type
    LivoHardwareSerial& tmcSerial; // Reference to the serial port for UART
    uint8_t tmcEnablePin;
    uint8_t tmcDriverAddress;
    float tmcRSense;

    // Store current settings if needed by your logic
    uint16_t currentIRUN_mA = 500; // Example default value
    uint16_t currentIHOLD_mA = 100; // Example default value

    // Last current pair written to the driver. rms_current() costs three UART
    // writes, each followed by TMCStepper's fixed 2 ms delay, which stalls every
    // software stepper. TMCModule requests the same pair on every start/stop.
    bool currentApplied = false;
    uint16_t appliedIRUN_mA = 0;
    float appliedHoldMultiplier = 0.0f;
    void applyCurrent(uint16_t mA, float holdMultiplier);
};

#endif /* TMCMOTOR_H_ */
