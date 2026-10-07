#include "Stm32SerialCompat.h"
#include "TMCMotor.h"

bool TMCMotor::diagnosticStatus(uint32_t& status) {
    // TMC2209 IOIN version byte is 0x21. A zero status alone is not proof of a reply.
    if(!tmcDriver || tmcDriver->version()!=0x21)return false;
    status=tmcDriver->DRV_STATUS();
    return tmcDriver->version()==0x21 && status!=0xffffffffUL;
}

// Constructor
TMCMotor::TMCMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                   LivoHardwareSerial& serialPort, uint8_t driverAddress, float rSense)
    : StepperMotor(AccelStepper::DRIVER, stepPin, dirPin),
      tmcSerial(serialPort),
      tmcEnablePin(enablePin),
      tmcDriverAddress(driverAddress),
      tmcRSense(rSense)
      {
    tmcDriver = new TMC2209Stepper(&tmcSerial, tmcRSense, tmcDriverAddress);
}

TMCMotor::~TMCMotor() {
    delete tmcDriver;
}

void TMCMotor::setupMotor() {
    pinMode(tmcEnablePin, OUTPUT);
    digitalWrite(tmcEnablePin, HIGH); // Disable driver initially (most TMC ENN are active LOW)

    tmcSerial.begin(500000); // TMC2209 supports up to 500k — 19200 caused ~4ms blocking per write
    tmcDriver->begin();      

    // GConf:
    uint32_t enableConfig               = 0b1111111111;
    uint32_t disableConfig              = 0b0000000000;
    uint32_t shaftInversemotordirection = 0b0000001000 & disableConfig;
    uint32_t mstep_reg_select           = 0b0010000000 & enableConfig;
    uint32_t pdn_disableUART            = 0b0001000000 & enableConfig;
    uint32_t multistep_filt             = 0b0100000000 & enableConfig;
    uint32_t gconfdata = multistep_filt
                        | pdn_disableUART
                        | mstep_reg_select
                        | shaftInversemotordirection;
    tmcDriver->GCONF(gconfdata);

    delay(500);

    // Set Currents(Irun Ihold IholdDelay)
    tmcDriver->rms_current(currentIRUN_mA, (float)currentIHOLD_mA / currentIRUN_mA);       
    tmcDriver->iholddelay(2);

    // ChopConf:
    enableConfig  = 0xFFFFFFFF;
    uint32_t dedge  = 0b10000000000000000000000000000000 & enableConfig;
    uint32_t intpol = 0b00010000000000000000000000000000 & enableConfig;
    uint32_t mres   = 0b00000000000000000000000000000000 & enableConfig;
    uint32_t vsense = 0b00000000000000100000000000000000 & enableConfig;
    uint32_t chm    = 0b00000000010000000000000000000000 & enableConfig;
    uint32_t tbl    = 0b00000000000000000010000000000000 & enableConfig;
    uint32_t hend   = 0b00000000000000000000001100000000 & enableConfig;
    uint32_t hstrt  = 0b00000000000000000000000001000000 & enableConfig;
    uint32_t toff   = 0b00000000000000000000000000000110 & enableConfig;
    
    uint32_t chopConfData = dedge
                            | intpol
                            | mres
                            | vsense
                            | chm
                            | tbl
                            | hend
                            | hstrt
                            | toff;
    tmcDriver->CHOPCONF(chopConfData);
    currentApplied = false; // CHOPCONF overwrote VSENSE chosen by rms_current().

    // // Basic TMC2209 configuration 
    // tmcDriver->toff(5);       
    // tmcDriver->blank_time(24);
    // // tmcDriver->en_spreadCycle(true);
    // tmcDriver->rms_current(currentIRUN_mA, (float)currentIHOLD_mA / currentIRUN_mA);
    // tmcDriver->microsteps(16); 
    // tmcDriver->TCOOLTHRS(0xFFFFF);
    // tmcDriver->semin(0);   

    // tmcDriver->en_spreadCycle(false);
    // tmcDriver->pwm_autoscale(true);
    // tmcDriver->IHOLD_IRUN((2UL << 16) | (10UL << 8) | 10UL);
    // tmcDriver->shaft(false);    

    // accelStepper.setMaxSpeed(20000);
    // accelStepper.setAcceleration(100000);
}

void TMCMotor::homing() {
    // Implement homing for TMC, potentially using StallGuard or limit switches
    // Serial.println("TMCMotor: Homing sequence initiated (implement actual logic).");
}

bool TMCMotor::ishomed() {
    // Implement logic to check if the TMC motor is homed.
    // This might involve checking a flag set by the homing() routine,
    // or reading a sensor status if applicable.
    // Serial.println("TMCMotor: ishomed()");
    return false;
}

void TMCMotor::applyCurrent(uint16_t mA, float holdMultiplier) {
    // Identical inputs produce identical CHOPCONF.VSENSE and IHOLD_IRUN values.
    if (currentApplied && appliedIRUN_mA == mA && appliedHoldMultiplier == holdMultiplier) return;
    tmcDriver->rms_current(mA, holdMultiplier);
    currentApplied = true;
    appliedIRUN_mA = mA;
    appliedHoldMultiplier = holdMultiplier;
}

void TMCMotor::setRMSCurrentIRUN(uint16_t mA) {
    currentIRUN_mA = mA;
    // Update driver configuration. If IRUN is 0, multiplier is irrelevant.
    if (currentIRUN_mA > 0) {
        applyCurrent(currentIRUN_mA, (float)currentIHOLD_mA / currentIRUN_mA);
    } else {
        applyCurrent(0, 0.0f); // Set IRUN to 0, hold multiplier to 0
    }
}

uint16_t TMCMotor::getRMSCurrentIRUN() {
    return currentIRUN_mA;
}

void TMCMotor::setRMSCurrentIHOLD(uint16_t mA) {
    currentIHOLD_mA = mA;
    // Update driver configuration. If IRUN is 0, multiplier is irrelevant.
    float hold_multiplier = (currentIRUN_mA > 0) ? (float)currentIHOLD_mA / currentIRUN_mA : 0.0f;
    applyCurrent(currentIRUN_mA, hold_multiplier);
}

uint16_t TMCMotor::getRMSCurrentIHOLD() {
    return currentIHOLD_mA;
}

void TMCMotor::setMicrostepsTMC(int microsteps) {
    tmcDriver->mstep_reg_select(true);
    tmcDriver->microsteps(microsteps);
}

void TMCMotor::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert) {
    StepperMotor::setPinsInverted(directionInvert, stepInvert, enableInvert); // Call base class
}

void TMCMotor::enableOutputs() {
    digitalWrite(tmcEnablePin, LOW); // Active LOW for enabling most TMC drivers
    setStepOutputsEnabled(true);     // only now may the step interrupt pulse
    // Serial.println("TMCMotor: Enabled");
}

void TMCMotor::disableOutputs() {
    setStepOutputsEnabled(false);
    digitalWrite(tmcEnablePin, HIGH); // Active HIGH for disabling
    // Serial.println("TMCMotor: Disabled");
}

void TMCMotor::activateRunCurrent() {
    if (currentIRUN_mA > 0) {
        applyCurrent(currentIRUN_mA, (float)currentIHOLD_mA / currentIRUN_mA);
    } else {
        applyCurrent(0, 0.0f); // Or some safe default
    }
}

void TMCMotor::activateHoldCurrent() {
    // This sets the RUN current register (IRUN) to the value stored for IHOLD.
    // The actual hold current (CHOPCONF.ihold) will then be currentIHOLD_mA * its_own_multiplier (based on new IRUN).
    // This matches the original StepperTMC2209 pattern: x_driver.rms_current(rms_current_IX_HOLD);
    // The one-argument library call reuses the last hold multiplier.
    applyCurrent(currentIHOLD_mA, tmcDriver->hold_multiplier());
    // Serial.print("TMCMotor: Activated HOLD current (IRUN set to IHOLD value): "); Serial.println(currentIHOLD_mA);
}
