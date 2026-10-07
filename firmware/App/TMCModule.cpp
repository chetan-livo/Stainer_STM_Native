#include "Stm32SerialCompat.h"
/*
* TMCModule.cpp
*
*  Created on: June 7, 2025
*      Author: Varalakshmi
*/

#include "TMCModule.h"
#include "LoopStats.h"

#ifdef Master

#ifdef Stainer_Gantry_PCB
    TMCModule GXMotor(TMCMotorGXConfig, GX_Step, GX_Dir, GXZR_EN, GXZR_SERIAL_PORT, GX_DRIVER_ADDRESS);
    TMCModule GZMotor(TMCMotorGZConfig, GZ_Step, GZ_Dir, GXZR_EN, GXZR_SERIAL_PORT, GZ_DRIVER_ADDRESS);
    TMCModule GRMotor(TMCMotorGRConfig, GR_Step, GR_Dir, GXZR_EN, GXZR_SERIAL_PORT, GR_DRIVER_ADDRESS);
    TMCModule GYMotor(TMCMotorGYConfig, GY1_Step, GY1_Dir, GY_En, GY_SERIAL_PORT, GY_DRIVER_ADDRESS);
    // Extra Motor
    TMCModule XMotor(TMCMotorG2YConfig, GY2_Step, GY2_Dir, GY_En, GY_SERIAL_PORT, GY2_DRIVER_ADDRESS);
#endif

#ifdef Nozzle_Mount_PCB
    TMCModule SXMotor(TMCMotorSXConfig, SX_Step__, SX_Dir___, SXY_En_____, SXY_SERIAL_PORT, SX_DRIVER_ADDRESS);
    TMCModule SYMotor(TMCMotorSYConfig, SY_Step__, SY_Dir___, SXY_En_____, SXY_SERIAL_PORT, SY_DRIVER_ADDRESS);
    TMCModule WXMotor(TMCMotorWXConfig, WX_Step__, WX_Dir___, WXY_En_____, WXY_SERIAL_PORT, WX_DRIVER_ADDRESS);
    TMCModule WYMotor(TMCMotorWYConfig, WY_Step__, WY_Dir___, WXY_En_____, WXY_SERIAL_PORT, WY_DRIVER_ADDRESS);
    TMCModule BXMotor(TMCMotorBXConfig, BX_Step__, BX_Dir___, BXY_En_____, BXY_SERIAL_PORT, BX_DRIVER_ADDRESS);
    TMCModule BYMotor(TMCMotorBYConfig, BY_Step__, BY_Dir___, BXY_En_____, BXY_SERIAL_PORT, BY_DRIVER_ADDRESS);
#endif

#ifdef Stainer_Master_PCB
    TMCModule ZMotor(TMCMotorZConfig, Z_Step, Z_Dir, ZT_En, ZT_SERIAL_PORT, Z_DRIVER_ADDRESS);
    TMCModule TMotor(TMCMotorTConfig, T_Step, T_Dir, ZT_En, ZT_SERIAL_PORT, T_DRIVER_ADDRESS);
    TMCModule M14Motor(TMCMotorM14Config, M14_Step, M14_Dir, ZT_En, ZT_SERIAL_PORT, M14_DRIVER_ADDRESS);
    TMCModule M15Motor(TMCMotorM15Config, M15_Step, M15_Dir, ZT_En, ZT_SERIAL_PORT, M15_DRIVER_ADDRESS);
    TMCModule XMotor(TMCMotorXConfig, X_Step, X_Dir, XY_En, XY_SERIAL_PORT, X_DRIVER_ADDRESS);
    TMCModule YMotor(TMCMotorYConfig, Y_Step, Y_Dir, XY_En, XY_SERIAL_PORT, Y_DRIVER_ADDRESS);
    TMCModule M17Motor(TMCMotorM17Config, M17_Step, M17_Dir, XY_En, XY_SERIAL_PORT, M17_DRIVER_ADDRESS);
    TMCModule M18Motor(TMCMotorM18Config, M18_Step, M18_Dir, XY_En, XY_SERIAL_PORT, M18_DRIVER_ADDRESS);
#endif

TMCModule::TMCModule(const TMCConfig& config, uint8_t stepPin, uint8_t dirPin, uint8_t enPin, LivoHardwareSerial& serial, uint8_t driverAddress)
    : motor(stepPin, dirPin, enPin, serial, driverAddress, DEF_R_SENSE),
      stepPin(stepPin), dirPin(dirPin), enPin(enPin), driverAddress(driverAddress)
{
    microsteps = config.microsteps;
    speed = config.speed;
    acceleration = config.acceleration;
    rms_current_I_RUN = config.rms_current_I_RUN;
    rms_current_I_HOLD = config.rms_current_I_HOLD;
}

TMCModule::~TMCModule() {

}

void TMCModule::setup() {
	// Configure Motor Pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
	pinMode(enPin, OUTPUT); 

    Enabled = false;
    
    delay(50);

    motor.setupMotor();

	// Configure TMC Stepper Motor parameters
    motor.setRMSCurrentIRUN(rms_current_I_RUN);
    motor.setRMSCurrentIHOLD(rms_current_I_HOLD);
    motor.setMicrostepsTMC(microsteps);

	// Configure AccelStepper parameters
    motor.setMaxSpeed(static_cast<float>(speed));
    motor.setAcceleration(static_cast<float>(acceleration));
    motor.setPinsInverted(false, false, true);

    // STEP pulses from the TIM7 interrupt (StepEngine.h). Falls back to polled
    // AccelStepper stepping where the engine is unavailable.
    motor.enableInterruptStepping();
}

void TMCModule::loop() {
    if (motor.isRunning()) {
        // Polled motors only: in interrupt mode servicing no longer times steps.
        if (stepGapTracked && !motor.interruptStepping())
            LoopStats::onStepService(stepServiceCycles, stepServiceGeneration);
        if (!Enabled) {
            motor.enableOutputs();                        // instant digitalWrite — motor responds to steps immediately
            Enabled = true;
            motor.setRMSCurrentIRUN(rms_current_I_RUN);  // UART write after enable so steps aren't missed
        }
    } else {
        stepServiceGeneration = 0;
        if (Enabled) {
            Enabled = false;
            motor.setRMSCurrentIHOLD(rms_current_I_HOLD);
        }
    }
    motor.loopMotor();
}

void TMCModule::moveTo(long value) {
    motor.moveTo(value);
}

void TMCModule::move(long value) {
    motor.move(value);
}

void TMCModule::stop() {
	motor.stop();
}

void TMCModule::homing() {
    isHoming = true;
    long target_ = motor.currentPosition() + XY_HOMING_MOVE;
    motor.moveTo(target_);
}

long TMCModule::getDistanceToGo() {
    return motor.distanceToGo();
}

void TMCModule::setMaxSpeed(long value) {
	if (value != 0) {
		this->speed = value; // Update the stored base speed
		motor.setMaxSpeed(static_cast<float>(this->speed));
	}
}

long TMCModule::getMaxSpeed() const {
	// return static_cast<long>(motor.maxSpeed());
	return static_cast<long>(this->speed);
    // return static_cast<long>(motor.maxSpeed());
}

void TMCModule::setAcceleration(long value) {
    if (value > 0) {
        this->acceleration = value; // Update the stored base speed
        motor.setAcceleration(static_cast<float>(this->acceleration)); // Update AccelStepper via TMCMotor
    }
    // Serial.println("Acceleration set");
}

long TMCModule::getAcceleration() const {
	// return static_cast<long>(motor.acceleration());
	return static_cast<long>(this->acceleration);
}

void TMCModule::setCurrentPosition(long position) {
	motor.setCurrentPosition(position);
}

long TMCModule::getCurrentPosition() const {
	return motor.currentPosition();
}

bool TMCModule::isMoving() const {
	return motor.isRunning();
}

void TMCModule::enableMotor(bool enable) {
	if (enable) {
		motor.enableOutputs();
		Enabled = true;
	} else {
		motor.disableOutputs();
		Enabled = false;
	}
	markActionSuccess(true);
}

bool TMCModule::isMotorEnabled() const {
	return Enabled;
}

void TMCModule::setMicrosteps(int value) {
    if (value > 0) {
        microsteps = value;
        motor.setMicrostepsTMC(microsteps); // Assuming TMCMotor has this method
        motor.setMaxSpeed(static_cast<float>(speed));
        motor.setAcceleration(static_cast<float>(acceleration));
    }
}

int TMCModule::getMicrosteps() const {
	return microsteps;
}

void TMCModule::setRMSCurrentIRUN(long value) {
    if (value > 0) {
        rms_current_I_RUN = value;
        motor.setRMSCurrentIRUN(rms_current_I_RUN);
    }
}

long TMCModule::getRMSCurrentIRUN() const  {
	return rms_current_I_RUN;
}

void TMCModule::setRMSCurrentIHOLD(long value) {
    if (value >= 0) {
        rms_current_I_HOLD = value;
        motor.setRMSCurrentIHOLD(rms_current_I_HOLD);
    }
}

long TMCModule::getRMSCurrentIHOLD() const {
	return rms_current_I_HOLD;
}

long TMCModule::getOffset() const { // Added const
    return motor.getOffset();
}

void TMCModule::setOffset(long position) {
    motor.setOffset(position);
}

long TMCModule::getindexPulseAtStep() const {
	return indexPulseAtStep;
}

void TMCModule::stopAtIndexPulse(bool isAtIndexPulse){
	if(isHoming && !ignoreIndex){
		if(isAtIndexPulse){
            // Serial.println("isAtIndexPulse");
			if(indexPulseAtStep != motor.currentPosition()){
				indexPulseAtStep = motor.currentPosition();
                motor.setCurrentPosition(0);
				motor.moveTo(motor.currentPosition() + getHomingOffset());
                if(autoUpdateEncoder){
                    ishomed = true;
                    Serial.println("Homed");
                }
				isHoming = false;
			}
		}
	}
}

bool TMCModule::isHomed() const {
	return ishomed;
}

void TMCModule::markActionSuccess(bool status) {
    actionSuccessFlag = status;
}

bool TMCModule::wasLastActionSuccessful() const {
    return actionSuccessFlag;
}

long TMCModule::getHomingOffset() const {
	return motor.getHomingOffset();
}

void TMCModule::setHomingOffset(long offset) {
	motor.setHomingOffset(offset);
}

long TMCModule::getDefaultMaxSpeed() const {
    return DEF_TMC_MAXSPEED; // Return the static const MAXSPEED defined in TMCModule.h
}

long TMCModule::getDefaultMaxAcceleration() const {
    return DEF_TMC_MAXACCELERATION; // Return the static const MAXACCELERATION defined in TMCModule.h
}

#endif