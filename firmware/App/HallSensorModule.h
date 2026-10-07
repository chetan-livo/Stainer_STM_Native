/*
* HallSensorModule.h
*
*  Created on: May 17, 2025
*      Author: Varalakshmi
*/

#ifndef HALLSENSORMODULE_H_
#define HALLSENSORMODULE_H_

#include "Constants.h"

#ifdef Master
	#include "I2CInstance.h"
#else
	class I2CInstance;
#endif

class HallSensorModule {
public:
    HallSensorModule(uint8_t deviceID, uint8_t pcbID, int numberOfHalls, I2CInstance* i2cInstance);

    void requestHallPosition();
    void requestAllHallValues();
    void requestHallValues();
    void requestFilteredHallValues();
    void requestHalfHallValues(int halfvalue);
    void requestHalfFilteredHallValues(int halfvalue);
    void requestSlideStatus();
    void requestMagzineStatus();
    void lockMagzine();
    void unlockMagzine();
    
    void loop();

    long getPosition() const;
    const long* getHallValues() const;
    const int* getSlideStatus() const;
    uint8_t getSlideStatusState() const { return slideStatusState; }
    uint8_t getSlideMagazineType() const { return slideMagazineType; }
    uint8_t getCalibrationValidMask() const { return calibrationValidMask; }
    uint32_t getSlideStatusSequence() const { return slideStatusSequence; }
    bool isSlideStatusValid() const { return slideStatusState == 1; }

    const char*  getPCBLabel(uint8_t id);
    int getMagzineStatus() const;
    uint8_t getPcbID() const;
    
    uint32_t sensorSequence() const { return sensorSequence_; }
    bool sensorDataValid() const { return sensorDataValid_; }
    bool isHallRequestPending() const { return hallRequestFlag; }
    bool isSlideRequestPending() const { return slideRequestFlag; }
    bool isAnyRequestPending() const {
        return positionRequestFlag || hallRequestFlag || slideRequestFlag
            || magzineRequestFlag || lockMagzineFlag || unlockMagzineFlag;
    }
    void clearPendingRequests();

    // IR terminology for Magazine Holder callers. The legacy sensor API remains
    // for the actual Gantry Hall boards and wire-protocol compatibility.
    void requestAllIrValues() { requestAllHallValues(); }
    void requestIrValues() { requestHallValues(); }
    void requestFilteredIrValues() { requestFilteredHallValues(); }
    void requestHalfIrValues(int half) { requestHalfHallValues(half); }
    void requestHalfFilteredIrValues(int half) { requestHalfFilteredHallValues(half); }
    const long* getIrValues() const { return getHallValues(); }
    bool isIrRequestPending() const { return isHallRequestPending(); }
    void setSilent(bool silent) {
        silentMode = silent;
        suppressResponsePrint_ = silent;
    }

private:
    uint32_t sensorSequence_=0, sensorRequestAt_=0;
    bool sensorDataValid_=false;
    bool silentMode = false;
    bool suppressResponsePrint_ = false;
    I2CInstance* i2cPort; 
    uint8_t deviceID;
    uint8_t pcbID;
    int numberOfHalls;

    int RequestedHallNum;
    int hallStoreOffset = 0;

    // Internal flags
    bool positionRequestFlag = false;
    bool hallRequestFlag = false;
    bool fullHallRequestFlag = false;
    bool slideRequestFlag = false;
    bool magzineRequestFlag = false;
    bool lockMagzineFlag = false;
    bool unlockMagzineFlag = false;

    // Data
    long position = 0;
    long hallValues[30] = {0}; 
    int slideStatus[20] = {0};
    uint8_t slideStatusState = 0;
    uint8_t slideMagazineType = 0;
    uint8_t calibrationValidMask = 0;
    uint32_t slideStatusSequence = 0;
    int magzineStatus = 0;
    int lockStatus = 0;

};

// Magazine holders use reflective IR channels. This alias keeps the shared
// analog-I2C transport implementation without misidentifying the hardware.
using MagazineIrSensorModule = HallSensorModule;

#ifdef Master
    #ifdef Stainer_Gantry_PCB
        extern MagazineIrSensorModule magazine1IrModule;
        extern MagazineIrSensorModule magazine2IrModule;

        extern MagazineIrSensorModule& Magazine1Ir;
        extern MagazineIrSensorModule& Magazine2Ir;
    #endif
#endif

#endif /* HALLSENSORMODULE_H_ */
