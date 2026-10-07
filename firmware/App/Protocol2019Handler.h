/*
 * Protocol2019Handler.h
 *
 * Created on: Apr 25, 2024
 * Author: Varalakshmi
 */

#ifndef PROTOCOL2019HANDLER_H_
#define PROTOCOL2019HANDLER_H_

#include "Arduino.h"
#include "ByteArrayHandler.h"
#include "TMCModule.h"
#ifdef Master
    #include "I2CInstance.h"
    #include "HallSensorModule.h"
    #include "AccelerometerInstance.h"
#endif
#include "Execution2019Handler.h"
#include "DCMotor.h"
#include "DCFan.h"

class Protocol2019Handler
{
private:
    // General Commands
    long idValue;   // ID value
    long helpValue; // Help value
    long ackValue;  // Acknowledgment value
    long debValue;  // Debug value
    long magazineUartTestValue; // 1=H1, 2=H2, 3=both

    // Extra Motor
    long xValue;    // X Absolute move
    long xrValue;   // X relative move
    long xiValue;   // Set X Run Current
    long xmValue;   // Set X microstep 
    long xsValue;   // Set X Speed
    long xaValue;   // Set X acceleration
    long xgpValue;  // Get X position
    long xmlValue;  // X dispense in mL × 1000 (S2 pump)

    // TMC Stepper Motor Y
    long yValue;    // Y Absolute move
    long yrValue;   // Y relative move
    long yiValue;   // Set Y Run Current
    long ymValue;   // Set Y microstep
    long ysValue;   // Set Y Speed
    long yaValue;   // Set Y acceleration
    long ygpValue;  // Get Y position
    long ymlValue;  // Y dispense in mL × 1000 (B2 pump)

    long sxcasValue; // SX cascade trigger (master) — DCW1 + DCS1
    long s2mixValue; // S2 cascade trigger (master) — X + Y pumps + DCF2
    long b1mixValue; // B1 cascade trigger (master) — M15 (B1) pump + DCM1 + DCM2
    long wycasValue; // WY cascade trigger (master)

    // Stain profile selection — value semantics handled by parser
    //   0..3 = PROFILE_RP/LM/MG/WG, -1 = query (print current), -2 = invalid code
    long profileValue;
    // Profile-driven trigger commands. Nozzle sends these (e.g. REQ:DSPE) so
    // master fills the µL from the active profile rather than the nozzle.
    long dspeValue;   // E  (M14)
    long dsps1Value;  // S1 (Z)
    long dspb1Value;  // B1 (M15, runs B1MIX cascade with profile volume)
    long dsps2Value;  // S2 (X, runs S2MIX cascade with profile volume)
    long dspb2Value;  // B2 (Y, paired with S2 in S2MIX cascade)
    long mloadValue;   // Master pump-load (prime each liquid line until IR drops below threshold)
    long munloadValue; // Master unload (reverse all pumps + DCW1/DCW2 + DCD wash sequence)
    long drainValue;   // Master drain — DCD on, monitor IR8 after 3s, stop when IR8 < 100
    long mixmValue;    // Master MIXM (legacy DCF2 / PB15, deprecated by MIXD/MIXR H-bridge)
    long mixdValue;    // Master MIXD — MX1919 H-bridge dispense direction (IN1=PB14 PWM, IN2=PB15 LOW)
    long mixrValue;    // Master MIXR — MX1919 H-bridge reverse direction  (IN1=PB14 LOW,  IN2=PB15 PWM)
    // Legacy continuous pump start/stop pairs. Starts are disabled in execution;
    // STOP commands remain available as safety stops.
    long etdisValue;  long etstopValue;   // E  (M14)
    long s1disValue;  long s1stopValue;   // S1 (Z)
    long s2disValue;  long s2stopValue;   // S2 (X)
    long b1disValue;  long b1stopValue;   // B1 (M15)
    long b2disValue;  long b2stopValue;   // B2 (Y)

    // Gantry-X
    long gxValue;    // Gantry-X Absolute
    long gxrValue;   // Gantry-X Move Relative
    long gxiValue;   // Gantry-X Set Run Current
    long gxmValue;   // Gantry-X Set Microstepping
    long gxsValue;   // Gantry-X Set Speed
    long gxaValue;   // Gantry-X Set Acceleration
    long gxhoValue;  // Gantry-X Home command
    long gxgpValue;  // Gantry-X Get Current Position
     
    // Gantry-Y
    long gyValue;    // Gantry-Y Move Absolute
    long gyrValue;   // Gantry-Y Move Relative
    long gyiValue;   // Gantry-Y Set Run Current
    long gymValue;   // Gantry-Y Set Microstepping
    long gyhoValue;  // Gantry-Y Home via IR4
    long gysValue;   // Gantry-Y Set Speed
    long gyaValue;   // Gantry-Y Set Acceleration
    long gygpValue;  // Gantry-Y Get Current Position

    // Gantry-Z
    long gzValue;    // Gantry-Z Move Absolute
    long gzrValue;   // Gantry-Z Move Relative
    long gziValue;   // Gantry-Z Set Run Current
    long gzmValue;   // Gantry-Z Set Microstepping
    long gzhoValue;  // Gantry-Z Home command
    long gzsValue;   // Gantry-Z Set Speed
    long gzaValue;   // Gantry-Z Set Acceleration
    long gzgpValue;  // Gantry-Z Get Current Position
    long gzcalValue; // Gantry-Z Calibration
    long ldm1Value;  // Load Slide Command (1-20)

    // Gantry-R
    long grValue;    // Gantry-R Move Absolute
    long grrValue;   // Gantry-R Move Relative
    long griValue;   // Gantry-R Set Run Current
    long grmValue;   // Gantry-R Set Microstepping
    long grsValue;   // Gantry-R Set Speed
    long graValue;   // Gantry-R Set Acceleration
    long grhoValue;  // Gantry-R Home via IR3
    long grldValue;  // Gantry-R Load pattern command
    long grgpValue;  // Gantry-R Get Current Position

    long ghomeValue;    // Home all gantry axes (GXHO+GYHO+GZHO+GRHO)
    long nzhoValue;     // Home all nozzle mount axes (SY-/BX+/SX+/WX+/WY+)
    long sxhoValue;     // Home SX motor  (+, Lim5/DIN5/PD14)
    long syhoValue;     // Home SY motor  (-, Lim2/DIN2/PD13)
    long wxhoValue;     // Home WX motor  (+, Lim3/DIN3/PE7)
    long wyhoValue;     // Home WY motor  (+, Lim1/DIN1/PE3)
    long bxhoValue;     // Home BX motor  (+, Lim4/DIN4/PE8)
    long feedValue;     // Start T motor feed (FEED label; RPM = label/60)
    long feedstopValue; // Stop T motor feed
    long paraCheckValue; // PARA CHECK / PARA STOP: observe current-profile execution
    long bedtestValue;  // Bed test: FEED 180, measure steps IR4→IR6 and IR4→IR12
    long btresetValue;  // Bed test IR4 anchor → master resets TMotor.position to 0
    long gloadValue; // Move all gantry axes to load position
    long loadValue;  // Auto load from active or explicit magazine
    long rldValue;   // Read corrected lidar distance
    long rstValue;   // Reset/recovery sequence based on IR states

    // Stain X
    long sxValue;    // Stain X Absolute
    long sxrValue;   // Stain X Move Relative
    long sxiValue;   // Stain X Set Run Current
    long sxsValue;   // Stain X Set Speed
    long sxaValue;   // Stain X Set Acceleration
    long sxgpValue;  // Stain X Get Current Position

    // Stain Y
    long syValue;    // Stain Y Move Absolute
    long syrValue;   // Stain Y Move Relative
    long syiValue;   // Stain Y Set Run Current
    long sysValue;   // Stain Y Set Speed
    long syaValue;   // Stain Y Set Acceleration
    long sygpValue;  // Stain Y Get Current Position

    // Wash X
    long wxValue;    // Wash X Absolute
    long wxrValue;   // Wash X Move Relative
    long wxiValue;   // Wash X Set Run Current
    long wxsValue;   // Wash X Set Speed
    long wxaValue;   // Wash X Set Acceleration
    long wxgpValue;  // Wash X Get Current Position

    // Wash Y
    long wyValue;    // Wash Y Move Absolute
    long wyrValue;   // Wash Y Move Relative
    long wyiValue;   // Wash Y Set Run Current
    long wysValue;   // Wash Y Set Speed
    long wyaValue;   // Wash Y Set Acceleration
    long wygpValue;  // Wash Y Get Current Position

    // Buffer X
    long bxValue;    // Buffer X Absolute
    long bxrValue;   // Buffer X Move Relative
    long bxiValue;   // Buffer X Set Run Current
    long bxsValue;   // Buffer X Set Speed
    long bxaValue;   // Buffer X Set Acceleration
    long bxgpValue;  // Buffer X Get Current Position

    // Buffer Y
    long byValue;    // Buffer Y Move Absolute
    long byrValue;   // Buffer Y Move Relative
    long byiValue;   // Buffer Y Set Run Current
    long bysValue;   // Buffer Y Set Speed
    long byaValue;   // Buffer Y Set Acceleration
    long bygpValue;  // Buffer Y Get Current Position

    // Z Motor
    long zValue;     // Z Motor Move Absolute
    long zrValue;    // Z Motor Move Relative
    long ziValue;    // Z Motor Set Run Current
    long zmValue;    // Z Motor Set Microstepping
    long zsValue;    // Z Motor Set Speed
    long zaValue;    // Z Motor Set Acceleration
    long zgpValue;   // Z Motor Get Current Position
    long zmlValue;   // Z dispense in mL ×1000 (sent as long, divided by 1000 in handler)

    // T Motor
    long tValue;     // T Motor Move Absolute
    long trValue;    // T Motor Move Relative
    long tiValue;    // T Motor Set Run Current
    long tmValue;    // T Motor Set Microstepping
    long tsValue;    // T Motor Set Speed
    long taValue;    // T Motor Set Acceleration
    long tgpValue;   // T Motor Get Current Position

    // M14 Motor
    long m14Value, m14rValue, m14iValue, m14mValue, m14sValue, m14aValue, m14gpValue, m14mlValue;
    // M15 Motor
    long m15Value, m15rValue, m15iValue, m15mValue, m15sValue, m15aValue, m15gpValue, m15mlValue;
    // M17 Motor
    long m17Value, m17rValue, m17iValue, m17mValue, m17sValue, m17aValue, m17gpValue;
    // M18 Motor
    long m18Value, m18rValue, m18iValue, m18mValue, m18sValue, m18aValue, m18gpValue;

    // Common
    long giValue;    // Get Run Currents
    long gmValue;    // Get microsteppings
    long gsValue;    // Get speeds
    long gaValue;    // Get accelerations

    long hValue;    // Homing 

    long hsValue;   // Hall Sensor PCBs
 
    long smcValue;  // Start Magazine Checks
    long cal1Value; // Calibrate Magazine 1 thresholds
    long cal2Value; // Calibrate Magazine 2 thresholds

    // Read IR sensor values
    long rirValue;      // Read IR sensor
    long rdhtValue;     // Read Nozzle Mount DHT11 on IR9
    long pirValue;      // Print IR sensor

    // Read Limit Switch values
    long rlsValue;      // Read LimitSwitch
    long plsValue;      // Print LimitSwitch

    // Hall Sensor Modules
    long gxhValue;      // Gantry X Hall
    long gxhiValue;     // Gantry X Hall Individual (1-10)
    long mhValue;       // Magazine Holder IR sensors (legacy MH command name)
    long mh2SampleCountValue; // Magazine 2 repeated full-value read count
    long mhp2SampleCountValue; // Magazine 2 repeated full-value read count with PE5 trigger per sample
    long gzhValue;      // Gantry Z Hall
    long gzhiValue;     // Gantry Z Hall Individual (1-10)

    long m1lValue;      // Magazine 1 Lock solenoid
    long m1uValue;      // Magazine 1 Unlock solenoid
    long m2lValue;      // Magazine 2 Lock solenoid
    long m2uValue;      // Magazine 2 Unlock solenoid

    // Read Accelerometer
    long ramValue;     // Accelerometer
    long pamValue;     // Print Accelerometer

    // DC Motor 
    long dcmValue;      // DC Motor
    long dcm1Value;     // DC Mix Motor1
    long dcm2Value;     // DC Mix Motor2
    long dcdValue;      // DC Drain Motor
    long dcw1Value;     // DC Wash Motor1
    long dcw2Value;     // DC Wash Motor2
    long dcs1Value;     // DC Suction Motor1
    long dcs2Value;     // DC Suction Motor2

    // DC Fan
    long dcf1Value;     // DC Fan1
    long dcf2Value;     // DC Fan2
    long dcf3Value;     // DC Fan3
    long dcf4Value;     // DC Fan4
    long dcfValue;      // Get DC Fan Speed

    // Helper method to extract value from key-value pair
    long extractValue(String pair);

public:
    long ignoreValue = 123456789; // Default value to ignore if not set

    // Constructor and Destructor
    Protocol2019Handler();
    virtual ~Protocol2019Handler();

    // Parse the input frame packet
    bool parsePacket(ByteArrayHandler &packet);

    // General Commands
    long getIDValue() const { return idValue; }
    long getHelpValue() const { return helpValue; }
    long getAckValue() const { return ackValue; }
    long getDebValue() const { return debValue; }
    long getMagazineUartTestValue() const { return magazineUartTestValue; }

    // TMC Stepper Motor X
    long getXValue() const { return xValue; }
    long getXRValue() const { return xrValue; }
    long getXIValue() const { return xiValue; }
    long getXMValue() const { return xmValue; }
    long getXSValue() const { return xsValue; }
    long getXAValue() const { return xaValue; }
    long getXGPValue() const { return xgpValue; }
    long getXMLValue() const { return xmlValue; }

    // TMC Stepper Motor Y
    long getYValue() const { return yValue; }
    long getYRValue() const { return yrValue; }
    long getYIValue() const { return yiValue; }
    long getYMValue() const { return ymValue; }
    long getYSValue() const { return ysValue; }
    long getYAValue() const { return yaValue; }
    long getYGPValue() const { return ygpValue; }
    long getYMLValue() const { return ymlValue; }
    long getSXCASValue() const { return sxcasValue; }
    long getS2MIXValue() const { return s2mixValue; }
    long getB1MIXValue() const { return b1mixValue; }
    long getWYCASValue() const { return wycasValue; }
    long getPROFILEValue() const { return profileValue; }
    long getDSPEValue()   const { return dspeValue;   }
    long getDSPS1Value()  const { return dsps1Value;  }
    long getDSPB1Value()  const { return dspb1Value;  }
    long getDSPS2Value()  const { return dsps2Value;  }
    long getDSPB2Value()  const { return dspb2Value;  }
    long getMLOADValue() const { return mloadValue; }
    long getMUNLOADValue() const { return munloadValue; }
    long getDRAINValue() const { return drainValue; }
    long getMIXMValue()  const { return mixmValue; }
    long getMIXDValue()  const { return mixdValue; }
    long getMIXRValue()  const { return mixrValue; }
    long getETDISValue()  const { return etdisValue;  }
    long getETSTOPValue() const { return etstopValue; }
    long getS1DISValue()  const { return s1disValue;  }
    long getS1STOPValue() const { return s1stopValue; }
    long getS2DISValue()  const { return s2disValue;  }
    long getS2STOPValue() const { return s2stopValue; }
    long getB1DISValue()  const { return b1disValue;  }
    long getB1STOPValue() const { return b1stopValue; }
    long getB2DISValue()  const { return b2disValue;  }
    long getB2STOPValue() const { return b2stopValue; }

    // Gantry-X 
    long getGXValue() const { return gxValue; }
    long getGXRValue() const { return gxrValue; }
    long getGXIValue() const { return gxiValue; }
    long getGXMValue() const { return gxmValue; }
    long getGXSValue() const { return gxsValue; }
    long getGXAValue() const { return gxaValue; }
    long getGXHOValue() const { return gxhoValue; }
    long getGXGPValue() const { return gxgpValue; }

    // Gantry-Y
    long getGYValue() const { return gyValue; }
    long getGYRValue() const { return gyrValue; }
    long getGYIValue() const { return gyiValue; }
    long getGYMValue() const { return gymValue; }
    long getGYHOValue() const { return gyhoValue; }
    long getGYSValue() const { return gysValue; }
    long getGYAValue() const { return gyaValue; }
    long getGYGPValue() const { return gygpValue; }

    // Gantry-Z
    long getGZValue() const { return gzValue; }
    long getGZRValue() const { return gzrValue; }
    long getGZIValue() const { return gziValue; }
    long getGZMValue() const { return gzmValue; }
    long getGZHOValue() const { return gzhoValue; }
    long getGZSValue() const { return gzsValue; }
    long getGZAValue() const { return gzaValue; }
    long getGZGPValue() const { return gzgpValue; }
    long getGZCALValue() const { return gzcalValue; }
    long getLDM1Value() const { return ldm1Value; }

    // Gantry-R
    long getGRValue() const { return grValue; }
    long getGRRValue() const { return grrValue; }
    long getGRIValue() const { return griValue; }
    long getGRMValue() const { return grmValue; }
    long getGRSValue() const { return grsValue; }
    long getGRAValue() const { return graValue; }
    long getGRHOValue() const { return grhoValue; }    long getGRLDValue() const { return grldValue; }    long getGRGPValue() const { return grgpValue; }

    long getGHOMEValue()    const { return ghomeValue; }
    long getNZHOValue()     const { return nzhoValue; }
    long getSXHOValue()     const { return sxhoValue; }
    long getSYHOValue()     const { return syhoValue; }
    long getWXHOValue()     const { return wxhoValue; }
    long getWYHOValue()     const { return wyhoValue; }
    long getBXHOValue()     const { return bxhoValue; }
    long getFEEDValue()     const { return feedValue; }
    long getFEEDSTOPValue() const { return feedstopValue; }
    long getBEDTESTValue()  const { return bedtestValue; }
    long getParaCheckValue() const { return paraCheckValue; }
    long getBTRESETValue()  const { return btresetValue; }
    long getGLOADValue() const { return gloadValue; }
    long getLOADValue() const { return loadValue; }
    long getRLDValue() const { return rldValue; }
    long getRSTValue() const { return rstValue; }

    // Stain X
    long getSXValue() const { return sxValue; }
    long getSXRValue() const { return sxrValue; }
    long getSXIValue() const { return sxiValue; }
    long getSXSValue() const { return sxsValue; }
    long getSXAValue() const { return sxaValue; }
    long getSXGPValue() const { return sxgpValue; }

    // Stain Y
    long getSYValue() const { return syValue; }
    long getSYRValue() const { return syrValue; }
    long getSYIValue() const { return syiValue; }
    long getSYSValue() const { return sysValue; }
    long getSYAValue() const { return syaValue; }
    long getSYGPValue() const { return sygpValue; }

    // Wash X
    long getWXValue() const { return wxValue; }
    long getWXRValue() const { return wxrValue; }
    long getWXIValue() const { return wxiValue; }
    long getWXSValue() const { return wxsValue; }
    long getWXAValue() const { return wxaValue; }
    long getWXGPValue() const { return wxgpValue; }

    // Wash Y
    long getWYValue() const { return wyValue; }
    long getWYRValue() const { return wyrValue; }
    long getWYIValue() const { return wyiValue; }
    long getWYSValue() const { return wysValue; }
    long getWYAValue() const { return wyaValue; }
    long getWYGPValue() const { return wygpValue; }

    // Buffer X
    long getBXValue() const { return bxValue; }
    long getBXRValue() const { return bxrValue; }
    long getBXIValue() const { return bxiValue; }
    long getBXSValue() const { return bxsValue; }
    long getBXAValue() const { return bxaValue; }
    long getBXGPValue() const { return bxgpValue; }

    // Buffer Y
    long getBYValue() const { return byValue; }
    long getBYRValue() const { return byrValue; }
    long getBYIValue() const { return byiValue; }
    long getBYSValue() const { return bysValue; }
    long getBYAValue() const { return byaValue; }
    long getBYGPValue() const { return bygpValue; }

    // Z Motor
    long getZValue() const { return zValue; }
    long getZRValue() const { return zrValue; }
    long getZIValue() const { return ziValue; }
    long getZMValue() const { return zmValue; }
    long getZSValue() const { return zsValue; }
    long getZAValue() const { return zaValue; }
    long getZGPValue() const { return zgpValue; }
    long getZMLValue() const { return zmlValue; }

    // T Motor
    long getTValue() const { return tValue; }
    long getTRValue() const { return trValue; }
    long getTIValue() const { return tiValue; }
    long getTMValue() const { return tmValue; }
    long getTSValue() const { return tsValue; }
    long getTAValue() const { return taValue; }
    long getTGPValue() const { return tgpValue; }

    // M14 / M15 / M17 / M18 Motors
    long getM14Value()   const { return m14Value;   }
    long getM14RValue()  const { return m14rValue;  }
    long getM14IValue()  const { return m14iValue;  }
    long getM14MValue()  const { return m14mValue;  }
    long getM14SValue()  const { return m14sValue;  }
    long getM14AValue()  const { return m14aValue;  }
    long getM14GPValue() const { return m14gpValue; }
    long getM14MLValue() const { return m14mlValue; }

    long getM15Value()   const { return m15Value;   }
    long getM15RValue()  const { return m15rValue;  }
    long getM15IValue()  const { return m15iValue;  }
    long getM15MValue()  const { return m15mValue;  }
    long getM15SValue()  const { return m15sValue;  }
    long getM15AValue()  const { return m15aValue;  }
    long getM15GPValue() const { return m15gpValue; }
    long getM15MLValue() const { return m15mlValue; }

    long getM17Value()   const { return m17Value;   }
    long getM17RValue()  const { return m17rValue;  }
    long getM17IValue()  const { return m17iValue;  }
    long getM17MValue()  const { return m17mValue;  }
    long getM17SValue()  const { return m17sValue;  }
    long getM17AValue()  const { return m17aValue;  }
    long getM17GPValue() const { return m17gpValue; }

    long getM18Value()   const { return m18Value;   }
    long getM18RValue()  const { return m18rValue;  }
    long getM18IValue()  const { return m18iValue;  }
    long getM18MValue()  const { return m18mValue;  }
    long getM18SValue()  const { return m18sValue;  }
    long getM18AValue()  const { return m18aValue;  }
    long getM18GPValue() const { return m18gpValue; }

    // Common
    long getGIValue() const { return giValue; }
    long getGMValue() const { return gmValue; }
    long getGSValue() const { return gsValue; }
    long getGAValue() const { return gaValue; }

    long getHValue() const { return hValue; }

    long getHSValue() const { return hsValue; }

    long getSMCValue() const { return smcValue;}
    long getCAL1Value() const { return cal1Value; }
    long getCAL2Value() const { return cal2Value; }

    // Read IR sensor values
    long getRIRValue() const { return rirValue; }
    long getRDHTValue() const { return rdhtValue; }
    long getPIRValue() const { return pirValue; }

    // Read Limit Switch values
    long getRLSValue() const { return rlsValue; }
    long getPLSValue() const { return plsValue; }

    // Hall Sensor Module
    long getGXHValue() const { return gxhValue; }
    long getGXHIValue() const { return gxhiValue; }
    long getMHValue() const { return mhValue; }
    long getMH2SampleCountValue() const { return mh2SampleCountValue; }
    long getMHP2SampleCountValue() const { return mhp2SampleCountValue; }
    long getGZHValue() const { return gzhValue; }
    long getGZHIValue() const { return gzhiValue; }

    // Read Accelerometer
    long getRAMValue() const { return ramValue; }
    long getPAMValue() const { return pamValue; }

    // DC Motor
    long getDCMValue() const { return dcmValue; }
    long getDCM1Value() const { return dcm1Value; }
    long getDCM2Value() const { return dcm2Value; }
    long getDCDValue() const { return dcdValue; }
    long getDCW1Value() const { return dcw1Value; }
    long getDCW2Value() const { return dcw2Value; }
    long getDCS1Value() const { return dcs1Value; }
    long getDCS2Value() const { return dcs2Value; }

    // DC Fan
    long getDCF1Value() const { return dcf1Value; }
    long getDCF2Value() const { return dcf2Value; }
    long getDCF3Value() const { return dcf3Value; }
    long getDCF4Value() const { return dcf4Value; }
    long getDCFValue() const { return dcfValue; }

    long getM1LValue() const { return m1lValue; }
    long getM1UValue() const { return m1uValue; }
    long getM2LValue() const { return m2lValue; }
    long getM2UValue() const { return m2uValue; }

    // Method to reset the parsed values
    void resetValues();

    // Method to check if a specific key exists in the packet
    bool hasKey(ByteArrayHandler &packet, const String &key);
    void process2019Packet();
    void displayPacketValues();
    void performPacketActions();
};

extern Protocol2019Handler protocol2019Handler;

#endif /* PROTOCOL2019HANDLER_H_ */
