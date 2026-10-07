#include "ServiceDiagnostics.h"
#include "DiagnosticFrame.h"
#include "Constants.h"
#include "TMCModule.h"
#include "LimitSwitch.h"
#include "SensorFaultPolicy.h"
#include "DCMotor.h"
#include "AccelerometerInstance.h"
#include "LidarReader.h"
#include "NozzleDHT.h"
#include "DCFan.h"

// The service transport is deliberately separate from the production parser.
// No arbitrary remote command execution or automatic homing from unknown coordinates.
bool serviceBackgroundCommand(const String& command) {
    return command == "ID" || command == "RDHT" || command == "HELP" ||
           command.startsWith("MAGID ") || command.startsWith("MAGFWSTAT");
}
namespace ServiceDiagnostics {
#ifdef Master
static bool owned=false;
static uint32_t heartbeat=0;
static String session;
static unsigned index=0;
static bool running=false;
static uint8_t motion=0;
static long origin=0,savedSpeed=0,savedAcceleration=0;
static uint32_t motionStarted=0;
static unsigned outputIndex=0;
static bool outputOn=false;
static uint32_t outputStarted=0;
static TMCModule* motors[]={
#if defined(Stainer_Master_PCB)
    &XMotor,&YMotor,&ZMotor,&TMotor,&M14Motor,&M15Motor,&M17Motor,&M18Motor
#elif defined(Stainer_Gantry_PCB)
    &GXMotor,&GYMotor,&GZMotor,&GRMotor,&XMotor
#elif defined(Nozzle_Mount_PCB)
    &SXMotor,&SYMotor,&WXMotor,&WYMotor,&BXMotor,&BYMotor
#endif
};
static const char* motorNames[]={
#if defined(Stainer_Master_PCB)
    "X","Y","Z","T","M14","M15","M17","M18"
#elif defined(Stainer_Gantry_PCB)
    "GX","GY","GZ","GR","GY2"
#elif defined(Nozzle_Mount_PCB)
    "SX","SY","WX","WY","BX","BY"
#endif
};
static unsigned rowSequence=0;
static void frame(const String& body) {
    char checksum[9];snprintf(checksum,sizeof(checksum),"%08lx",(unsigned long)DiagnosticFrame::crc(body.c_str(),body.length()));
    mirroredSerial.println(body+" CRC="+checksum);
    // Bound bursts through the Master's relay. All test outputs are stopped
    // before sensor frames are emitted; this delay never extends a pump pulse.
    mirroredSerial._b.flush();delay(25);
}
static void row(const char* code,const char* status,const String& evidence) {
    String body=String("DROW ")+session+" "+code+" "+status+" BOARD="+String(DEVICE_ID)+" "+evidence+" SEQ="+String(rowSequence++);
    frame(body);
}
void result(const char* code,const char* status,const String& evidence){row(code,status,evidence);}
static void stop() {
    for(auto motor:motors) motor->setCurrentPosition(motor->getCurrentPosition());
    if(motion && index<sizeof(motors)/sizeof(motors[0])) {
        motors[index]->setMaxSpeed(savedSpeed);motors[index]->setAcceleration(savedAcceleration);
    }
    motion=0;
    outputOn=false;
#ifdef Stainer_Master_PCB
    mixdc1.stopMotor();mixdc2.stopMotor();washdc1.stopMotor();washdc2.stopMotor();
    suctiondc1.stopMotor();suctiondc2.stopMotor();digitalWrite(D_Motor,LOW);dcFan2.stopFan();
#endif
#ifdef Nozzle_Mount_PCB
    dcMotor.stopMotor();dcFan1.stopFan();dcFan2.stopFan();dcFan3.stopHighSpeedFan();dcFan4.stopHighSpeedFan();
#endif
    running=false;
}
bool locked(){return owned;}
bool consume(const String& command) {
    if(owned && (command=="FEEDSTOP" || command=="ETSTOP" || command=="S1STOP" || command=="S2STOP" || command=="B1STOP" || command=="B2STOP")) {stop();return true;}
    if(command=="DLOCK 1") {
        if (!owned && serviceProductionBusy()) {
            mirroredSerial.println("DBUSY PRODUCTION_ACTIVE"); return true;
        }
        if(!owned){stop();serviceDiagnosticQuiesce();}
        owned=true;heartbeat=millis();mirroredSerial.print("DLOCKED BOARD=");mirroredSerial.println(DEVICE_ID);return true;
    }
    // Stale remote service cleanup must not stop production motors or pumps.
    if(command=="DLOCK 0" || command=="DSTOP") {
        if (owned) {
            stop();
            if(command=="DLOCK 0"){owned=false;serviceDiagnosticRelease();}
        }
        return true;
    }
    if(command.startsWith("DTEST ")) {
        String id=command.substring(6);
        if(!owned || id.length()!=8 || running || id==session)return true;
        for(unsigned i=0;i<id.length();++i)if(!isxdigit(id[i]))return true;
        session=id; rowSequence=0;index=0;motion=0;outputIndex=0;outputOn=false;running=true;return true;
    }
    if(owned && command!="ID" && command!="MAGID 1" && command!="MAGID 2")return true;
    return false;
}
void loop() {
    if(!owned)return;
    if(uint32_t(millis()-heartbeat)>10000){stop();return;} // Remain locked on lost ESP link.
    if(!running)return;
    const unsigned count=sizeof(motors)/sizeof(motors[0]);
    if(index<count) {
        auto motor=motors[index];uint32_t raw=0;
        if(motion) {
            bool limit=false;
            for(unsigned i=0;i<sizeof(limitPins)/sizeof(limitPins[0]);++i)if(digitalRead(limitPins[i])==LOW)limit=true;
            if(limit || uint32_t(millis()-motionStarted)>3000) {
                motor->setCurrentPosition(motor->getCurrentPosition());
                row("LIVO-SEN-023","FAIL",String("MOTOR=")+motorNames[index]+" bounded_exercise_interrupted_limit_or_timeout");
                motor->setMaxSpeed(savedSpeed);motor->setAcceleration(savedAcceleration);motion=0;++index;return;
            }
            motor->loop();
            if(motor->isMoving())return;
            if(motion==1){motor->moveTo(origin);motion=2;return;}
            motor->setMaxSpeed(savedSpeed);motor->setAcceleration(savedAcceleration);motion=0;
            row("LIVO-SEN-023","NOT_TESTED",String("MOTOR=")+motorNames[index]+" exercised_16_steps_and_returned_commanded_origin_no_shaft_feedback");
            ++index;return;
        }
        bool linked=motor->motor.diagnosticStatus(raw);
        String evidence=String("MOTOR=")+motorNames[index]+" DRV_STATUS="+String(raw,HEX);
        bool optional=false;
#if defined(Stainer_Master_PCB)
        optional=index==6; // M17 is an extra port, not an assumed fitted motor.
#elif defined(Stainer_Gantry_PCB)
        optional=index==4;
#elif defined(Nozzle_Mount_PCB)
        optional=index==5;
#endif
        row("LIVO-COM-005",linked?"PASS":optional?"NOT_TESTED":"FAIL",evidence+(optional?" optional_port_presence_unconfigured":" driver_register_link_only"));
        if(linked) {
            row("LIVO-SEN-017",raw&2?"FAIL":"PASS",evidence+" overtemperature_shutdown_bit");
            row("LIVO-SEN-018",raw&0x3c?"FAIL":"PASS",evidence+" short_circuit_bits");
            if(raw&0x3e){stop();return;} // Do not exercise more outputs after a critical driver indication.
        }
        bool limit=false;
        for(unsigned i=0;i<sizeof(limitPins)/sizeof(limitPins[0]);++i)if(digitalRead(limitPins[i])==LOW)limit=true;
        if(linked && !(raw&0x3e) && !limit) {
            origin=motor->getCurrentPosition();savedSpeed=motor->getMaxSpeed();savedAcceleration=motor->getAcceleration();
            motor->setMaxSpeed(128);motor->setAcceleration(256);motor->move(16);motion=1;motionStarted=millis();return;
        }
        row("LIVO-SEN-023","NOT_TESTED",evidence+" exercise_blocked_driver_or_active_limit");
        ++index;return;
    }
#ifdef Stainer_Master_PCB
    // Each DC output is bounded independently; pump delivery has no flow feedback.
    DCMotor* outputs[]={&mixdc1,&mixdc2,&washdc1,&washdc2,&suctiondc1,&suctiondc2};
    const char* names[]={"DCM1","DCM2","DCW1","DCW2","DCS1","DCS2","DCD","DCF2"};
    if(outputIndex<8) {
        if(!outputOn) {
            if(outputIndex<6)outputs[outputIndex]->runMotor(64);
            else if(outputIndex==6)digitalWrite(D_Motor,HIGH);else dcFan2.runFan(64);
            outputOn=true;outputStarted=millis();return;
        }
        if(millis()-outputStarted<200)return;
        if(outputIndex<6)outputs[outputIndex]->stopMotor();
        else if(outputIndex==6)digitalWrite(D_Motor,LOW);else dcFan2.stopFan();
        row("LIVO-SEN-002","NOT_TESTED",String("MOTOR=")+names[outputIndex]+" exercised_200ms_no_rotation_or_flow_feedback");
        outputOn=false;++outputIndex;return;
    }
#endif
#ifdef Nozzle_Mount_PCB
    if(outputIndex<5) {
        if(!outputOn) {
            if(outputIndex==0)dcMotor.runMotor(64);
            else if(outputIndex==1)dcFan1.runFan(64);else if(outputIndex==2)dcFan2.runFan(64);
            else {auto& fan=outputIndex==3?dcFan3:dcFan4;fan.getHighSpeedFanSpeed();fan.runHighSpeedFanPercent(25);}
            outputOn=true;outputStarted=millis();return;
        }
        if(millis()-outputStarted<(outputIndex>=3?1500UL:200UL))return;
        const char* names[]={"M1_PWM","DCF1","DCF2","DCF3","DCF4"};
        const char* result="NOT_TESTED";String evidence=String("MOTOR=")+names[outputIndex]+" exercised_no_rotation_feedback";
        if(outputIndex==0)dcMotor.stopMotor();else if(outputIndex==1)dcFan1.stopFan();else if(outputIndex==2)dcFan2.stopFan();
        else {auto& fan=outputIndex==3?dcFan3:dcFan4;float rpm=fan.getHighSpeedFanSpeed();fan.stopHighSpeedFan();result=rpm>0?"PASS":"FAIL";evidence=String("FAN=DCF")+String(outputIndex)+" TACH="+String(rpm)+" tach_pulses_only";}
        row(outputIndex?"LIVO-SEN-020":"LIVO-SEN-002",result,evidence);
        outputOn=false;++outputIndex;return;
    }
#endif
    // Report every configured analog channel individually. A valid ADC domain is
    // not proof that a sensor changes correctly under a physical stimulus.
    int* values=limitSwitch.getSmoothedSensorValues();
    for(unsigned i=0;i<sizeof(irsensorPins)/sizeof(irsensorPins[0]);++i) {
#ifdef Nozzle_Mount_PCB
        if(i==8)continue; // IR9 is the digital DHT pin, not an ADC sensor.
#endif
        int raw=values?values[i]:-1;
        String evidence=String("SENSOR=IR")+String(i+1)+" RAW="+String(raw);
        row("LIVO-SEN-021",SensorFaultPolicy::adcValid(raw)?"PASS":"FAIL",evidence+" ADC_domain_only");
    }
    for(unsigned i=0;i<sizeof(limitPins)/sizeof(limitPins[0]);++i)
        row("LIVO-SEN-023","NOT_TESTED",String("SENSOR=LIMIT")+String(i+1)+" RAW="+String(digitalRead(limitPins[i]))+" static_digital_sample_only");
#if defined(Stainer_Master_PCB) || defined(Stainer_Gantry_PCB)
    int16_t sample[3];bool accel=acc1.readAccelSample(sample);
    row("LIVO-SEN-006",accel?"PASS":"FAIL","SENSOR=ACC1 fresh_I2C_transaction_only");
#endif
#ifdef Stainer_Gantry_PCB
    float distance=0;bool lidar=lidarReader.readCorrectedDistanceMm(distance,1);
    row("LIVO-SEN-003",lidar?"PASS":"FAIL",String("SENSOR=LIDAR MM=")+String(distance));
#if !MAGAZINE_UART_TEST
    bool accel2=acc2.readAccelSample(sample);
    row("LIVO-SEN-006",accel2?"PASS":"FAIL","SENSOR=ACC2 fresh_I2C_transaction_only");
#endif
#endif
#ifdef Nozzle_Mount_PCB
    bool dht=diagnosticNozzleDHT();
    row(diagnosticNozzleDHTCode(),dht?"PASS":"FAIL","SENSOR=DHT11 recent_checksum_and_range_validity");
#endif
    serviceDiagnosticPeripherals();
    frame(String("DDONE ")+session+" BOARD="+String(DEVICE_ID)+" COUNT="+String(rowSequence));
    running=false;
}
#else
bool locked(){return false;}
bool consume(const String&){return false;}
void loop(){}
void result(const char*,const char*,const String&){}
#endif
}
