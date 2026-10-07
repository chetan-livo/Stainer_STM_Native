// Included only in the Master section of Execution2019Handler.cpp.
static uint32_t masterBedEpoch=0,masterBedSentAt=0,masterBedDigest=0;
static uint32_t masterBedSentCycles=0; // LOOPSTAT barrier latency
static uint32_t masterBedFaultReason=0;
static bool masterBedWaiting=false,masterBedEstablished=false;
static uint32_t masterBedSeen[BedSteps::ActionCount]={};
static BedSteps::Frame masterBedPending[BedSteps::ActionCount];
static unsigned masterBedPendingCount=0;
// Reference counts retain overlapping per-slide ON/OFF windows.
static int masterBedWindows[5]={};
static uint32_t masterBedS2Sequence=0;
static int masterBedWindow(uint16_t action,int& delta) {
    using namespace BedSteps;
    switch(action) {
    case AirOn:delta=1;return 0; case AirOff:delta=-1;return 0;
    case Wash1On:delta=1;return 1; case Wash1Off:delta=-1;return 1;
    case Suction1On:delta=1;return 2; case Suction1Off:delta=-1;return 2;
    case Wash2On:delta=1;return 3; case Wash2Off:delta=-1;return 3;
    case Suction2On:delta=1;return 4; case Suction2Off:delta=-1;return 4;
    default:delta=0;return -1;
    }
}
static bool masterBedWindowsActive() {
    for(int count:masterBedWindows)if(count)return true;
    return false;
}
static void masterBedApplyWindows() {
    // EXPERIMENT S2-DCM2 rollback reference: original shared air-window outputs.
    // if(masterBedWindows[0]&&!reagentDispenseLocked){mixdc1.runMotor(DCM1_DUTY);mixdc2.runMotor(DCM2_DUTY);}
    // else {mixdc1.stopMotor();mixdc2.stopMotor();}
    if(masterBedWindows[0]&&!reagentDispenseLocked)mixdc1.runMotor(DCM1_DUTY);
    else mixdc1.stopMotor();
    const bool s2OwnsDCM2=S2_DCM2_EXPERIMENT_ENABLED &&
        (activeProfileId==PROFILE_MG || activeProfileId==PROFILE_WG);
    if(s2OwnsDCM2) {
        // S2Start arms this state; S2Dispatch closes it at its absolute position.
        // Existing FEEDSTOP/fault/reset paths stop DCM2; resume reapplies this window.
        // Previous experiment condition (immediate ON), retained for rollback:
        // if(!reagentDispenseLocked && s2MixState==S2MIX_ACTIVE && !s2MixDispatchStarted)
        // Delay follows bed travel only; a pause cannot consume the remaining steps.
        if(!reagentDispenseLocked && s2MixState==S2MIX_ACTIVE && !s2MixDispatchStarted &&
           BedSteps::difference(masterBedPosition,(uint32_t)s2MixStartStep)>=
               (int32_t)scaleTMotorSteps(S2_DCM2_EXPERIMENT_DELAY_STEPS))
            mixdc2.runMotor(S2_DCM2_EXPERIMENT_VALUE);
        else mixdc2.stopMotor();
    } else {
        if(masterBedWindows[0]&&!reagentDispenseLocked)mixdc2.runMotor(DCM2_DUTY);
        else mixdc2.stopMotor();
    }
    if(masterBedWindows[1])washdc1.runMotor(DCW1_DUTY);else washdc1.stopMotor();
    if(masterBedWindows[2])suctiondc1.runMotor(DCS1_DUTY);else suctiondc1.stopMotor();
    if(masterBedWindows[3])washdc2.runMotor(DCW2_DUTY);else washdc2.stopMotor();
    if(masterBedWindows[4])suctiondc2.runMotor(DCS2_DUTY);else suctiondc2.stopMotor();
}
static void masterBedFail(uint32_t reason) {
    if(masterBedFault)return;
    masterBedFault=true;masterBedFaultReason=reason;tFeedActive=false;masterBedWaiting=false;
    for(auto& count:masterBedWindows)count=0;masterBedS2Sequence=0;
    TMotor.setCurrentPosition(TMotor.getCurrentPosition());
    for(TMCModule* m:{&M14Motor,&ZMotor,&M15Motor,&XMotor,&YMotor,&M18Motor})m->setCurrentPosition(m->getCurrentPosition());
    washdc1.stopMotor();washdc2.stopMotor();suctiondc1.stopMotor();suctiondc2.stopMotor();mixdc1.stopMotor();mixdc2.stopMotor();
    m15MixState=M15MIX_IDLE;sxCasState=SXCAS_IDLE;wyCasState=WYCAS_IDLE;s2MixState=S2MIX_IDLE;m18OscState=M18_OSC_IDLE;
    BedSteps::send(NozzleSerial,'F',masterBedEpoch,masterBedPosition,reason);
    Serial.print("[BED] FAULT ");Serial.print(reason);Serial.print(" at step=");Serial.println(masterBedPosition);
}
static void masterBedSend(char kind) {
    masterBedPendingCount=0;masterBedDigest=0;masterBedWaiting=true;masterBedSentAt=millis();
#ifdef LIVO_LOOPSTATS
    masterBedSentCycles=LoopStats::cycles();
#endif
    BedSteps::send(NozzleSerial,kind,masterBedEpoch,masterBedPosition,
        kind=='R'?((uint32_t)activeProfileId<<16|currentSph):0);
}
static bool masterBedStart() {
    if(!bedRecipeConfigured(activeProfileId)){
        Serial.println("[BED] start rejected: invalid IR4 recipe; configure absolute s2disp_at for S2 profiles");return false;
    }
    if(masterBedFault){
        Serial.print("[BED] start rejected: latched fault reason=");Serial.println(masterBedFaultReason);
        Serial.println("[BED] inspect BEDSTATUS; FEEDSTOP then BEDRESET aborts queued work; re-home before restart");
        return false;
    }
    // Boot announcements can be missed when Master restarts after Nozzle.
    // The R/T acknowledgement barrier below proves current readiness and keeps
    // serviceMasterBed from generating even one pulse until Nozzle responds.
    if(!masterBedEstablished){
        if(anyCascadeActive() || M14Motor.isMoving() || ZMotor.isMoving() || M15Motor.isMoving() || XMotor.isMoving() || YMotor.isMoving() || M18Motor.isMoving()){
            Serial.println("[BED] start rejected: manual pump/cascade active");return false;
        }
        masterBedEpoch=micros()^((uint32_t)millis()<<16);if(!masterBedEpoch)masterBedEpoch=1;
        masterBedPosition=0;masterBedEstablished=true;
        for(auto& seq:masterBedSeen)seq=0;
        masterBedSend('R');
    } else {const uint32_t digest=masterBedDigest;masterBedSend('T');masterBedDigest=digest;}
    return true;
}
static void masterBedPause() {
    tFeedActive=false;
    TMotor.setCurrentPosition(TMotor.getCurrentPosition());
    if(masterBedEstablished)BedSteps::send(NozzleSerial,'P',masterBedEpoch,masterBedPosition);
    // Retain all step deadlines. Output state is restored when feed resumes.
    washdc1.stopMotor();washdc2.stopMotor();suctiondc1.stopMotor();suctiondc2.stopMotor();mixdc1.stopMotor();mixdc2.stopMotor();
}
static bool masterBedRequestReady(uint16_t action,uint32_t seq) {
    int delta=0;if(masterBedWindow(action,delta)>=0)return true;
    bool ready=true;
    switch(action){
    case BedSteps::EthPump:ready=!M14Motor.isMoving();break;
    case BedSteps::S1:ready=!ZMotor.isMoving();break;
    case BedSteps::Buffer1:ready=m15MixState==M15MIX_IDLE&&!M15Motor.isMoving();break;
    case BedSteps::S2Start:ready=s2MixState==S2MIX_IDLE&&!XMotor.isMoving()&&!YMotor.isMoving()&&!M18Motor.isMoving();break;
    case BedSteps::S2Dispatch:
        ready=seq==masterBedS2Sequence &&
            (reagentDispenseLocked ? s2MixState==S2MIX_IDLE :
             s2MixState==S2MIX_ACTIVE&&!s2MixDispatchStarted&&!XMotor.isMoving()&&!YMotor.isMoving());break;
    default:masterBedFail(52);return false;
    }
    if(!ready){masterBedFail(60+action);return false;}
    return true;
}
static bool masterBedRequest(const BedSteps::Frame& f) {
    const uint16_t action=f.value&255;const uint32_t seq=f.value>>8;
    masterBedSeen[action]=seq;
    int delta=0;const int window=masterBedWindow(action,delta);
    if(window>=0){masterBedWindows[window]+=delta;pcEvent((TimingKnob)(action-1));}
    else switch(action){
    case BedSteps::EthPump:
        if(!reagentDispenseLocked&&profile().e_enabled){M14Motor.setMaxSpeed(M14_DISPENSE_SPEED);M14Motor.setAcceleration(M14_DISPENSE_ACCEL);M14Motor.move(m14StepsForMl(profile().e_uL/1000.0f));pcEvent(TK_M14_AT);}break;
    case BedSteps::S1:
        if(!reagentDispenseLocked&&profile().s1_enabled){ZMotor.setMaxSpeed(Z_DISPENSE_SPEED);ZMotor.setAcceleration(Z_DISPENSE_ACCEL);ZMotor.move(zStepsForMl(profile().s1_uL/1000.0f));pcEvent(TK_S1_AT);}break;
    case BedSteps::Buffer1:armB1Mixcade();break;
    case BedSteps::S2Start:masterBedS2Sequence=seq;armS2Mixcade();break;
    case BedSteps::S2Dispatch:
        if(!reagentDispenseLocked){
            s2MixDispatchStarted=true;s2MixDispatchStartStep=masterBedPosition;
            m18OscState=M18_OSC_IDLE;
            M18Motor.setMaxSpeed(MIX_DISPATCH_SPEED);M18Motor.move(MIX_DISPATCH_STEPS);
            ackManager.requestMovementAck(&M18Motor,"M18",micros());pcEvent(TK_S2DISP_AT);
        }break;
    }
    Serial.print("[BED] master action=");Serial.print(action);Serial.print(" seq=");Serial.print(seq);
    Serial.print(" step=");Serial.println(masterBedPosition);
    return true;
}
static bool masterBedInput(const String& line) {
    if(!line.startsWith("BS2 "))return false;
    BedSteps::Frame f;
    if(!BedSteps::parse(line.c_str(),f)){masterBedFail(80);return true;}
    if(f.epoch!=masterBedEpoch || f.step!=masterBedPosition){masterBedFail(81);return true;}
    if(f.kind=='F'){masterBedFail(f.value);return true;}
    if(masterBedFault)return true;
    if(f.kind=='A'){
        if(f.value!=masterBedDigest)masterBedFail(87);
        else {
            int counts[5];for(unsigned w=0;w<5;++w)counts[w]=masterBedWindows[w];
            bool used[BedSteps::ActionCount]={};
            for(unsigned i=0;i<masterBedPendingCount;++i){
                const auto value=masterBedPending[i].value;const uint16_t action=value&255;
                if(!masterBedRequestReady(action,value>>8))return true;
                int delta=0;const int window=masterBedWindow(action,delta);
                if(window>=0)counts[window]+=delta;
                else if(used[action]){masterBedFail(90);return true;}
                used[action]=true;
            }
            for(int count:counts)if(count<0){masterBedFail(91);return true;}
            for(unsigned i=0;i<masterBedPendingCount;++i)
                if(!masterBedRequest(masterBedPending[i]))return true;
            masterBedPendingCount=0;masterBedWaiting=false;
#ifdef LIVO_LOOPSTATS
            LoopStats::onBedAck(masterBedSentCycles);LoopStats::bedCommandedStepsPerS=tFeedSpeed;
#endif
            if(tFeedActive)masterBedApplyWindows();
        }
    }
    else if(f.kind=='Q'){
        if(!masterBedWaiting){masterBedFail(88);return true;}
        const uint16_t action=f.value&255;const uint32_t seq=f.value>>8;
        if(!seq || action==0 || action>=BedSteps::ActionCount){masterBedFail(50);return true;}
        if(seq==masterBedSeen[action])return true;
        for(unsigned i=0;i<masterBedPendingCount;++i)if(masterBedPending[i].value==f.value)return true;
        if(masterBedPendingCount>=BedSteps::ActionCount || seq<masterBedSeen[action]){masterBedFail(89);return true;}
        masterBedPending[masterBedPendingCount++]=f;
        masterBedDigest=(masterBedDigest*16777619UL)^f.value;
    }
    else masterBedFail(82);
    return true;
}
static void serviceMasterBed() {
    if(masterBedFault)return;
    if(masterBedWaiting){
        if(millis()-masterBedSentAt>BedSteps::LinkTimeoutMs)masterBedFail(83);
        return;
    }
    if(!tFeedActive){if(!masterBedEstablished)TMotor.loop();return;}
    if(!masterBedEstablished){masterBedFail(84);return;}
    if(!TMotor.isMoving()){TMotor.motor.setMaxSpeed(tFeedSpeed);TMotor.move(-2000000000L);}
    const long before=TMotor.getCurrentPosition();
    TMotor.loop();
    const long after=TMotor.getCurrentPosition();
    const int32_t advanced=(int32_t)((uint32_t)before-(uint32_t)after);
    if(advanced==1){++masterBedPosition;masterBedSend('T');}
    else if(advanced!=0)masterBedFail(85);
}

static void masterBedReset() {
    masterBedPause();
    for(TMCModule* m:{&M14Motor,&ZMotor,&M15Motor,&XMotor,&YMotor,&M18Motor})m->setCurrentPosition(m->getCurrentPosition());
    m15MixState=M15MIX_IDLE;sxCasState=SXCAS_IDLE;wyCasState=WYCAS_IDLE;s2MixState=S2MIX_IDLE;m18OscState=M18_OSC_IDLE;
    BedSteps::send(NozzleSerial,'C',masterBedEpoch,masterBedPosition);
    masterBedEstablished=false;masterBedWaiting=false;masterBedFault=false;masterBedFaultReason=0;lastNozzleQueueDepth=0;
    for(auto& count:masterBedWindows)count=0;masterBedS2Sequence=0;
    Serial.println("[BED] aborted and cleared; remove affected slides and home before next run");
}

static void masterBedStatus() {
    Serial.print("[BED] STATUS fault=");Serial.print(masterBedFault?1:0);
    Serial.print(" reason=");Serial.print(masterBedFaultReason);
    Serial.print(" nozzle_boot_seen=");Serial.print(nozzleReady?1:0);
    Serial.print(" established=");Serial.print(masterBedEstablished?1:0);
    Serial.print(" waiting_ack=");Serial.print(masterBedWaiting?1:0);
    Serial.print(" feed_active=");Serial.print(tFeedActive?1:0);
    Serial.print(" epoch=");Serial.print(masterBedEpoch);
    Serial.print(" step=");Serial.print(masterBedPosition);
    Serial.print(" nozzle_queue=");Serial.println(lastNozzleQueueDepth);
}
