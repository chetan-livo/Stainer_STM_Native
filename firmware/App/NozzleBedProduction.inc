// Included only in the Nozzle section of Execution2019Handler.cpp.
static BedSteps::Scheduler<256> nzBed;
static SlidePositionTracker nzSlidePositions;
static bool nzBedAckPending=false;
static uint32_t nzSlideReportAt=0;
static void nzReportSlidePositions() {
    // Full snapshots recover dropped reports and ESP restarts. Never extrapolate
    // while paused, faulted, or disconnected: only observed bed steps advance.
    if(uint32_t(millis()-nzSlideReportAt)<500 || nzBedAckPending)return;
    uint8_t minutes[15];
    const uint16_t mask=nzSlidePositions.snapshot(T_MOTOR_STEPS_PER_REV, currentSph, minutes);
    char line[52];
    snprintf(line,sizeof(line),"UITR %08lX %04X ",(unsigned long)nzBed.epoch(),unsigned(mask));
    const char* hex="0123456789ABCDEF";
    for(unsigned i=0;i<15;++i){line[19+i*2]=hex[minutes[i]>>4];line[20+i*2]=hex[minutes[i]&15];}
    line[49]='\n';line[50]=0;
    if(MasterSerial.availableForWrite()<50)return;
    MasterSerial.print(line);nzSlideReportAt=millis();
}
static uint32_t nzBedAccepted[3]={};
static uint32_t nzBedSeq[3]={}, nzBedCandidate[3]={}, nzBedLastSample=0;
static bool nzBedFault=false, nzBedEnabled=false;
static uint32_t nzBedLastRx=0,nzBedAckDigest=0;
static bool nzBedCandidateActive[3]={};
static uint16_t nzBedEthWindows=0, nzBedDryWindows=0;
static int nzBedDepth() { return (int)nzBed.depth() +
    (nzSYOpState!=MOP_IDLE)+(nzSXOpState!=MOP_IDLE)+(nzWYOpState!=MOP_IDLE)+(nzWXOpState!=MOP_IDLE)+
    nzBedEthWindows+nzBedDryWindows; }
static void nzBedFail(uint32_t reason) {
    if(nzBedFault) return;
    nzBedFault=true; nzBedEnabled=false;
    nzSlidePositions.invalidate();
    dcFan4.stopHighSpeedFan();dcFan3.stopHighSpeedFan();
    nzFan4Running=nzWXParkedFanActive=false;
    // Halt axes without clearing their outstanding events; recovery needs homing.
    SYMotor.setCurrentPosition(SYMotor.getCurrentPosition());
    SXMotor.setCurrentPosition(SXMotor.getCurrentPosition());
    WYMotor.setCurrentPosition(WYMotor.getCurrentPosition());
    WXMotor.setCurrentPosition(WXMotor.getCurrentPosition());
    nzHomingComplete=false;
    BedSteps::send(MasterSerial,'F',nzBed.epoch(),nzBed.position(),reason);
    Serial.print("[BED] nozzle fault=");Serial.println(reason);
}
static bool nzBedReady(const BedSteps::Event& e) {
    switch(e.action) {
    case BedSteps::SY:return nzSYOpState==MOP_IDLE && !SYMotor.isMoving() && SYMotor.getCurrentPosition()==0;
    case BedSteps::SX:return nzSXOpState==MOP_IDLE && !SXMotor.isMoving() && SXMotor.getCurrentPosition()==nzSXParkPositionForProfile(nzActiveProfileId);
    case BedSteps::WY:return nzWYOpState==MOP_IDLE && !WYMotor.isMoving() && WYMotor.getCurrentPosition()==NZ_WY_FIXED_PARK_POSITION;
    case BedSteps::WX:return nzWXOpState==MOP_IDLE && !WXMotor.isMoving() && WXMotor.getCurrentPosition()==NZ_WX_INIT_POSITION;
    default:return true;
    }
}
static void nzBedStroke(TMCModule& motor,NzMotorOpState& state,long& saved,long speed,long stroke,NzMotorTraceState& trace,const BedSteps::Event& e) {
    saved=motor.getMaxSpeed(); trace={e.sequence,millis(),millis(),motor.getCurrentPosition()};
    motor.setMaxSpeed(speed);motor.move(stroke);state=MOP_STROKING;
}
static BedSteps::Execution nzBedStart(const BedSteps::Event& e) {
    using namespace BedSteps;
    switch(e.action) {
    case SY:nzBedStroke(SYMotor,nzSYOpState,nzSYSavedSpeed,speedFor(nzActiveProfileId,SK_SY_STROKE),NZ_SY_STROKE,nzSYTrace,e);break;
    case SX:nzBedStroke(SXMotor,nzSXOpState,nzSXSavedSpeed,speedFor(nzActiveProfileId,SK_SX_STROKE),NZ_SX_STROKE,nzSXTrace,e);break;
    case WY:nzBedStroke(WYMotor,nzWYOpState,nzWYSavedSpeed,speedFor(nzActiveProfileId,SK_WY_STROKE),nzWYStrokeForProfile(nzActiveProfileId),nzWYTrace,e);break;
    case WX:nzBedStroke(WXMotor,nzWXOpState,nzWXSavedSpeed,speedFor(nzActiveProfileId,SK_WX_STROKE),NZ_WX_STROKE,nzWXTrace,e);break;
    case EthOn:++nzBedEthWindows;break;
    case EthOff:if(!nzBedEthWindows)return Execution::Failed;--nzBedEthWindows;break;
    case DryOn:++nzBedDryWindows;break;
    case DryOff:if(!nzBedDryWindows)return Execution::Failed;--nzBedDryWindows;break;
    default:
        // Action identity is unique at this barrier: action+IR4 slide sequence.
        // Master consumes Q before the subsequent A, before permitting a step.
        nzBedAckDigest=(nzBedAckDigest*16777619UL)^((e.sequence<<8)|e.action);
        BedSteps::send(MasterSerial,'Q',nzBed.epoch(),e.step,(e.sequence<<8)|e.action);
        break;
    }
    Serial.print("[BED] event=");Serial.print(e.action);Serial.print(" seq=");Serial.print(e.sequence);
    Serial.print(" step=");Serial.println(e.step);
    return Execution::Started;
}
static void nzBedAdmit(uint8_t stage,uint32_t anchor) {
    using namespace BedSteps;
    const ProfileId p=nzActiveProfileId;
    if(nzBedSeq[stage] && (uint32_t)(anchor-nzBedAccepted[stage])<BED_SLIDE_MIN_PITCH_STEPS)return;
    nzBedAccepted[stage]=anchor;
    const uint32_t seq=++nzBedSeq[stage];
    if(seq>0xFFFFFFUL){nzBedFail(20);return;} // do not truncate wire identities
    if(stage!=0) {
        // Downstream detections are observations only. They cannot create work,
        // shift a deadline, or assign a different IR4 origin to an existing slide.
        Serial.print("[IR] monitor=");Serial.print(stage==1?6:12);
        Serial.print(" edge=");Serial.print(seq);Serial.print(" bed=");Serial.println(anchor);
        return;
    }
    if(!bedRecipeConfigured(p)){nzBedFail(94);return;}
    static_assert(BedSteps::ActionCount==TK_COUNT+1,"Recipe/wire action order mismatch");
    Event events[TK_COUNT];size_t count=0;
    const auto& recipe=nzProfile();
    for(unsigned k=0;k<TK_COUNT;++k) {
        if(k==TK_M14_AT&&!recipe.e_enabled)continue;
        if(k==TK_S1_AT&&!recipe.s1_enabled)continue;
        if((k==TK_B1_AT||k==TK_AIR_ON||k==TK_AIR_OFF)&&!recipe.b1_enabled)continue;
        if(k==TK_SX_AT&&nzSXIsFixedForProfile(p))continue;
        if((k==TK_W1_ON||k==TK_W1_OFF)&&!recipe.sxcas_dcw1_enabled)continue;
        if((k==TK_SUCTION1_ON||k==TK_SUCTION1_OFF)&&!recipe.sxcas_dcs1_enabled)continue;
        if((k==TK_S2MIX_AT||k==TK_S2DISP_AT)&&!recipe.s2_enabled&&!recipe.b2_enabled)continue;
        if((k==TK_W2_ON||k==TK_W2_OFF)&&!recipe.wycas_dcw2_enabled)continue;
        if((k==TK_SUCTION2_ON||k==TK_SUCTION2_OFF)&&!recipe.wycas_dcs2_enabled)continue;
        if(k==TK_WX_AT&&nzWXIsFixedForProfile(p))continue;
        const uint32_t offset=timingStepsFor(p,(TimingKnob)k);
        events[count++]={anchor+offset,seq,(uint16_t)(k+1),0};
    }
    if(nzBedFault)return;
    if(nzBed.admit(nzBed.epoch(),stage,seq,events,count)!=Admission::Accepted)nzBedFail(30+(uint32_t)nzBed.fault());
    else {
        nzSlidePositions.observe(nzBed.position(),T_MOTOR_STEPS_PER_REV);
        nzSlidePositions.admit(seq,anchor,T_MOTOR_STEPS_PER_REV);
        Serial.print("[SLIDE] IR");Serial.print(stage==0?4:stage==1?6:12);Serial.print(" seq=");Serial.print(seq);Serial.print(" bed=");Serial.println(anchor);
    }
}
bool nozzleBedInput(const String& command) {
    if(!command.startsWith("BS2 "))return false;
    BedSteps::Frame f;
    if(!BedSteps::parse(command.c_str(),f)){nzBedFail(1);return true;}
    nzBedLastRx=millis();
    if(f.kind=='C'){
        for(TMCModule* m:{&SYMotor,&SXMotor,&WYMotor,&WXMotor})m->setCurrentPosition(m->getCurrentPosition());
        if(nzBedDepth())nzHomingComplete=false;
        nzBed=BedSteps::Scheduler<256>();nzBedEnabled=false;nzBedFault=false;nzBedAckPending=false;
        nzSlidePositions.reset();
        nzBedEthWindows=nzBedDryWindows=0;nzSYOpState=nzSXOpState=nzWYOpState=nzWXOpState=MOP_IDLE;
        dcFan3.stopHighSpeedFan();dcFan4.stopHighSpeedFan();nzFan4Running=nzWXParkedFanActive=false;
        return true;
    }
    if(f.kind=='R'){
        // A repeat of the same reset is a retransmission, never an erase.
        if(nzBed.synchronized() && f.epoch==nzBed.epoch()){
            if(nzBedFault || f.step!=nzBed.position())nzBedFail(2);
            else nzBedAckPending=true;
            return true;
        }
        if(nzBedDepth() || !nzHomingComplete || SYMotor.isMoving() || SXMotor.isMoving() || WYMotor.isMoving() || WXMotor.isMoving()){
            BedSteps::send(MasterSerial,'F',f.epoch,f.step,3);return true;
        }
        if(f.value!=((uint32_t)nzActiveProfileId<<16|currentSph) || !bedRecipeConfigured(nzActiveProfileId)){
            BedSteps::send(MasterSerial,'F',f.epoch,f.step,4);return true;
        }
        nzBedAckDigest=0;nzBed.reset(f.epoch,f.step);nzBedFault=false;nzBedEnabled=true;nzBedAckPending=true;
        nzSlidePositions.reset(f.step);
        for(unsigned i=0;i<3;++i){nzBedFilters[i].reset();nzBedSeq[i]=0;nzBedCandidate[i]=f.step;nzBedCandidateActive[i]=false;}
    } else if(f.epoch!=nzBed.epoch()){nzBedFail(5);
    } else if(f.kind=='F'){nzBedFail(f.value);
    } else if(f.kind=='P'){
        nzBedEnabled=false;
        // Paused bed cannot accumulate a liquid wash at a stationary slide.
        dcFan4.stopHighSpeedFan();dcFan3.stopHighSpeedFan();
        nzFan4Running=nzWXParkedFanActive=false;
    } else if(f.kind=='T'){
        if(f.step!=nzBed.position())nzBedAckDigest=0;
        if(nzBedFault || !nzBed.synchronized() || BedSteps::difference(f.step,nzBed.position())>1 || !nzBed.observe(f.epoch,f.step))nzBedFail(6);
        else {nzBedEnabled=true;nzBedAckPending=true;nzSlidePositions.observe(f.step,T_MOTOR_STEPS_PER_REV);}
    } else nzBedFail(7);
    return true;
}
static void serviceNozzleBedProduction() {
    if(nzBedFault || !nzBedEnabled)return;
    nzServiceMotor("BED",SYMotor,"SY",nzSYOpState,NZ_SY_STROKE,nzSYSavedSpeed,speedFor(nzActiveProfileId,SK_SY_RETURN),nzSYTrace);
    nzServiceMotor("BED",SXMotor,"SX",nzSXOpState,NZ_SX_STROKE,nzSXSavedSpeed,speedFor(nzActiveProfileId,SK_SX_RETURN),nzSXTrace);
    nzServiceMotor("BED",WYMotor,"WY",nzWYOpState,nzWYStrokeForProfile(nzActiveProfileId),nzWYSavedSpeed,speedFor(nzActiveProfileId,SK_WY_RETURN),nzWYTrace);
    nzServiceMotor("BED",WXMotor,"WX",nzWXOpState,NZ_WX_STROKE,nzWXSavedSpeed,speedFor(nzActiveProfileId,SK_WX_RETURN),nzWXTrace);
    if(!nzBedEnabled)return;
    if(millis()-nzBedLastRx>BedSteps::LinkTimeoutMs){nzBedFail(8);return;}
    int* ir=limitSwitch.getSmoothedSensorValues();
    const uint32_t revision=limitSwitch.sampleRevision();
    if(!ir){nzBedFail(9);return;}
    if(revision!=nzBedLastSample){
        nzBedLastSample=revision;
        const int channels[]={NZ_IR4_IDX,NZ_IR6_IDX,NZ_IR12_IDX};
        const int present[]={NZ_IR4_THRESHOLD,NZ_IR6_THRESHOLD,NZ_IR12_THRESHOLD};
        const int clear[]={NZ_IR4_CLEAR_THRESHOLD,NZ_IR6_CLEAR_THRESHOLD,NZ_IR12_CLEAR_THRESHOLD};
        for(uint8_t i=0;i<3;++i){
            if(!SensorFaultPolicy::adcValid(ir[channels[i]])){nzBedFail(10+i);return;}
            // Capture the bed coordinate at the first candidate sample, retaining
            // debounce time only as an electrical-noise filter.
            if(ir[channels[i]]<present[i]){
                if(!nzBedCandidateActive[i]){nzBedCandidate[i]=nzBed.position();nzBedCandidateActive[i]=true;}
            } else nzBedCandidateActive[i]=false;
            if(nzBedFilters[i].sample(millis(),ir[channels[i]],present[i],clear[i],NZ_SLIDE_PRESENT_STABLE_MS,
                i?NZ_CASCADE_CLEAR_STABLE_MS:NZ_SLIDE_CLEAR_STABLE_MS))nzBedAdmit(i,nzBedCandidate[i]);
        }
    }
    if(!nzBedFault && !nzBed.dispatch(nzBedReady,nzBedStart))nzBedFail(30+(uint32_t)nzBed.fault());
    if(nzBedFault)return;
    if(nzFan4Running!=(nzBedEthWindows!=0)){
        nzFan4Running=nzBedEthWindows!=0;
        if(nzFan4Running)dcFan4.runHighSpeedFanPercent(NZ_ETHANOL_SPEED_PCT);else dcFan4.stopHighSpeedFan();
    }
    if(nzWXParkedFanActive!=(nzBedDryWindows!=0)){
        nzWXParkedFanActive=nzBedDryWindows!=0;
        if(nzWXParkedFanActive)nzStartWXStrokeFan();else nzStopWXStrokeFan();
    }
    // Keep the barrier closed until all Q messages are ahead of A on the wire.
    if(nzBedAckPending){BedSteps::send(MasterSerial,'A',nzBed.epoch(),nzBed.position(),nzBedAckDigest);nzBedAckPending=false;}
    nzServiceIR4Trace(ir[NZ_IR4_IDX]);nzServiceIR12Trace(ir[NZ_IR12_IDX]);
    nzReportPipelineDepthIfChanged();
}

bool nozzleBedCommandAllowed(const String& command) {
    if(!nzBedEnabled && !nzBedDepth())return true;
    return command.startsWith("FEED ") || command.startsWith("PROFILE ") ||
        command=="ID" || command=="COMMSTAT" || command=="LOOPSTAT" || command=="PROFILE" ||
        command=="PARA STOP" || command.startsWith("PIR ");
}
