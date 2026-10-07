// Fresh post-motion rolling windows, one bounded correction, then verification.
namespace GzArrival {
enum State { Idle, Moving, Settling, Confirming, Correcting, Passed, Mismatch, Invalid };
static State state=Idle;
static long target=0, firstEstimate=0, savedSpeed=0, savedAcceleration=0, correctionDelta=0;
static unsigned long settledAt=0, lastPoll=0, correctionAt=0;
static uint16_t firstSequence=0, firstUncertainty=0;
static uint32_t modelId=0;
static int32_t minimum=0, maximum=0;
static bool correctionAttempted=false, correctionAllowed=true, profileSaved=false, haveFirst=false;
static constexpr long toleranceSteps=350; // 328.125 um; accepted map peak 335.078 steps.
static constexpr long stabilitySteps=64, maxCorrectionSteps=500;
static constexpr unsigned long filterFlushMs=100, pollMs=10;
static constexpr uint32_t expectedModelId=1196255454UL;
static bool pending() { return state==Moving || state==Settling || state==Confirming || state==Correcting; }
static bool passed() { return state==Passed; }
static bool failed() { return state==Mismatch || state==Invalid; }
static void restore() {
    if(profileSaved) { GZMotor.setMaxSpeed(savedSpeed);GZMotor.setAcceleration(savedAcceleration);profileSaved=false; }
}
static void invalid(const char* reason) {
    if(state==Correcting) GZMotor.setCurrentPosition(GZMotor.getCurrentPosition());
    restore();
    if(correctionAttempted) GzCalibration::homed=false;
    state=Invalid;
    Serial.print("GZVERIFY target=");Serial.print(target);
    Serial.print(" result=UNVERIFIED reason=");Serial.println(reason);
}
static void arm(long intended, bool allowCorrection=true) {
    if(pending()) { invalid("target_superseded");return; }
    target=intended;state=Moving;modelId=0;haveFirst=false;
    correctionAttempted=false;correctionAllowed=allowCorrection;profileSaved=false;
}
static void cancel() { if(pending()) invalid("cancelled"); }
static bool command(const String& text) {
    if(text!="GZCHECK" && text!="GZCORRECT") return false;
    if(pending() || GzFeedback::busy() || GzCalibration::enabled || GZMotor.isMoving() ||
       GzCalibration::otherMotion() || serviceProductionBusy()) { Serial.println("GZVERIFY BUSY");return true; }
    arm(GZMotor.getCurrentPosition(),text=="GZCORRECT");return true;
}
static void beginSettling() {
    settledAt=millis();lastPoll=settledAt-pollMs;haveFirst=false;state=Settling;
}
static void tick() {
    if(!pending()) return;
    if(!GzCalibration::homed) { invalid("home_required");return; }
    if(GzCalibration::enabled) { invalid("calibration_active");return; }
    if(state==Correcting) {
        if(GzCalibration::otherMotion()) { invalid("other_motion_during_correction");return; }
        if(millis()-correctionAt>2000UL) { invalid("correction_timeout");return; }
        if(correctionDelta<0 && digitalRead(Lim2)==LOW) { invalid("top_input_active");return; }
        if(GZMotor.isMoving()) return;
        restore();
        if(GZMotor.getCurrentPosition()!=target) { invalid("correction_interrupted");return; }
        beginSettling();return;
    }
    if(GZMotor.isMoving()) {
        if(state!=Moving) invalid("motion_during_check");
        return;
    }
    if(GZMotor.getCurrentPosition()!=target) { invalid("target_not_reached");return; }
    if(state==Moving) { beginSettling();return; }
    if(millis()-settledAt>5000UL) { invalid("verification_timeout");return; }
    if(GzCalibration::otherMotion() || millis()-lastPoll<pollMs) return;
    lastPoll=millis();
    if(!modelId) {
        uint8_t info[19];
        if(!GzFeedback::receive(CMD_Z_HALL_MODEL_INFO,info,sizeof(info)) ||
           info[0]!=1 || info[1]!=1 || GzFeedback::u16(info+16)<2) { invalid("model_unavailable");return; }
        modelId=GzFeedback::u32(info+2);
        if(modelId!=expectedModelId) { invalid("model_mismatch");return; }
        if(GzFeedback::u16(info+14)!=GZMotor.getMicrosteps() || GZMotor.getMicrosteps()!=32) {
            invalid("microstep_mismatch");return;
        }
        minimum=(int32_t)GzFeedback::u32(info+6);maximum=(int32_t)GzFeedback::u32(info+10);
        if(minimum>=maximum || target<minimum || target>maximum) { invalid("target_outside_mapping");return; }
    }
    if(GZMotor.getMicrosteps()!=32) { invalid("microstep_mismatch");return; }
    uint8_t snapshot[20];
    // Elapsed captured BEFORE I2C: transfer time must not make an old window eligible.
    const unsigned long elapsed=millis()-settledAt;
    if(!GzFeedback::receive(CMD_Z_HALL_FAST_SNAPSHOT,snapshot,sizeof(snapshot))) { invalid("hall_read_failed");return; }
    const uint16_t age=GzFeedback::u16(snapshot+4), span=GzFeedback::u16(snapshot+12);
    if(snapshot[0]!=1 || GzFeedback::u32(snapshot+14)!=modelId) { invalid("model_or_protocol_mismatch");return; }
    if(age>=100 || snapshot[18]!=32 || span<100 || span>250) { invalid("hall_stale_or_incomplete");return; }
    // All samples must follow arrival AND filter washout; no fixed 500 ms sleep.
    if(elapsed<(unsigned long)age+span+filterFlushMs) return;
    if(snapshot[1]!=1) { invalid("hall_estimate_rejected");return; }
    const int32_t estimate=(int32_t)GzFeedback::u32(snapshot+6);
    const uint16_t sequence=GzFeedback::u16(snapshot+2), uncertainty=GzFeedback::u16(snapshot+10);
    if(estimate<minimum || estimate>maximum || uncertainty>toleranceSteps) { invalid("estimate_outside_range_or_uncertain");return; }
    if(!haveFirst) {
        firstEstimate=estimate;firstSequence=sequence;firstUncertainty=uncertainty;
        haveFirst=true;state=Confirming;return;
    }
    if(sequence==firstSequence) return;
    if(labs(estimate-firstEstimate)>stabilitySteps) {
        firstEstimate=estimate;firstSequence=sequence;firstUncertainty=uncertainty;return;
    }
    const long error=target-estimate, firstError=target-firstEstimate;
    if((labs(error)<=toleranceSteps)!=(labs(firstError)<=toleranceSteps)) {
        firstEstimate=estimate;firstSequence=sequence;firstUncertainty=uncertainty;return;
    }
    if(labs(error)<=toleranceSteps && labs(firstError)<=toleranceSteps) {
        state=Passed;
    } else if(correctionAllowed && !correctionAttempted && labs(error)>toleranceSteps &&
              labs(firstError)>toleranceSteps && (error<0)==(firstError<0) &&
              labs(error)<=maxCorrectionSteps && labs(firstError)<=maxCorrectionSteps &&
              uncertainty<=250 && firstUncertainty<=250) {
        const long actual=(estimate+firstEstimate)/2;
        correctionDelta=target-actual;
        if(correctionDelta<0 && digitalRead(Lim2)==LOW) { invalid("top_input_active");return; }
        savedSpeed=GZMotor.getMaxSpeed();savedAcceleration=GZMotor.getAcceleration();profileSaved=true;
        if(savedSpeed<=0 || savedAcceleration<=0) { invalid("motion_profile_invalid");return; }
        GZMotor.setMaxSpeed(savedSpeed<2000 ? savedSpeed : 2000);
        GZMotor.setAcceleration(savedAcceleration<10000 ? savedAcceleration : 10000);
        GZMotor.setCurrentPosition(actual);GZMotor.moveTo(target);
        correctionAttempted=true;correctionAt=millis();state=Correcting;
        Serial.print("GZAUTOCORRECT steps=");Serial.println(correctionDelta);return;
    } else {
        state=Mismatch;
        if(correctionAttempted) GzCalibration::homed=false;
    }
    Serial.print("GZVERIFY target=");Serial.print(target);
    Serial.print(" hall=");Serial.print(estimate);
    Serial.print(" error_steps=");Serial.print(error);
    Serial.print(" tolerance_steps=");Serial.print(toleranceSteps);
    Serial.print(" corrected=");Serial.print(correctionAttempted ? 1 : 0);
    Serial.print(" result=");Serial.println(passed() ? "PASS" : "MISMATCH");
}
}
