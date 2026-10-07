// On-demand checks only. A correction is one bounded move followed by a check;
// it never retries automatically and cannot run with an unvalidated Hall table.
namespace GzFeedback {
enum Phase { Idle, Waiting, Confirming, Correcting, Verifying };
static Phase phase = Idle;
static bool correctionRequested = false;
static unsigned long started = 0;
static long target = 0, savedSpeed = 0, savedAcceleration = 0;
static int32_t minimum = 0, maximum = 0;
static uint32_t calibrationId = 0;
static long firstError = 0;
static uint16_t firstSequence = 0;
static bool busy() { return phase != Idle; }
static bool anyMotion() { return GZMotor.isMoving() || GzCalibration::otherMotion(); }
static void restore() {
    GZMotor.setMaxSpeed(savedSpeed); GZMotor.setAcceleration(savedAcceleration);
}
static void cancel(const char* reason) {
    if (phase == Correcting) {
        GZMotor.setCurrentPosition(GZMotor.getCurrentPosition()); restore();
        GzCalibration::homed = false;
    }
    phase = Idle;
    Serial.print("GZCHECK INVALID "); Serial.println(reason);
}
static bool receive(uint8_t command, uint8_t* data, uint8_t size) {
    Wire3.beginTransmission(0x15); Wire3.write(command);
    if (Wire3.endTransmission() != 0) { Wire3.end(); Wire3.begin(); return false; }
    Wire3.requestFrom((uint8_t)0x15,size);
    uint8_t count=0, sum=0;
    while (Wire3.available() && count<size) data[count++]=Wire3.read();
    if (count!=size) { Wire3.end(); Wire3.begin(); return false; }
    for (uint8_t i=0;i<size-1;++i) sum+=data[i];
    return sum==data[size-1];
}
static uint32_t u32(const uint8_t* b) {
    return ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
}
static uint16_t u16(const uint8_t* b) { return ((uint16_t)b[0]<<8)|b[1]; }
static bool command(const String& text) {
    if (text!="GZCHECK" && text!="GZCORRECT") return false;
    if (busy() || GzCalibration::enabled || anyMotion() || serviceProductionBusy()) {
        Serial.println("GZCHECK BUSY"); return true;
    }
    correctionRequested = text=="GZCORRECT";
    if (!GzCalibration::homed) {
        Serial.println("GZCHECK INVALID home_required"); return true;
    }
    target=GZMotor.getCurrentPosition(); started=millis(); phase=Waiting;
    Serial.println("GZCHECK SETTLING"); return true;
}
static void tick() {
    if (!busy()) return;
    if (correctionRequested && !GzCalibration::homed) { cancel("reference_lost"); return; }
    if (GzCalibration::enabled || GzCalibration::otherMotion()) { cancel("other_motion"); return; }
    if (phase==Correcting) {
        if (millis()-started>5000UL) { cancel("correction_timeout"); return; }
        if (digitalRead(Lim2)==LOW && GZMotor.getCurrentPosition()>target) { cancel("top_input_active"); return; }
        if (!GZMotor.isMoving()) { restore();phase=Verifying;started=millis(); }
        return;
    }
    if (GZMotor.isMoving() || GZMotor.getCurrentPosition()!=target) { cancel("position_changed"); return; }
    if (millis()-started<500UL) return;
    uint8_t info[19], snapshot[32];
    if (!receive(CMD_Z_HALL_MODEL_INFO,info,sizeof(info)) || info[0]!=1 || info[1]!=1 || u16(info+16)<2) {
        cancel("model_unavailable"); return;
    }
    if (u16(info+14)!=GZMotor.getMicrosteps()) { cancel("microstep_mismatch"); return; }
    if ((phase==Confirming || phase==Verifying) && u32(info+2)!=calibrationId) {
        GzCalibration::homed=false;cancel("model_changed");return;
    }
    calibrationId=u32(info+2); minimum=(int32_t)u32(info+6); maximum=(int32_t)u32(info+10);
    if (!calibrationId || target<minimum || target>maximum) { cancel("target_outside_calibration"); return; }
    if (!receive(CMD_Z_HALL_SNAPSHOT,snapshot,sizeof(snapshot))) { cancel("hall_read_failed"); return; }
    if (snapshot[0]!=1 || u16(snapshot+3)>=250) { cancel("hall_estimate_invalid_or_stale"); return; }
    const int32_t actual=(int32_t)u32(snapshot+5);
    const long tolerance=max(100L,(long)u16(snapshot+9));
    if (actual<minimum || actual>maximum) { cancel("estimate_outside_calibration"); return; }
    const long error=target-actual;
    Serial.print("GZCHECK commanded=");Serial.print(target);
    Serial.print(" hall=");Serial.print(actual);
    Serial.print(" error=");Serial.print(error);
    Serial.print(" tolerance=");Serial.print(tolerance);
    Serial.print(" model=");Serial.print(calibrationId,HEX);
    Serial.print(" result=");Serial.println(labs(error)<=tolerance ? "WITHIN_TOLERANCE" : "MISMATCH");
    if (phase==Verifying || !correctionRequested || labs(error)<=tolerance) {phase=Idle;return;}
    if (labs(error)>500 || tolerance>250) {cancel("correction_limit_rehome_required");return;}
    if (error<0 && digitalRead(Lim2)==LOW) {cancel("top_input_active");return;}
    if (phase==Waiting) {
        firstError=error;firstSequence=u16(snapshot+1);
        phase=Confirming;started=millis();Serial.println("GZCORRECT CONFIRMING");return;
    }
    if (u16(snapshot+1)==firstSequence || (error<0)!=(firstError<0) || labs(error-firstError)>tolerance) {
        cancel("estimate_not_repeatable");return;
    }
    savedSpeed=GZMotor.getMaxSpeed();savedAcceleration=GZMotor.getAcceleration();
    GZMotor.setMaxSpeed(500);GZMotor.setAcceleration(1000);
    GZMotor.setCurrentPosition(actual);GZMotor.moveTo(target);
    phase=Correcting;started=millis();
    Serial.print("GZCORRECT bounded_steps=");Serial.println(error);
}
}
