// Supervised calibration only. No production move or automatic correction.
// Included inside the gantry translation unit after motor and UART declarations.
namespace GzCalibration {
enum Phase { Idle, SeekTop, Backoff, Approach, Jog };
static Phase phase = Idle;
static bool enabled = false, homed = false;
static unsigned long heartbeat = 0, started = 0, sampled = 0;
static long savedSpeed = 0, savedAcceleration = 0;
static void halt(const char* reason) {
    GZMotor.setCurrentPosition(GZMotor.getCurrentPosition());
    if (enabled) {
        GZMotor.setMaxSpeed(savedSpeed);
        GZMotor.setAcceleration(savedAcceleration);
    }
    enabled = false;
    homed = false;
    phase = Idle;
    Serial.print("GZCAL STOP "); Serial.println(reason);
}
static bool otherMotion() {
    return GXMotor.isMoving() || GYMotor.isMoving() || GRMotor.isMoving() || XMotor.isMoving();
}
static void sample(bool filtered = false) {
    // Existing raw-Hall packet: ten big-endian ADC readings plus sum8.
    // Record both step counters: moving samples are observation only, not fit data.
    const long before = GZMotor.getCurrentPosition();
    Wire3.beginTransmission(0x15);
    Wire3.write((uint8_t)(filtered ? CMD_FILTERED_HALL_DATA : CMD_HALL_DATA));
    const uint8_t error = Wire3.endTransmission();
    uint8_t bytes[21], count = 0, sum = 0;
    if (!error) {
        Wire3.requestFrom((uint8_t)0x15, (uint8_t)21);
        while (Wire3.available() && count < sizeof(bytes)) bytes[count++] = Wire3.read();
    }
    if (error || count != sizeof(bytes)) {
        // STM32duino 3.0.0 can put a failed master into slave LISTEN state.
        Wire3.end(); Wire3.begin();
        Serial.println("GZCAL HALL_ERROR");
        if (enabled) halt("hall_transaction_failed");
        return;
    }
    for (uint8_t i = 0; i < 20; ++i) sum += bytes[i];
    if (sum != bytes[20]) {
        Serial.println("GZCAL CHECKSUM_ERROR");
        if (enabled) halt("hall_checksum_failed");
        return;
    }
    Serial.print(filtered ? "GZFILT," : "GZRAW,"); Serial.print(millis());
    Serial.print(','); Serial.print(before);
    Serial.print(','); Serial.print(GZMotor.getCurrentPosition());
    Serial.print(','); Serial.print(GZMotor.isMoving() ? 1 : 0);
    Serial.print(','); Serial.print(digitalRead(Lim2) == LOW ? 1 : 0);
    for (uint8_t i = 0; i < 10; ++i) {
        Serial.print(','); Serial.print(((unsigned)bytes[2*i] << 8) | bytes[2*i+1]);
    }
    Serial.println();
}
static bool command(const String& text) {
    if (!text.startsWith("GZCAL")) return false;
    if (text == "GZCAL STOP") { halt("requested"); return true; }
    if (text == "GZCAL END") {
        if (!enabled || !homed || phase != Idle || GZMotor.isMoving()) {
            Serial.println("GZCAL REJECTED end_requires_stationary_home_reference");return true;
        }
        GZMotor.setMaxSpeed(savedSpeed);GZMotor.setAcceleration(savedAcceleration);
        enabled=false;Serial.println("GZCAL ENDED reference_retained");return true;
    }
    if (text == "GZCAL PING") { heartbeat = millis(); return true; }
    if (text == "GZCAL READ" || text == "GZCAL READF") {
        if (!GZMotor.isMoving() && !otherMotion() && !serviceProductionBusy()) sample(text == "GZCAL READF");
        else Serial.println("GZCAL BUSY");
        return true;
    }
    if (text == "GZCAL HOME") {
        if (enabled || GZMotor.isMoving() || otherMotion() || serviceProductionBusy()) {
            Serial.println("GZCAL BUSY"); return true;
        }
        savedSpeed = GZMotor.getMaxSpeed();
        savedAcceleration = GZMotor.getAcceleration();
        GZMotor.setMaxSpeed(1000); GZMotor.setAcceleration(1000);
        enabled = true; homed = false; phase = SeekTop;
        heartbeat = started = millis();
        // Finite upward seek; top input and heartbeat are checked each tick.
        // Repeated 500-step acceleration ramps could exceed the home timeout.
        GZMotor.move(-85000L);
        Serial.println("GZCAL HOME_STARTED");
        return true;
    }
    if (text.startsWith("GZCAL JOG ")) {
        const String arg = text.substring(10);
        char* end = nullptr;
        const long delta = strtol(arg.c_str(), &end, 10);
        if (!enabled || !homed || phase != Idle || !arg.length() || *end ||
            delta == 0 || delta < -500 || delta > 500 ||
            GZMotor.getCurrentPosition() + delta < 0 ||
            GZMotor.getCurrentPosition() + delta > 68500L) {
            Serial.println("GZCAL REJECTED jog_requires_home_and_max_500_steps"); return true;
        }
        heartbeat = started = millis();
        GZMotor.setMaxSpeed(500); GZMotor.setAcceleration(1000);
        phase = Jog; GZMotor.move(delta);
        Serial.println("GZCAL JOG_STARTED");
        return true;
    }
    Serial.println("GZCAL commands: HOME | PING | READ | READF | JOG <-500..500> | END | STOP");
    return true;
}
static void tick() {
    if (!enabled) return;
    const unsigned long now = millis();
    if (now - heartbeat > 3000) { halt("heartbeat_timeout"); return; }
    if (otherMotion()) { halt("other_axis_motion"); return; }
    if (phase != Idle && now - started > 90000) { halt("motion_timeout"); return; }
    const bool top = digitalRead(Lim2) == LOW;
    if (phase == SeekTop && top) {
        GZMotor.setCurrentPosition(GZMotor.getCurrentPosition());
        phase = Backoff; GZMotor.move(2500);
    } else if (phase == SeekTop && !GZMotor.isMoving()) {
        halt("home_travel_limit"); return;
    }
    else if (phase == Backoff && !GZMotor.isMoving()) {
        if (top) { halt("top_still_active_after_backoff"); return; }
        phase = Approach; GZMotor.setMaxSpeed(500); GZMotor.move(-500);
    } else if (phase == Approach && top) {
        GZMotor.setCurrentPosition(0); phase = Idle; homed = true;
        Serial.println("GZCAL HOMED 0");
    } else if (phase == Approach && !GZMotor.isMoving()) GZMotor.move(-100);
    else if (phase == Jog && !GZMotor.isMoving()) {
        phase = Idle; Serial.print("GZCAL ARRIVED "); Serial.println(GZMotor.getCurrentPosition());
    }
    if (now - sampled >= 100) { sampled = now; sample(); }
}
}
