#ifndef LIVO_PARA_CHECK_H
#define LIVO_PARA_CHECK_H

// Observational, single-slide trace. All positions are real T step-counter
// differences; no millis-to-steps estimate is used for CURRENT.
struct ParaCheckTrace {
    bool active = false, anchored = false, fanDone = false;
    unsigned long startedMs = 0;
    long origin = 0, sxArm = -1, wyArm = -1, b1End = -1;
    unsigned long value[TK_COUNT] = {};
    long actual[TK_COUNT];
    const char* status[TK_COUNT];

    static const char* name(unsigned int k) {
        static const char* names[] = {"m14_at", "eth_on", "eth_off", "sy_at", "s1_at", "b1_at", "air_on", "air_off", "sx_at", "w1_on", "w1_off", "suction1_on", "suction1_off", "s2mix_at", "s2disp_at", "w2_on", "w2_off", "suction2_on", "suction2_off", "wy_at", "wx_at", "dry_on", "dry_off"};
        return names[k];
    }
    long current(long position) const { return origin - position; }
    void begin(ProfileId pid, long position) {
        active = true; anchored = false; fanDone = false;
        startedMs = millis(); origin = position;
        sxArm = wyArm = b1End = -1;
        for (unsigned int k = 0; k < TK_COUNT; ++k) {
            value[k] = timingStepsFor(pid, (TimingKnob)k);
            actual[k] = -1; status[k] = "NOT_OBSERVED";
        }

    }
    long expected(unsigned int k) const {
        return value[k]==UNSET_BED_POSITION?-1:(long)value[k];
    }
    void row(Print& out, unsigned int k) const {
        out.print("[PARA] "); out.print(name(k)); out.print(" CURRENT:");
        if (actual[k] < 0) out.print("N/A"); else out.print(actual[k]);
        const long target = expected(k);
        out.print(" PARAMETER:");
        if (target < 0) out.print("N/A"); else out.print(target);
        out.print(" VALUE:"); out.print(value[k]);
        out.print(" DELTA:");
        if (actual[k] < 0 || target < 0) out.print("N/A");
        else out.print(actual[k] - target);
        out.print(" STATUS:"); out.println(status[k]);
    }
    void event(Print& out, TimingKnob k, long position, const char* state = "FIRED") {
        if (!active || !anchored) return;
        if (actual[k] >= 0) {
            out.print("[PARA] DUPLICATE "); out.println(name(k));
            return;
        }
        actual[k] = current(position); status[k] = state; row(out, k);
    }
    void note(Print& out, const char* eventName, long position) const {
        if (!active || !anchored) return;
        out.print("[PARA] "); out.print(eventName);
        out.print(" CURRENT:"); out.print(current(position));
        out.println(" PARAMETER:N/A");
    }
    void finish(Print& out, const char* reason) {
        if (!active) return;
        out.print("[PARA] END "); out.println(reason);
        active = false;
    }
};
#endif
