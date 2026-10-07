// Equivalence test: the clean-room MotionPlanner against AccelStepper 1.64,
// the library the Arduino firmware steps with.
//
// AccelStepper is compiled from the local Arduino sketchbook only into this
// test program (GPL-3.0; never part of the firmware). Both are polled every
// microsecond on a fake clock, the way AccelStepper::run() works; the
// StepEngine interrupt path (StepCountdown at 20 us ticks) is checked too.
#include "MotionPlanner.h"
#include "check.h"
#include <AccelStepper.h>
#undef millis
#undef micros
#undef delay
#undef delayMicroseconds
#undef pinMode
#undef digitalWrite
#undef digitalRead
#undef yield
#include <math.h>
#include <stdio.h>
#include <vector>

static unsigned long nowUs = 0;
unsigned long arduino_micros(void) { return nowUs; }
unsigned long arduino_millis(void) { return nowUs / 1000; }
void arduino_delay(unsigned long) {}
void arduino_delayMicroseconds(unsigned int) {}
void arduino_pinMode(uint8_t, uint8_t) {}
void arduino_digitalWrite(uint8_t, uint8_t) {}
int arduino_digitalRead(uint8_t) { return 0; }
void arduino_yield(void) {}

struct RecordingStepper : AccelStepper {
    std::vector<unsigned long> times;
    std::vector<long> positions;
    RecordingStepper() : AccelStepper(AccelStepper::DRIVER, 1, 2) {}
    void step(long position) override { times.push_back(nowUs); positions.push_back(position); }
};

// What happens at a given step count during the move.
struct Event { long atStep; enum Kind { None, MoveTo, Stop, MaxSpeed, Accel } kind; double value; };

struct Run { std::vector<unsigned long> times; std::vector<long> positions; long end; };

template <typename Apply, typename Step>
void drive(Apply apply, Step stepIfDue, const std::vector<Event>& events, bool& running, unsigned long limitUs)
{
    size_t next = 0;
    long steps = 0;
    for (nowUs = 0; running && nowUs < limitUs; ++nowUs) {
        if (stepIfDue()) {
            ++steps;
            while (next < events.size() && events[next].atStep == steps) apply(events[next++]);
        }
    }
}

Run referenceRun(float vmax, float accel, long target, const std::vector<Event>& events)
{
    RecordingStepper ref;
    ref.setMaxSpeed(vmax); ref.setAcceleration(accel); ref.moveTo(target);
    bool running = true;
    size_t before = 0;
    drive([&](const Event& e) {
              if (e.kind == Event::MoveTo) ref.moveTo((long)e.value);
              if (e.kind == Event::Stop) ref.stop();
              if (e.kind == Event::MaxSpeed) ref.setMaxSpeed((float)e.value);
              if (e.kind == Event::Accel) ref.setAcceleration((float)e.value); },
          [&]() { running = ref.run(); bool stepped = ref.times.size() != before; before = ref.times.size(); return stepped; },
          events, running, 120000000UL);
    return {ref.times, ref.positions, ref.currentPosition()};
}

// Polled engine semantics (PolledStepper): step when the interval has elapsed
// since the previous step, then plan the next.
Run polledRun(float vmax, float accel, long target, const std::vector<Event>& events)
{
    MotionPlanner p;
    p.setMaxSpeed(vmax); p.setAcceleration(accel); p.moveTo(target);
    Run run;
    unsigned long last = 0;
    bool running = true;
    drive([&](const Event& e) {
              if (e.kind == Event::MoveTo) p.moveTo((long)e.value);
              if (e.kind == Event::Stop) p.stop();
              if (e.kind == Event::MaxSpeed) p.setMaxSpeed((float)e.value);
              if (e.kind == Event::Accel) p.setAcceleration((float)e.value); },
          [&]() {
              running = p.isRunning();
              const uint32_t interval = p.stepIntervalUs();
              if (!interval || nowUs - last < interval) return false;
              last = nowUs;
              p.stepTaken();
              run.times.push_back(nowUs); run.positions.push_back(p.currentPosition());
              running = p.isRunning();
              return true; },
          events, running, 120000000UL);
    run.end = p.currentPosition();
    return run;
}

// Interrupt engine semantics: 50 kHz tick with StepCountdown.
Run tickRun(float vmax, float accel, long target)
{
    MotionPlanner p;
    p.setMaxSpeed(vmax); p.setAcceleration(accel); p.moveTo(target);
    StepCountdown c;
    Run run;
    for (unsigned long t = 20; p.isRunning() && t < 120000000UL; t += 20) {
        if (!p.stepIntervalUs()) continue;
        if (!c.tick(20)) continue;
        p.stepTaken();
        c.scheduleNext(p.stepIntervalUs());
        run.times.push_back(t); run.positions.push_back(p.currentPosition());
    }
    run.end = p.currentPosition();
    return run;
}

struct Case { const char* name; float vmax, accel; long target; std::vector<Event> events; };

int main()
{
    const std::vector<Case> cases = {
        {"M14 dispense",          25600, 128000, 20000, {}},
        {"Y B2 dispense",         50000, 256000, 27000, {}},
        {"GX travel (triangle)",  30000, 10000,  60000, {}},
        {"WX wash stroke",        22000, 400000, 6000,  {}},
        {"SY stain stroke",       10000, 100000, 4000,  {}},
        {"M18 mix stroke",        75000, 400000, -3000, {}},
        {"T bed 80 sps",          80,    1600,   2000,  {}},
        {"short move",            5000,  20000,  3,     {}},
        {"retarget further",      20000, 50000,  5000,  {{1500, Event::MoveTo, 12000}}},
        {"retarget closer",       20000, 50000,  12000, {{3000, Event::MoveTo, 3500}}},
        {"reverse mid-move",      20000, 50000,  12000, {{2500, Event::MoveTo, -4000}}},
        {"stop() at speed",       25600, 128000, 20000, {{3000, Event::Stop, 0}}},
        {"lower max speed",       30000, 60000,  40000, {{5000, Event::MaxSpeed, 12000}}},
        {"raise max speed",       12000, 60000,  40000, {{5000, Event::MaxSpeed, 30000}}},
        {"change acceleration",   20000, 40000,  30000, {{800, Event::Accel, 160000}}},
    };
    for (const Case& c : cases) {
        const Run ref = referenceRun(c.vmax, c.accel, c.target, c.events);
        const Run pol = polledRun(c.vmax, c.accel, c.target, c.events);
        CHECK(ref.end == pol.end);
        CHECK(ref.times.size() == pol.times.size());
        CHECK(ref.positions == pol.positions);
        long worst = 0;
        for (size_t i = 0; i < ref.times.size() && i < pol.times.size(); ++i) {
            const long d = labs((long)ref.times[i] - (long)pol.times[i]);
            if (d > worst) worst = d;
        }
        const double total = ref.times.empty() ? 0 : (double)ref.times.back();
        const double totalPol = pol.times.empty() ? 0 : (double)pol.times.back();
        const double drift = total ? 100.0 * fabs(totalPol - total) / total : 0;
        // Float vs AccelStepper's mixed float/double arithmetic may move a
        // step by a microsecond or two; whole moves must agree within 0.05 %.
        CHECK(drift <= 0.05);
        printf("  %-22s steps %6zu  end %6ld  worst step offset %4ld us  duration %9.1f ms (ref %9.1f)  drift %.4f%%\n",
               c.name, pol.times.size(), pol.end, worst, totalPol / 1000.0, total / 1000.0, drift);

        if (c.events.empty()) {
            // Interrupt engine: same steps; within one 20 us tick of exact time.
            // Compare first-to-last-step spans: the fake clock starts at 0, so
            // AccelStepper's first step waits one c0 there, while on hardware an
            // idle motor (and the tick engine) steps immediately.
            const Run tick = tickRun(c.vmax, c.accel, c.target);
            CHECK(tick.end == c.target && tick.positions == pol.positions);
            if (ref.times.size() > 1) {
                const double span = (double)(ref.times.back() - ref.times.front());
                const double tickSpan = (double)(tick.times.back() - tick.times.front());
                // Each step lands on a 20 us tick; whole moves agree within 0.5 % + 1 tick.
                CHECK(fabs(tickSpan - span) <= span * 0.005 + 20);
                printf("  %-22s tick engine span %9.1f ms vs %9.1f ms\n", "", tickSpan / 1000.0, span / 1000.0);
            }
        }
    }

    // State semantics.
    MotionPlanner p;
    CHECK(!p.isRunning() && p.stepIntervalUs() == 0);
    p.setMaxSpeed(1000); p.setAcceleration(1000);
    p.moveTo(10);
    CHECK(p.isRunning() && p.forward() && p.distanceToGo() == 10 && p.stepIntervalUs() > 0);
    p.setCurrentPosition(500);
    CHECK(!p.isRunning() && p.currentPosition() == 500 && p.stepIntervalUs() == 0);
    p.move(-5);
    CHECK(!p.forward() && p.targetPosition() == 495);
    return report("planner vs AccelStepper 1.64");
}
