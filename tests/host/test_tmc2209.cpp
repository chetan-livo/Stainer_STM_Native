// Equivalence test: Devices/Tmc2209 against TMCStepper 0.7.3 itself.
//
// Both drivers run the Livo firmware's real call sequences (TMCMotor::
// setupMotor, TMCModule::setup, start/stop current switching, diagnostics
// reads) on recording mock UARTs. Every transmitted byte and every value
// read back must match. TMCStepper is compiled from the locally installed
// Arduino library (MIT); see tools/run_host_tests.ps1.
#include "Tmc2209.h"
#include "check.h"
#include <TMCStepper.h>
// The test code below uses the Platform API; TMCStepper saw the renamed stubs.
#undef Stream
#undef millis
#undef micros
#undef delay
#undef delayMicroseconds
#undef pinMode
#undef digitalWrite
#undef digitalRead
#undef yield
#include <deque>
#include <string>
#include <vector>

// ---- fake time: one clock for both the reference library and Platform -------
#include "Timebase.h"
static unsigned long fakeTicks = 0;               // 1 tick = 0.25 ms, advances when polled
static unsigned long nowMs() { return ++fakeTicks / 4; }
unsigned long arduino_millis(void) { return nowMs(); }
unsigned long arduino_micros(void) { return fakeTicks * 250; }
void arduino_delay(unsigned long ms) { fakeTicks += ms * 4; }
void arduino_delayMicroseconds(unsigned int) {}
void arduino_pinMode(uint8_t, uint8_t) {}
void arduino_digitalWrite(uint8_t, uint8_t) {}
int arduino_digitalRead(uint8_t) { return 0; }
void arduino_yield(void) {}
SPIClass SPI;
uint32_t millis() { return (uint32_t)nowMs(); }
uint32_t micros() { return (uint32_t)(fakeTicks * 250); }
void delay(uint32_t ms) { fakeTicks += ms * 4; }
void delayMicroseconds(uint32_t) {}

// ---- simulated single-wire TMC2209 bus ---------------------------------------
// Echoes every byte (TX and RX share the wire) and answers read requests.
struct Bus {
    std::vector<uint8_t> sent;
    std::deque<uint8_t> rx;
    std::vector<uint8_t> pending;
    enum Mode { Good, CorruptOnce, Silent } mode = Good;
    uint32_t value = 0x21000040; // IOIN-like: version 0x21 in the top byte

    void put(uint8_t c)
    {
        sent.push_back(c);
        rx.push_back(c);                       // echo
        pending.push_back(c);
        if (pending.size() == 4 && pending[0] == 0x05 && !(pending[2] & 0x80)) {
            const uint8_t reg = pending[2];
            uint8_t reply[8] = {0x05, 0xFF, reg, (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                                (uint8_t)(value >> 8), (uint8_t)value, 0};
            reply[7] = Tmc2209::calcCRC(reply, 7);
            if (mode == CorruptOnce) { reply[5] ^= 0x10; mode = Good; }
            if (mode != Silent) for (uint8_t b : reply) rx.push_back(b);
            pending.clear();
        } else if (pending.size() == 8) {
            pending.clear();
        }
    }
    int get() { if (rx.empty()) return -1; int c = rx.front(); rx.pop_front(); return c; }
};

struct RefPort : ArduinoStream {
    Bus& bus; explicit RefPort(Bus& b) : bus(b) {}
    int available() override { return (int)bus.rx.size(); }
    int read() override { return bus.get(); }
    int peek() override { return bus.rx.empty() ? -1 : bus.rx.front(); }
    size_t write(uint8_t c) override { bus.put(c); return 1; }
};
struct NativePort : ::Stream {
    Bus& bus; explicit NativePort(Bus& b) : bus(b) {}
    int available() override { return (int)bus.rx.size(); }
    int read() override { return bus.get(); }
    int peek() override { return bus.rx.empty() ? -1 : bus.rx.front(); }
    size_t write(uint8_t c) override { bus.put(c); return 1; }
    using Print::write;
};

// The Livo firmware's TMC call sequences (TMCMotor.cpp / TMCModule.cpp).
template <typename Driver>
void firmwareSequence(Driver& d, uint16_t irun, uint16_t ihold, uint16_t microsteps)
{
    d.begin();
    d.GCONF(0x01C0);                                   // multistep_filt|pdn_disable|mstep_reg_select
    d.rms_current(500, 100.0f / 500.0f);              // TMCMotor defaults before TMCModule
    d.iholddelay(2);
    d.CHOPCONF(0x80420346);                            // TMCMotor::setupMotor chopConfData
    d.rms_current(irun, 100.0f / irun);               // TMCModule::setup: setRMSCurrentIRUN
    d.rms_current(irun, (float)ihold / irun);         //                   setRMSCurrentIHOLD
    d.mstep_reg_select(true);                          // setMicrostepsTMC
    d.microsteps(microsteps);
    d.rms_current(ihold);                              // activateHoldCurrent (one-argument form)
    d.rms_current(2500, 0.5f);                         // clamps CS at 31
    // rms_current(0) is not compared: TMCStepper converts -1.0 to uint8_t
    // there (undefined behaviour; x86 and ARM differ). See armZeroCurrent().
}

std::string hex(const std::vector<uint8_t>& v)
{
    std::string s; char t[4];
    for (uint8_t b : v) { snprintf(t, sizeof t, "%02X", b); s += t; }
    return s;
}

// IRUN 0 (unreachable in the firmware: TMCModule rejects it) must match the
// ARM build of TMCStepper: __aeabi_d2uiz saturates -1.0 to 0, so vsense is set
// and IRUN = IHOLD = 0.
void armZeroCurrent()
{
    Bus bus;
    NativePort port(bus);
    Tmc2209 nat(port, 0.11f, 0);
    nat.rms_current(0, 0.0f);
    CHECK(nat.irun() == 0 && nat.ihold() == 0);
    CHECK(bus.sent.size() == 3 * 8);                   // CHOPCONF (vsense), IHOLD_IRUN x2
    CHECK(bus.sent[2] == (0x6C | 0x80) && bus.sent[3] == 0x10 && bus.sent[4] == 0x02); // 0x1002xxxx
}

int main()
{
    armZeroCurrent();
    struct Config { uint8_t address; uint16_t irun, ihold, ms; } configs[] = {
        {0, 800, 200, 128}, {1, 800, 100, 128}, {2, 1200, 200, 32}, {3, 800, 200, 32},
        {0, 1000, 200, 16}, {1, 1000, 100, 16}, {2, 800, 100, 1},   {3, 300, 100, 256},
    };
    for (const Config& c : configs) {
        Bus refBus, natBus;
        RefPort refPort(refBus);
        NativePort natPort(natBus);
        TMC2209Stepper ref(&refPort, 0.11f, c.address);
        Tmc2209 nat(natPort, 0.11f, c.address);
        firmwareSequence(ref, c.irun, c.ihold, c.ms);
        firmwareSequence(nat, c.irun, c.ihold, c.ms);
        CHECK(!refBus.sent.empty());
        if (hex(refBus.sent) != hex(natBus.sent)) {
            printf("datagrams differ for address %u irun %u ihold %u ms %u\n  ref %s\n  nat %s\n",
                   c.address, c.irun, c.ihold, c.ms, hex(refBus.sent).c_str(), hex(natBus.sent).c_str());
            CHECK(false);
        } else {
            CHECK(true);
        }
        CHECK(ref.IHOLD_IRUN() == nat.IHOLD_IRUN());
        CHECK(ref.hold_multiplier() == nat.hold_multiplier());

        // Diagnostic reads: good reply, CRC error then retry, no reply.
        for (Bus::Mode mode : {Bus::Good, Bus::CorruptOnce, Bus::Silent}) {
            refBus.mode = natBus.mode = mode;
            refBus.sent.clear(); natBus.sent.clear();
            const uint32_t r1 = ref.DRV_STATUS(), n1 = nat.DRV_STATUS();
            const bool refCrc = ref.CRCerror, natCrc = nat.CRCerror;
            const uint8_t r2 = ref.version(), n2 = nat.version();
            const uint32_t r3 = ref.GCONF(), n3 = nat.GCONF();
            const uint32_t r4 = ref.CHOPCONF(), n4 = nat.CHOPCONF();
            CHECK(r1 == n1 && r2 == n2 && r3 == n3 && r4 == n4);
            CHECK(refCrc == natCrc);
            CHECK(hex(refBus.sent) == hex(natBus.sent));
            if (mode == Bus::Good) CHECK(n1 == 0x21000040 && n2 == 0x21);
            if (mode == Bus::Silent) CHECK(n1 == 0 && natCrc);
        }
    }
    return report("tmc2209 vs TMCStepper 0.7.3");
}
