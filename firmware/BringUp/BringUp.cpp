// Bring-up application (milestone 1). Replaced by the ported firmware.
//
// Identifies the board on USB and on its upstream UART and offers read-only
// bench checks. It drives no outputs: motor enables, pumps and fans stay in
// their reset state (inputs). Commands (USB or upstream UART):
//   ID            identity, clock, reset cause
//   ADC <pin>     one sample and a 64-sample average, e.g. "ADC PA4" (12-bit)
//   I2CSCAN       addresses that ACK on the board's I2C buses
// Anything else is echoed.
#include "Platform.h"
#include "stm32f4xx_hal.h"

namespace {
#if defined(Stainer_Master_PCB)
Uart upstream(PB11, PB10);        // USART3 to the ESP32
#define HAS_UPSTREAM 1
#elif defined(Nozzle_Mount_PCB) || defined(Stainer_Gantry_PCB_UART)
Uart upstream(PC11, PC10);        // UART4 to the Master
#define HAS_UPSTREAM 1
#elif defined(Magazine1_IR_PCB_UART) || defined(Magazine2_IR_PCB_UART)
Uart upstream(PB7, PB6);          // USART1 to the Gantry
#define HAS_UPSTREAM 1
#else
#define HAS_UPSTREAM 0            // Hall boards: I2C slave, USB console only
#endif

void identify(Print& out)
{
    out.print("Stainer_STM_Native BOARD=" BOARD_NAME " MCU=" MCU_NAME " SYSCLK=");
    out.print((unsigned long)(SystemCoreClock / 1000000U));
    out.print("MHz UID=");
    const uint32_t* uid = (const uint32_t*)UID_BASE;
    for (int i = 2; i >= 0; --i) out.print((unsigned long)uid[i], HEX);
    out.print(" RESET=0x");
    out.print((unsigned long)(Watchdog::lastResetFlags() >> 24), HEX);
    if (Watchdog::lastResetWasWatchdog()) out.print("(IWDG)");
    out.print(" BUILD=" __DATE__ " " __TIME__ " UP_MS=");
    out.println((unsigned long)millis());
}

#if defined(Stainer_Master_PCB)
TwoWire bus1(PB9, PB8), bus3(PC9, PA8);
TwoWire* buses[] = {&bus1, &bus3};
const char* busNames[] = {"I2C1 PB9/PB8", "I2C3 PC9/PA8"};
#elif defined(Stainer_Gantry_PCB_UART)
TwoWire bus1(PB7, PB8), bus3(PC9, PA8);
TwoWire* buses[] = {&bus1, &bus3};
const char* busNames[] = {"I2C1 PB7/PB8", "I2C3 PC9/PA8"};
#else
TwoWire** buses = nullptr;   // Hall boards are I2C slaves; Nozzle/Magazine have no master bus
const char** busNames = nullptr;
#endif
#if defined(Stainer_Master_PCB) || defined(Stainer_Gantry_PCB_UART)
constexpr size_t busCount = 2;
#else
constexpr size_t busCount = 0;
#endif

// "PA4" -> PA4; NC when malformed.
uint8_t parsePin(const char* text)
{
    if (text[0] != 'P' || text[1] < 'A' || text[1] > 'I') return NC;
    int number = 0, digits = 0;
    for (const char* p = text + 2; *p >= '0' && *p <= '9'; ++p, ++digits) number = number * 10 + (*p - '0');
    if (!digits || number > 15) return NC;
    return (uint8_t)(((text[1] - 'A') << 4) | number);
}

void adcCommand(const char* arg, Print& out)
{
    const uint8_t pin = parsePin(arg);
    if (pin == NC) { out.println("ADC ERROR use ADC <pin>, e.g. ADC PA4"); return; }
    analogReadResolution(12);
    const int first = analogRead(pin);
    uint32_t sum = 0;
    for (int i = 0; i < 64; ++i) { sum += (uint32_t)analogRead(pin); delayMicroseconds(25); }
    out.print("ADC "); out.print(arg); out.print(" RAW="); out.print(first);
    out.print(" AVG64="); out.println((unsigned long)(sum / 64U));
}

void i2cScan(Print& out)
{
    if (!busCount) { out.println("I2CSCAN no master bus on this board"); return; }
    for (size_t b = 0; b < busCount; ++b) {
        TwoWire& bus = *buses[b];
        bus.begin();
        out.print("I2CSCAN "); out.print(busNames[b]); out.print(":");
        int found = 0;
        for (uint8_t address = 1; address < 0x78; ++address) {
            bus.beginTransmission(address);
            if (bus.endTransmission() == 0) { out.print(" 0x"); out.print(address, HEX); ++found; }
        }
        if (!found) out.print(" none");
        out.println();
        bus.end();
    }
}

struct LineReader {
    char line[64];
    size_t used = 0;
    void poll(Stream& in, Print& out)
    {
        while (in.available()) {
            const char c = (char)in.read();
            if (c == '\r') continue;
            if (c != '\n') { if (used < sizeof(line) - 1) line[used++] = c; continue; }
            line[used] = '\0';
            if (!strcmp(line, "ID")) identify(out);
            else if (!strncmp(line, "ADC ", 4)) adcCommand(line + 4, out);
            else if (!strcmp(line, "I2CSCAN")) i2cScan(out);
            else if (used) { out.print("ECHO "); out.println(line); }
            used = 0;
        }
    }
};
LineReader usbReader;
#if HAS_UPSTREAM
LineReader upstreamReader;
#endif
}

void appSetup()
{
    Serial.begin();
#if HAS_UPSTREAM
    upstream.begin(115200);
    identify(upstream);
#endif
}

void appLoop()
{
    static bool announced = false;
    if (Serial && !announced) { identify(Serial); announced = true; }
    if (!Serial) announced = false;
    usbReader.poll(Serial, Serial);
#if HAS_UPSTREAM
    upstreamReader.poll(upstream, upstream);
#endif
}
