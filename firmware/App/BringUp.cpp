// Bring-up application (milestone 1). Replaced by the ported firmware.
//
// Identifies the board on USB and on its upstream UART, answers "ID" and
// echoes other lines. It touches no other pins: motor enables and outputs
// stay in their reset state (inputs).
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
    out.print(" BUILD=" __DATE__ " " __TIME__ " UP_MS=");
    out.println((unsigned long)millis());
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
