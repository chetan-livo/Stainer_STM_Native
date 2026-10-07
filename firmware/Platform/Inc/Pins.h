#pragma once
#include <stdint.h>
#include "stm32f4xx.h"

// Pin identifiers: (port index << 4) | pin number, so PA0 = 0x00, PB0 = 0x10,
// PI15 = 0x8F. They fit uint8_t, which keeps the existing board headers
// (`const uint8_t X = PE11;`) unchanged. PX_n spellings are aliases.
enum : uint8_t {
#define LIVO_PORT_PINS(P, n) \
    P##0 = (n << 4) | 0, P##1, P##2, P##3, P##4, P##5, P##6, P##7, \
    P##8, P##9, P##10, P##11, P##12, P##13, P##14, P##15
    LIVO_PORT_PINS(PA, 0), LIVO_PORT_PINS(PB, 1), LIVO_PORT_PINS(PC, 2),
    LIVO_PORT_PINS(PD, 3), LIVO_PORT_PINS(PE, 4), LIVO_PORT_PINS(PF, 5),
    LIVO_PORT_PINS(PG, 6), LIVO_PORT_PINS(PH, 7), LIVO_PORT_PINS(PI, 8),
#undef LIVO_PORT_PINS
    NC = 0xFF
};

#define LIVO_PIN_ALIASES(P) \
    constexpr uint8_t P##_0 = P##0, P##_1 = P##1, P##_2 = P##2, P##_3 = P##3, \
        P##_4 = P##4, P##_5 = P##5, P##_6 = P##6, P##_7 = P##7, P##_8 = P##8, \
        P##_9 = P##9, P##_10 = P##10, P##_11 = P##11, P##_12 = P##12, \
        P##_13 = P##13, P##_14 = P##14, P##_15 = P##15;
LIVO_PIN_ALIASES(PA) LIVO_PIN_ALIASES(PB) LIVO_PIN_ALIASES(PC)
LIVO_PIN_ALIASES(PD) LIVO_PIN_ALIASES(PE) LIVO_PIN_ALIASES(PF)
LIVO_PIN_ALIASES(PG) LIVO_PIN_ALIASES(PH) LIVO_PIN_ALIASES(PI)
#undef LIVO_PIN_ALIASES

namespace pins {
inline constexpr uint8_t portIndex(uint8_t pin) { return pin >> 4; }
inline constexpr uint16_t mask(uint8_t pin) { return (uint16_t)(1U << (pin & 0x0F)); }
inline constexpr uint8_t number(uint8_t pin) { return pin & 0x0F; }
// nullptr for NC or a port this MCU does not have.
GPIO_TypeDef* port(uint8_t pin);
void enableClock(uint8_t pin);
}
