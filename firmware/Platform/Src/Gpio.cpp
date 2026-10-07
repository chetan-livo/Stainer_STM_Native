#include "Gpio.h"
#include "stm32f4xx_hal.h"

namespace pins {
GPIO_TypeDef* port(uint8_t pin)
{
    if (pin == NC) return nullptr;
    switch (portIndex(pin)) {
    case 0: return GPIOA;
    case 1: return GPIOB;
    case 2: return GPIOC;
#ifdef GPIOD
    case 3: return GPIOD;
#endif
#ifdef GPIOE
    case 4: return GPIOE;
#endif
#ifdef GPIOF
    case 5: return GPIOF;
#endif
#ifdef GPIOG
    case 6: return GPIOG;
#endif
#ifdef GPIOH
    case 7: return GPIOH;
#endif
#ifdef GPIOI
    case 8: return GPIOI;
#endif
    default: return nullptr;
    }
}

void enableClock(uint8_t pin)
{
    if (!port(pin)) return;
    // GPIOx enable bits are consecutive from GPIOAEN in RCC->AHB1ENR.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN << portIndex(pin);
    (void)RCC->AHB1ENR; // ensure the clock is running before register access
}
}

static void configure(uint8_t pin, uint32_t mode, uint32_t pull, uint32_t alternate)
{
    GPIO_TypeDef* gpio = pins::port(pin);
    if (!gpio) return;
    pins::enableClock(pin);
    GPIO_InitTypeDef init = {};
    init.Pin = pins::mask(pin);
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = alternate;
    HAL_GPIO_Init(gpio, &init);
}

void pinMode(uint8_t pin, uint8_t mode)
{
    switch (mode) {
    case OUTPUT:            configure(pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0); break;
    case OUTPUT_OPEN_DRAIN: configure(pin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, 0); break;
    case INPUT_PULLUP:      configure(pin, GPIO_MODE_INPUT, GPIO_PULLUP, 0); break;
    case INPUT_PULLDOWN:    configure(pin, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0); break;
    case INPUT_ANALOG:      configure(pin, GPIO_MODE_ANALOG, GPIO_NOPULL, 0); break;
    case INPUT:
    default:                configure(pin, GPIO_MODE_INPUT, GPIO_NOPULL, 0); break;
    }
}

void pinAlternate(uint8_t pin, uint8_t alternate, bool openDrain, bool pullUp)
{
    configure(pin, openDrain ? GPIO_MODE_AF_OD : GPIO_MODE_AF_PP,
              pullUp ? GPIO_PULLUP : GPIO_NOPULL, alternate);
}
