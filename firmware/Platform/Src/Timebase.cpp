#include "Timebase.h"
#include "stm32f4xx_hal.h"

void timeInit()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t cyclesPerMicrosecond() { return SystemCoreClock / 1000000U; }

uint32_t millis() { return HAL_GetTick(); }

uint32_t micros()
{
    // Read the tick and SysTick phase consistently: if the tick advanced
    // between reads, the counter reloaded and the second pair is valid.
    uint32_t ms0 = HAL_GetTick();
    uint32_t val0 = SysTick->VAL;
    uint32_t ms1 = HAL_GetTick();
    uint32_t val1 = SysTick->VAL;
    const uint32_t load = SysTick->LOAD + 1U;
    if (ms1 != ms0) { ms0 = ms1; val0 = val1; }
    return ms0 * 1000U + ((load - val0) * 1000U) / load;
}

void delay(uint32_t ms)
{
    const uint32_t start = millis();
    while (millis() - start < ms) {}
}

void delayMicroseconds(uint32_t us)
{
    const uint32_t start = cycles();
    const uint32_t wait = us * cyclesPerMicrosecond();
    while (cycles() - start < wait) {}
}
