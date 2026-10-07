#include "platform_error.h"
#include "stm32f4xx.h"

volatile platform_error_t platform_last_error = PLATFORM_ERROR_NONE;

void platform_error(platform_error_t code)
{
    __disable_irq();
    platform_last_error = code;
#ifdef DEBUG
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) __BKPT(0);
    for (;;) {}
#else
    NVIC_SystemReset();
#endif
}

void Error_Handler(void)
{
    platform_error(PLATFORM_ERROR_HAL);
}
