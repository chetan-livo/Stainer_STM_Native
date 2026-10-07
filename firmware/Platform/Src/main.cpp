// Native entry point.
//
// HAL, the clock tree and the platform come up in a constructor with
// priority 101, before every default-priority C++ constructor, as STM32duino
// does (premain/hw_config_init). Application globals therefore see the final
// SystemCoreClock and a running SysTick, exactly as in the Arduino build
// (e.g. the DHT driver sizes its pulse timeout from the clock). main() then
// runs the application's setup()/loop() pair.
#include "Platform.h"
#include "stm32f4xx_hal.h"

extern "C" void SystemClock_Config(void);

void platformInit()
{
    Watchdog::resetCauseCapture();
    timeInit();
    HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_PRIO_SYSTICK, 0);
}

__attribute__((constructor(101))) static void premain()
{
    HAL_Init();
    SystemClock_Config();
    platformInit();
}

int main()
{
    appSetup();
    for (;;) appLoop();
}
