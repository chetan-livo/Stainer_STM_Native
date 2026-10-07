// Native entry point: HAL, clock tree and platform, then the application's
// setup()/loop() pair, keeping the Arduino firmware's control structure.
#include "Platform.h"
#include "stm32f4xx_hal.h"

extern "C" void SystemClock_Config(void);

void platformInit()
{
    Watchdog::resetCauseCapture();
    timeInit();
    HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_PRIO_SYSTICK, 0);
}

int main()
{
    HAL_Init();
    SystemClock_Config();
    platformInit();
    appSetup();
    for (;;) appLoop();
}
