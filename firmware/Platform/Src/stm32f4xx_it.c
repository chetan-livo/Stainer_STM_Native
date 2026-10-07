/* Core exception handlers. Peripheral IRQ handlers live with their drivers. */
#include "stm32f4xx_hal.h"
#include "platform_error.h"

void NMI_Handler(void) { for (;;) {} }
void HardFault_Handler(void) { platform_error(PLATFORM_ERROR_HARDFAULT); }
void MemManage_Handler(void) { platform_error(PLATFORM_ERROR_MEMMANAGE); }
void BusFault_Handler(void) { platform_error(PLATFORM_ERROR_BUSFAULT); }
void UsageFault_Handler(void) { platform_error(PLATFORM_ERROR_USAGEFAULT); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }
