#include "Watchdog.h"
#include "stm32f4xx.h"

namespace {
bool running = false;
uint32_t resetFlags = 0;
constexpr uint32_t LsiHz = 32000; // nominal; the RC varies roughly 17-47 kHz
}

namespace Watchdog {
bool begin(uint32_t timeoutMs)
{
    if (timeoutMs == 0) return false;
    // Smallest prescaler (4 .. 256) whose 12-bit reload covers the timeout.
    uint32_t prescalerCode = 0, divider = 4;
    uint64_t reload = 0;
    for (; prescalerCode <= 6; ++prescalerCode, divider <<= 1) {
        reload = ((uint64_t)timeoutMs * LsiHz) / (1000U * divider);
        if (reload <= 0x1000U) break;
    }
    if (prescalerCode > 6 || reload == 0) return false;
#ifdef DEBUG
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; // pause while halted in the debugger
#endif
    IWDG->KR = 0xCCCC;                // start (also enables LSI)
    IWDG->KR = 0x5555;                // unlock PR/RLR
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) {}
    IWDG->PR = prescalerCode;
    IWDG->RLR = (uint32_t)(reload - 1U);
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) {}
    IWDG->KR = 0xAAAA;
    running = true;
    return true;
}

void reload() { if (running) IWDG->KR = 0xAAAA; }
bool started() { return running; }

void resetCauseCapture()
{
    resetFlags = RCC->CSR & 0xFE000000U;
    RCC->CSR |= RCC_CSR_RMVF;
}
bool lastResetWasWatchdog() { return resetFlags & RCC_CSR_IWDGRSTF; }
uint32_t lastResetFlags() { return resetFlags; }
}
