#include "StepEngine.h"
#include <math.h>
#include "Gpio.h"
#include "Timebase.h"
#include "platform_irq.h"
#include "stm32f4xx_hal.h"

namespace {
#ifdef LIVO_STEP_ENGINE
bool timerStarted = false;
IsrStepper* steppers[StepEngine::MaxSteppers] = {};
uint8_t stepperCount = 0;
#endif
volatile uint32_t tickCount = 0;

// LOOPSTAT window, written only by the step interrupt.
volatile uint32_t isrMaxCycles = 0, isrMaxPeriodCycles = 0, isrLastEntry = 0;
volatile uint64_t isrTotalCycles = 0;
volatile uint32_t isrSteps = 0;
volatile bool isrPeriodValid = false;
[[maybe_unused]] uint32_t statsWindowStartMs = 0; // unused where TIM7 is absent (F401)

#ifdef LIVO_STEP_ENGINE
void onTick() {
    const uint32_t entry = DWT->CYCCNT;
    if (isrPeriodValid) {
        const uint32_t period = entry - isrLastEntry;
        if (period > isrMaxPeriodCycles) isrMaxPeriodCycles = period;
    }
    isrLastEntry = entry;
    isrPeriodValid = true;

    const uint32_t tick = ++tickCount;
    IsrStepper* due[StepEngine::MaxSteppers];
    uint8_t dueCount = 0;
    for (uint8_t i = 0; i < stepperCount; ++i)
        if (steppers[i]->dueForStep(tick)) due[dueCount++] = steppers[i];

    if (dueCount) {
        // DIR was written in dueForStep(), before every rising edge.
        for (uint8_t i = 0; i < dueCount; ++i) due[i]->raiseStep();
        const uint32_t raised = DWT->CYCCNT;
        for (uint8_t i = 0; i < dueCount; ++i) due[i]->planNextStep();
        // AccelStepper's default minimum STEP high time is 1 us.
        const uint32_t pulseCycles = SystemCoreClock / 1000000UL;
        while (DWT->CYCCNT - raised < pulseCycles) {}
        for (uint8_t i = 0; i < dueCount; ++i) due[i]->lowerStep();
        isrSteps += dueCount;
    }

    const uint32_t spent = DWT->CYCCNT - entry;
    isrTotalCycles += spent;
    if (spent > isrMaxCycles) isrMaxCycles = spent;
}

void startTimer() {
    // TIM7 on APB1: kernel clock is 2 x PCLK1 (APB1 prescaler 4 on F446/F407).
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    (void)RCC->APB1ENR;
    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t clock = (RCC->CFGR & RCC_CFGR_PPRE1) ? pclk1 * 2U : pclk1;
    TIM7->PSC = 0;
    TIM7->ARR = clock / StepEngine::TickHz - 1U;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0;
    TIM7->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM7_IRQn, IRQ_PRIO_STEP, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    statsWindowStartMs = millis();
    timerStarted = true;
    TIM7->CR1 = TIM_CR1_CEN;
}
#endif
} // namespace

#ifdef LIVO_STEP_ENGINE
extern "C" void TIM7_IRQHandler(void)
{
    TIM7->SR = ~TIM_SR_UIF;
    onTick();
}
#endif

namespace StepEngine {
uint32_t ticks() { return tickCount; }

Guard::Guard() {
#ifdef LIVO_STEP_ENGINE
    wasEnabled = NVIC_GetEnableIRQ(TIM7_IRQn);
    NVIC_DisableIRQ(TIM7_IRQn);
    __DSB(); __ISB();
#else
    wasEnabled = false;
#endif
}

Guard::~Guard() {
#ifdef LIVO_STEP_ENGINE
    if (wasEnabled) NVIC_EnableIRQ(TIM7_IRQn);
#endif
}

void reportStats(Print& out) {
#ifdef LIVO_STEP_ENGINE
    uint32_t maxCycles, maxPeriod, steps; uint64_t total;
    {
        Guard guard;
        maxCycles = isrMaxCycles; maxPeriod = isrMaxPeriodCycles; total = isrTotalCycles; steps = isrSteps;
        isrMaxCycles = 0; isrMaxPeriodCycles = 0; isrTotalCycles = 0; isrSteps = 0; isrPeriodValid = false;
    }
    const uint32_t cyclesPerUs = SystemCoreClock / 1000000UL;
    const uint32_t windowMs = millis() - statsWindowStartMs;
    statsWindowStartMs = millis();
    uint8_t active = 0;
    for (uint8_t i = 0; i < stepperCount; ++i) if (steppers[i]->active()) ++active;
    out.print(" STEP_ISR_MOTORS:"); out.print(active);
    out.print(" ISR_STEPS:"); out.print(steps);
    out.print(" ISR_MAX_US:"); out.print(maxCycles / cyclesPerUs);
    out.print(" ISR_MAX_PERIOD_US:"); out.print(maxPeriod / cyclesPerUs);
    out.print(" ISR_LOAD_PCT:");
    out.print(windowMs ? (float)total * 100.0f / ((float)windowMs * (SystemCoreClock / 1000UL)) : 0.0f, 1);
#else
    (void)out;
#endif
}
} // namespace StepEngine

bool IsrStepper::attach(uint8_t stepPin, uint8_t dirPin) {
#ifdef LIVO_STEP_ENGINE
    if (stepPort_) return true;
    if (stepperCount >= StepEngine::MaxSteppers) return false;
    stepPort_ = pins::port(stepPin);
    stepMask_ = pins::mask(stepPin);
    dirPort_ = pins::port(dirPin);
    dirMask_ = pins::mask(dirPin);
    if (!stepPort_ || !dirPort_) { stepPort_ = nullptr; return false; }
    setPinsInverted(dirInverted_, stepInverted_);
    {
        StepEngine::Guard guard;
        steppers[stepperCount++] = this;
    }
    if (!timerStarted) startTimer();
    return true;
#else
    (void)stepPin; (void)dirPin;
    return false;
#endif
}

void IsrStepper::setPinsInverted(bool directionInvert, bool stepInvert) {
    dirInverted_ = directionInvert;
    stepInverted_ = stepInvert;
    stepHigh_ = stepInvert ? (stepMask_ << 16) : stepMask_;
    stepLow_ = stepInvert ? stepMask_ : (stepMask_ << 16);
}

bool IsrStepper::dueForStep(uint32_t tick) {
    if (!active_ || !planner_.stepIntervalUs() || !driverEnabled_ || held_ || !leased_ ||
        tick - leaseTick_ >= StepEngine::LeaseTicks) return false;
    if (!countdown_.tick(StepEngine::TickUs)) return false;
    // DIR before the rising STEP edge (written here, raised after all motors).
    const bool high = planner_.forward() != dirInverted_;
    dirPort_->BSRR = high ? dirMask_ : (dirMask_ << 16);
    return true;
}

void IsrStepper::planNextStep() {
    planner_.stepTaken();
    countdown_.scheduleNext(planner_.stepIntervalUs());
}
