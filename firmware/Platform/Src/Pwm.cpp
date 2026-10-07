#include "Analog.h"
#include "Gpio.h"
#include "PinMaps.h"

namespace {
int writeBits = 8;            // STM32duino PWM_RESOLUTION
uint32_t writeFrequency = 1000; // STM32duino PWM_FREQUENCY

bool isApb2(TIM_TypeDef* tim)
{
    return tim == TIM1 || tim == TIM9 || tim == TIM10 || tim == TIM11
#ifdef TIM8
        || tim == TIM8
#endif
        ;
}

bool isAdvanced(TIM_TypeDef* tim)
{
    return tim == TIM1
#ifdef TIM8
        || tim == TIM8
#endif
        ;
}

void enableClock(TIM_TypeDef* tim)
{
    struct Entry { TIM_TypeDef* tim; volatile uint32_t* reg; uint32_t bit; };
    static const Entry entries[] = {
        {TIM1, &RCC->APB2ENR, RCC_APB2ENR_TIM1EN},
        {TIM2, &RCC->APB1ENR, RCC_APB1ENR_TIM2EN},
        {TIM3, &RCC->APB1ENR, RCC_APB1ENR_TIM3EN},
        {TIM4, &RCC->APB1ENR, RCC_APB1ENR_TIM4EN},
        {TIM5, &RCC->APB1ENR, RCC_APB1ENR_TIM5EN},
#ifdef TIM8
        {TIM8, &RCC->APB2ENR, RCC_APB2ENR_TIM8EN},
#endif
        {TIM9, &RCC->APB2ENR, RCC_APB2ENR_TIM9EN},
        {TIM10, &RCC->APB2ENR, RCC_APB2ENR_TIM10EN},
        {TIM11, &RCC->APB2ENR, RCC_APB2ENR_TIM11EN},
#ifdef TIM12
        {TIM12, &RCC->APB1ENR, RCC_APB1ENR_TIM12EN},
        {TIM13, &RCC->APB1ENR, RCC_APB1ENR_TIM13EN},
        {TIM14, &RCC->APB1ENR, RCC_APB1ENR_TIM14EN},
#endif
    };
    for (const Entry& e : entries)
        if (e.tim == tim) { *e.reg |= e.bit; (void)*e.reg; return; }
}

uint32_t timerClock(TIM_TypeDef* tim)
{
    // Timer kernel clock is twice PCLKx when the APB prescaler is not 1.
    const bool apb2 = isApb2(tim);
    const uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    const uint32_t ppre = apb2 ? (RCC->CFGR & RCC_CFGR_PPRE2) : (RCC->CFGR & RCC_CFGR_PPRE1);
    return ppre == 0 ? pclk : pclk * 2U;
}

void setFrequency(TIM_TypeDef* tim, uint32_t hz)
{
    // Same arithmetic as STM32duino HardwareTimer::setOverflow(HERTZ_FORMAT).
    const uint32_t periodCycles = timerClock(tim) / (hz ? hz : 1U);
    const uint32_t prescaler = periodCycles / 0x10000U + 1U;
    tim->PSC = prescaler - 1U;
    tim->ARR = periodCycles / prescaler - 1U;
}

volatile uint32_t* ccr(TIM_TypeDef* tim, uint8_t channel)
{
    switch (channel) {
    case 1: return &tim->CCR1;
    case 2: return &tim->CCR2;
    case 3: return &tim->CCR3;
    default: return &tim->CCR4;
    }
}

void configureChannel(const TimPin& map)
{
    TIM_TypeDef* tim = map.tim;
    const uint32_t ch = map.channel - 1U;
    volatile uint32_t* ccmr = ch < 2 ? &tim->CCMR1 : &tim->CCMR2;
    const uint32_t shift = 8U * (ch & 1U);
    // PWM mode 1 with preload, active high.
    *ccmr = (*ccmr & ~(0xFFU << shift)) | ((6U << 4) | (1U << 3)) << shift;
    const uint32_t enable = map.complementary ? (TIM_CCER_CC1NE << (4U * ch)) : (TIM_CCER_CC1E << (4U * ch));
    const uint32_t polarity = (TIM_CCER_CC1P | TIM_CCER_CC1NP) << (4U * ch);
    tim->CCER = (tim->CCER & ~polarity) | enable;
    if (isAdvanced(tim)) tim->BDTR |= TIM_BDTR_MOE;
}

const TimPin* startPin(uint8_t pin, uint32_t hz)
{
    const TimPin* map = findPin(timPins, timPinCount, pin);
    if (!map) return nullptr;
    TIM_TypeDef* tim = map->tim;
    const bool running = tim->CR1 & TIM_CR1_CEN;
    if (!running) enableClock(tim);
    setFrequency(tim, hz);
    // Pin first set to PWM: configure the channel and route the pin.
    GPIO_TypeDef* gpio = pins::port(pin);
    const uint32_t moderShift = 2U * pins::number(pin);
    const bool alternateMode = ((gpio->MODER >> moderShift) & 3U) == 2U;
    if (!alternateMode || !(tim->CCER & ((map->complementary ? TIM_CCER_CC1NE : TIM_CCER_CC1E) << (4U * (map->channel - 1U))))) {
        configureChannel(*map);
        pinAlternate(pin, map->alternate);
    }
    if (!running) {
        tim->CR1 |= TIM_CR1_ARPE;
        tim->EGR = TIM_EGR_UG;
        tim->CR1 |= TIM_CR1_CEN;
    }
    return map;
}

void setCompare(const TimPin* map, uint32_t value, uint32_t fullScale)
{
    const uint32_t top = map->tim->ARR + 1U;
    uint32_t compare = (uint32_t)(((uint64_t)top * value) / fullScale);
    if (compare > top) compare = top;
    *ccr(map->tim, map->channel) = compare;
}
}

void analogWriteResolution(int bits)
{
    if (bits >= 1 && bits <= 16) writeBits = bits;
}

void analogWriteFrequency(uint32_t hz)
{
    if (hz) writeFrequency = hz;
}

void analogWrite(uint8_t pin, int value)
{
    const uint32_t fullScale = (1U << writeBits) - 1U;
    uint32_t v = value < 0 ? 0U : (uint32_t)value;
    if (v > fullScale) v = fullScale;
    if (const TimPin* map = startPin(pin, writeFrequency)) {
        setCompare(map, v, fullScale);
        return;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, (v * 255U / fullScale) < 128U ? LOW : HIGH);
}

bool pwmWritePercent(uint8_t pin, uint32_t frequencyHz, uint32_t percent)
{
    const TimPin* map = startPin(pin, frequencyHz);
    if (!map) return false;
    setCompare(map, percent > 100U ? 100U : percent, 100U);
    return true;
}

void pwmStop(uint8_t pin)
{
    if (const TimPin* map = findPin(timPins, timPinCount, pin)) {
        *ccr(map->tim, map->channel) = 0;
        const uint32_t ch = map->channel - 1U;
        map->tim->CCER &= ~((map->complementary ? TIM_CCER_CC1NE : TIM_CCER_CC1E) << (4U * ch));
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}
