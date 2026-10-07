#include "Interrupts.h"
#include "Gpio.h"
#include "platform_irq.h"
#include "stm32f4xx_hal.h"

namespace {
void (*volatile callbacks[16])() = {};

IRQn_Type irqFor(uint8_t line)
{
    if (line <= 4) return (IRQn_Type)(EXTI0_IRQn + line);
    return line <= 9 ? EXTI9_5_IRQn : EXTI15_10_IRQn;
}

void dispatch(uint32_t first, uint32_t last)
{
    const uint32_t pending = EXTI->PR;
    for (uint32_t line = first; line <= last; ++line) {
        const uint32_t bit = 1U << line;
        if (!(pending & bit)) continue;
        EXTI->PR = bit; // write 1 to clear
        if (callbacks[line]) callbacks[line]();
    }
}
}

void attachInterrupt(uint8_t pin, void (*callback)(), uint8_t edge)
{
    GPIO_TypeDef* gpio = pins::port(pin);
    if (!gpio || !callback) return;
    const uint8_t line = pins::number(pin);
    const uint32_t bit = 1U << line;

    pins::enableClock(pin);
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    EXTI->IMR &= ~bit;
    callbacks[line] = callback;
    volatile uint32_t& exticr = SYSCFG->EXTICR[line >> 2];
    const uint32_t shift = 4U * (line & 3U);
    exticr = (exticr & ~(0xFU << shift)) | ((uint32_t)pins::portIndex(pin) << shift);
    if (edge & RISING) EXTI->RTSR |= bit; else EXTI->RTSR &= ~bit;
    if (edge & FALLING) EXTI->FTSR |= bit; else EXTI->FTSR &= ~bit;
    EXTI->PR = bit;
    EXTI->IMR |= bit;

    const IRQn_Type irq = irqFor(line);
    HAL_NVIC_SetPriority(irq, IRQ_PRIO_EXTI, 0);
    HAL_NVIC_EnableIRQ(irq);
}

void detachInterrupt(uint8_t pin)
{
    const uint8_t line = pins::number(pin);
    EXTI->IMR &= ~(1U << line);
    callbacks[line] = nullptr;
}

void noInterrupts() { __disable_irq(); }
void interrupts() { __enable_irq(); }

extern "C" {
void EXTI0_IRQHandler(void) { dispatch(0, 0); }
void EXTI1_IRQHandler(void) { dispatch(1, 1); }
void EXTI2_IRQHandler(void) { dispatch(2, 2); }
void EXTI3_IRQHandler(void) { dispatch(3, 3); }
void EXTI4_IRQHandler(void) { dispatch(4, 4); }
void EXTI9_5_IRQHandler(void) { dispatch(5, 9); }
void EXTI15_10_IRQHandler(void) { dispatch(10, 15); }
}
