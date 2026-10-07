#include "Uart.h"
#include "Gpio.h"
#include "platform_irq.h"
#include "stm32f4xx_hal.h"

namespace {
struct UartPins { USART_TypeDef* usart; uint8_t tx, rx, alternate; };

// TX/RX pairs used by the Livo boards, checked against RM0390 / RM0090 /
// RM0368 alternate-function tables. PC10/PC11 resolve to UART4 because the
// Master also uses USART3 (PB10/PB11).
const UartPins pinTable[] = {
    {USART1, PA9,  PA10, GPIO_AF7_USART1},
    {USART1, PB6,  PB7,  GPIO_AF7_USART1},
    {USART2, PA2,  PA3,  GPIO_AF7_USART2},
    {USART2, PD5,  PD6,  GPIO_AF7_USART2},
#ifdef UART4
    {UART4,  PC10, PC11, GPIO_AF8_UART4},
    {UART4,  PA0,  PA1,  GPIO_AF8_UART4},
#endif
#ifdef USART3
    {USART3, PB10, PB11, GPIO_AF7_USART3},
    {USART3, PD8,  PD9,  GPIO_AF7_USART3},
    {USART3, PC10, PC11, GPIO_AF7_USART3},
#endif
#ifdef UART5
    {UART5,  PC12, PD2,  GPIO_AF8_UART5},
#if defined(STM32F446xx)
    {UART5,  PE8,  PE7,  GPIO_AF8_UART5},
#endif
#endif
    {USART6, PC6,  PC7,  GPIO_AF8_USART6},
#ifdef GPIOG
    {USART6, PG14, PG9,  GPIO_AF8_USART6},
#endif
};

enum { SLOT_USART1, SLOT_USART2, SLOT_USART3, SLOT_UART4, SLOT_UART5, SLOT_USART6, SLOT_COUNT };
Uart* active[SLOT_COUNT] = {};

int slot(USART_TypeDef* u)
{
    if (u == USART1) return SLOT_USART1;
    if (u == USART2) return SLOT_USART2;
#ifdef USART3
    if (u == USART3) return SLOT_USART3;
#endif
#ifdef UART4
    if (u == UART4) return SLOT_UART4;
#endif
#ifdef UART5
    if (u == UART5) return SLOT_UART5;
#endif
    if (u == USART6) return SLOT_USART6;
    return -1;
}

IRQn_Type irqOf(USART_TypeDef* u)
{
    if (u == USART1) return USART1_IRQn;
    if (u == USART2) return USART2_IRQn;
#ifdef USART3
    if (u == USART3) return USART3_IRQn;
#endif
#ifdef UART4
    if (u == UART4) return UART4_IRQn;
#endif
#ifdef UART5
    if (u == UART5) return UART5_IRQn;
#endif
    return USART6_IRQn;
}

void enableClock(USART_TypeDef* u, bool on)
{
    volatile uint32_t* reg = nullptr;
    uint32_t bit = 0;
    if (u == USART1) { reg = &RCC->APB2ENR; bit = RCC_APB2ENR_USART1EN; }
    else if (u == USART2) { reg = &RCC->APB1ENR; bit = RCC_APB1ENR_USART2EN; }
#ifdef USART3
    else if (u == USART3) { reg = &RCC->APB1ENR; bit = RCC_APB1ENR_USART3EN; }
#endif
#ifdef UART4
    else if (u == UART4) { reg = &RCC->APB1ENR; bit = RCC_APB1ENR_UART4EN; }
#endif
#ifdef UART5
    else if (u == UART5) { reg = &RCC->APB1ENR; bit = RCC_APB1ENR_UART5EN; }
#endif
    else if (u == USART6) { reg = &RCC->APB2ENR; bit = RCC_APB2ENR_USART6EN; }
    if (!reg) return;
    if (on) { *reg |= bit; (void)*reg; } else { *reg &= ~bit; }
}

uint32_t clockOf(USART_TypeDef* u)
{
    return (u == USART1 || u == USART6) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
}
}

Uart::Uart(uint8_t rxPin, uint8_t txPin, USART_TypeDef* instance) : rxPin_(rxPin), txPin_(txPin)
{
    for (const UartPins& entry : pinTable) {
        if (entry.tx == txPin && entry.rx == rxPin && (!instance || entry.usart == instance)) {
            usart_ = entry.usart;
            alternate_ = entry.alternate;
            break;
        }
    }
}

bool Uart::begin(uint32_t baud, UartFraming framing)
{
    const int index = usart_ ? slot(usart_) : -1;
    if (index < 0 || baud == 0) return false;
    if (started_) end();

    enableClock(usart_, true);
    pinAlternate(txPin_, alternate_, false, true);
    pinAlternate(rxPin_, alternate_, false, true);

    usart_->CR1 = 0;
    usart_->CR2 = 0;
    usart_->CR3 = 0;
    // Oversampling by 16: BRR = fCK / baud, rounded (mantissa.fraction/16).
    usart_->BRR = (clockOf(usart_) + baud / 2U) / baud;
    rx_.clear();
    tx_.clear();
    active[index] = this;
    uint32_t cr1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    if (framing == SERIAL_8E1) cr1 |= USART_CR1_M | USART_CR1_PCE; // 9 bits = 8 data + parity
    usart_->CR1 = cr1 | USART_CR1_UE;

    const IRQn_Type irq = irqOf(usart_);
    HAL_NVIC_SetPriority(irq, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(irq);
    started_ = true;
    return true;
}

void Uart::end()
{
    if (!started_) return;
    flush();
    HAL_NVIC_DisableIRQ(irqOf(usart_));
    usart_->CR1 = 0;
    active[slot(usart_)] = nullptr;
    started_ = false;
}

size_t Uart::write(uint8_t c)
{
    if (!started_) return 0;
    while (!tx_.push(c)) {} // the TXE interrupt is draining the buffer
    usart_->CR1 |= USART_CR1_TXEIE;
    return 1;
}

size_t Uart::write(const uint8_t* buffer, size_t size)
{
    for (size_t i = 0; i < size; ++i) write(buffer[i]);
    return started_ ? size : 0;
}

void Uart::flush()
{
    if (!started_) return;
    while (!tx_.empty()) {}
    while (!(usart_->SR & USART_SR_TC)) {}
}

void Uart::irq()
{
    const uint32_t sr = usart_->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // Reading SR then DR clears RXNE and the error flags.
        const uint8_t value = (uint8_t)(usart_->DR & 0xFFU);
        if (sr & USART_SR_ORE) ++overruns_;
        if (!rx_.push(value)) ++overruns_;
    }
    if ((sr & USART_SR_TXE) && (usart_->CR1 & USART_CR1_TXEIE)) {
        const int next = tx_.pop();
        if (next >= 0) usart_->DR = (uint8_t)next;
        else usart_->CR1 &= ~USART_CR1_TXEIE;
    }
}

extern "C" {
void USART1_IRQHandler(void) { if (active[SLOT_USART1]) active[SLOT_USART1]->irq(); }
void USART2_IRQHandler(void) { if (active[SLOT_USART2]) active[SLOT_USART2]->irq(); }
#ifdef USART3
void USART3_IRQHandler(void) { if (active[SLOT_USART3]) active[SLOT_USART3]->irq(); }
#endif
#ifdef UART4
void UART4_IRQHandler(void) { if (active[SLOT_UART4]) active[SLOT_UART4]->irq(); }
#endif
#ifdef UART5
void UART5_IRQHandler(void) { if (active[SLOT_UART5]) active[SLOT_UART5]->irq(); }
#endif
void USART6_IRQHandler(void) { if (active[SLOT_USART6]) active[SLOT_USART6]->irq(); }
}
