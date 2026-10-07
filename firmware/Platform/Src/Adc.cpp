#include "Analog.h"
#include "Gpio.h"
#include "PinMaps.h"
#include "Timebase.h"

// DMA allocation (RM0090/RM0390 DMA2 request mapping):
//   ADC1 -> DMA2 Stream 4, channel 0
//   ADC3 -> DMA2 Stream 1, channel 2
namespace {
constexpr uint8_t MaxChannels = 16;

struct ScanAdc {
    ADC_TypeDef* adc;
    DMA_Stream_TypeDef* stream;
    uint32_t dmaChannel;
    volatile uint32_t* tcFlagReg;   // DMA2->HISR / LISR
    volatile uint32_t* tcClearReg;  // DMA2->HIFCR / LIFCR
    uint32_t tcFlag;
    uint8_t pins[MaxChannels];
    uint8_t channels[MaxChannels];
    volatile uint16_t samples[MaxChannels];
    uint8_t count;
};

ScanAdc scan1 = {ADC1, DMA2_Stream4, 0U, &DMA2->HISR, &DMA2->HIFCR, DMA_HISR_TCIF4, {}, {}, {}, 0};
#ifdef ADC3
ScanAdc scan3 = {ADC3, DMA2_Stream1, 2U, &DMA2->LISR, &DMA2->LIFCR, DMA_LISR_TCIF1, {}, {}, {}, 0};
#endif
int readBits = 10; // Arduino default until analogReadResolution()

ScanAdc* scanFor(ADC_TypeDef* adc)
{
    if (adc == ADC1) return &scan1;
#ifdef ADC3
    if (adc == ADC3) return &scan3;
#endif
    return nullptr;
}

void enableClocks(ADC_TypeDef* adc)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    if (adc == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
#ifdef ADC3
    if (adc == ADC3) RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
#endif
    (void)RCC->APB2ENR;
    // Common prescaler: PCLK2 / 4 (STM32duino ADC_CLOCK_SYNC_PCLK_DIV4).
#if defined(ADC123_COMMON)
    ADC_Common_TypeDef* common = ADC123_COMMON;
#else
    ADC_Common_TypeDef* common = ADC1_COMMON; // F401: ADC1 only
#endif
    common->CCR = (common->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
}

void setSampleTime(ADC_TypeDef* adc, uint8_t channel)
{
    // 15 cycles = SMPx code 001 (STM32duino ADC_SAMPLETIME_15CYCLES on F4).
    if (channel >= 10) {
        const uint32_t shift = 3U * (channel - 10U);
        adc->SMPR1 = (adc->SMPR1 & ~(7U << shift)) | (1U << shift);
    } else {
        const uint32_t shift = 3U * channel;
        adc->SMPR2 = (adc->SMPR2 & ~(7U << shift)) | (1U << shift);
    }
}

void restart(ScanAdc& s)
{
    ADC_TypeDef* adc = s.adc;
    adc->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_CONT);
    s.stream->CR &= ~DMA_SxCR_EN;
    while (s.stream->CR & DMA_SxCR_EN) {}

    adc->CR1 = ADC_CR1_SCAN;                // 12-bit, scan mode
    adc->SQR1 = (uint32_t)(s.count - 1U) << ADC_SQR1_L_Pos;
    adc->SQR2 = 0; adc->SQR3 = 0;
    for (uint8_t i = 0; i < s.count; ++i) {
        const uint32_t ch = s.channels[i];
        if (i < 6) adc->SQR3 |= ch << (5U * i);
        else if (i < 12) adc->SQR2 |= ch << (5U * (i - 6U));
        else adc->SQR1 |= ch << (5U * (i - 12U));
        setSampleTime(adc, s.channels[i]);
    }

    *s.tcClearReg = s.tcFlag | (s.tcFlag >> 1) | (s.tcFlag >> 2) | (s.tcFlag >> 3) | (s.tcFlag >> 5); // TC, HT, TE, DME, FE
    s.stream->PAR = (uint32_t)&adc->DR;
    s.stream->M0AR = (uint32_t)s.samples;
    s.stream->NDTR = s.count;
    s.stream->FCR = 0;                      // direct mode
    s.stream->CR = (s.dmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 |
                   DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
    s.stream->CR |= DMA_SxCR_EN;

    adc->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    delayMicroseconds(3);                   // tSTAB after ADON
    adc->CR2 |= ADC_CR2_SWSTART;
}

void waitForFreshScan(ScanAdc& s)
{
    *s.tcClearReg = s.tcFlag;
    const uint32_t start = micros();
    while (!(*s.tcFlagReg & s.tcFlag) && micros() - start < 1000U) {}
}

int scale(uint32_t raw)
{
    if (readBits == 12) return (int)raw;
    return readBits > 12 ? (int)(raw << (readBits - 12)) : (int)(raw >> (12 - readBits));
}
}

void analogReadResolution(int bits)
{
    if (bits >= 1 && bits <= 16) readBits = bits;
}

int analogRead(uint8_t pin)
{
    const AdcPin* map = findPin(adcPins, adcPinCount, pin);
    ScanAdc* s = map ? scanFor(map->adc) : nullptr;
    if (!s) return 0;

    GPIO_TypeDef* gpio = pins::port(pin);
    const uint32_t moderShift = 2U * pins::number(pin);
    if (((gpio->MODER >> moderShift) & 3U) != 3U) pinMode(pin, INPUT_ANALOG);

    for (uint8_t i = 0; i < s->count; ++i)
        if (s->pins[i] == pin) return scale(s->samples[i]);

    if (s->count >= MaxChannels) return 0;
    enableClocks(s->adc);
    s->pins[s->count] = pin;
    s->channels[s->count] = map->channel;
    s->samples[s->count] = 0;
    ++s->count;
    restart(*s);
    waitForFreshScan(*s);
    return scale(s->samples[s->count - 1]);
}
