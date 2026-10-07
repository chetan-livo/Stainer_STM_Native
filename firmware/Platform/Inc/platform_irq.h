#pragma once
/* NVIC preemption priorities (group 4: 16 levels, no subpriority).
 * Lower number = more urgent. Keep every peripheral driver on this table. */
#define IRQ_PRIO_SYSTICK   0U  /* millis()/micros() timebase, HAL timeouts */
#define IRQ_PRIO_STEP      1U  /* STEP pulse timer: bounded, a few microseconds */
#define IRQ_PRIO_UART      2U  /* 1-byte RX register: 20 us budget at 500 kbaud */
#define IRQ_PRIO_USB       3U
#define IRQ_PRIO_I2C       4U
#define IRQ_PRIO_ADC_DMA   5U
#define IRQ_PRIO_EXTI      6U
#define IRQ_PRIO_LOWEST   15U
