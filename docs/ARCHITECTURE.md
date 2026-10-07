# Architecture

## Layers

```
App/        application: command parsing, motion, bed scheduling, diagnostics
Boards/     per-board pins and peripheral assignments
Platform/   native drivers with an Arduino-shaped API (Print/Stream, pinMode, millis, ...)
Drivers/    ST HAL/LL + CMSIS (unmodified)
Middlewares ST USB device library, CDC class (unmodified)
```

The platform keeps the API the existing firmware already calls (`Serial.print`, `digitalWrite`, `millis`, `Uart::begin`). The application classes can therefore be moved across mostly unchanged and verified against the Arduino build. Each driver is native: it owns its peripheral registers and interrupts, with no hidden re-initialisation per call.

## One project, many MCUs

| Item | How it varies |
|---|---|
| Device header / HAL | `STM32F446xx`, `STM32F407xx` or `STM32F401xC` define per configuration |
| Startup file | `Platform/Startup/startup_*.s`; the other two are excluded per configuration |
| Linker script | `Platform/LinkerScripts/*.ld`, selected per configuration |
| Board code | `Boards/<Board>/`; other boards' folders are excluded per configuration |
| Board identity | The same board define as the Arduino sketch (`Stainer_Master_PCB`, ...) |

`tools/generate_cubeide_project.py` is the single source of these settings.

## Clock trees

These are identical to the STM32duino generic variants the boards run today, using HSI only with no crystal assumed:

| MCU | SYSCLK | APB1 / APB2 | USB 48 MHz |
|---|---|---|---|
| F446ZE | 180 MHz (PLLR, overdrive) | 45 / 90 MHz | PLLSAI-P |
| F407VE | 168 MHz | 42 / 84 MHz | PLLQ |
| F401RC | 84 MHz | 42 / 84 MHz | PLLQ |

## Conventions

- **Pins.** `PA0` = 0x00 through `PI15` = 0x8F (port << 4 | pin), stored in `uint8_t`. `PX_n` names are aliases. The existing board headers compile unchanged.
- **Interrupt priorities** (`platform_irq.h`, group 4):

  | Priority | Source |
  |---|---|
  | 0 | SysTick |
  | 1 | step timer |
  | 2 | UART |
  | 3 | USB |
  | 4 | I2C |
  | 5 | ADC/DMA |
  | 6 | EXTI |

  New drivers must use this table.
- **Serial buffers.** UART RX and TX are 1 KiB each; USB is 1 KiB RX and 2 KiB TX. Writes never block for a disconnected USB host. UART writes block only while the transmit buffer is full.
- **Files that would shadow C headers** on Windows' case-insensitive filesystem are avoided (`Timebase.h`, not `Time.h`; `WString.h`, not `String.h`).
- **Errors.** `platform_error()` records the code in `platform_last_error`. Debug builds stop at a breakpoint when a debugger is attached; Release builds reset.
- **C++.** gnu++17, no exceptions, no RTTI, no thread-safe statics. Heap via newlib-nano `_sbrk` (`sysmem.c`).

## USB console

ST VCP VID/PID 0483:5740 (same as STM32duino). Product string "Livo Stainer <Board>". The serial number is the MCU's 96-bit unique ID. Only PA11/PA12 are configured: no VBUS or ID pins, because PA9/PA10 carry board functions (PA10 is Gantry NRST on the Master).
