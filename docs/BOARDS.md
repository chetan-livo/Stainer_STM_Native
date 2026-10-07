# Board matrix

Derived from the Arduino firmware's board headers (`StainerMasterPCBV1.h`, `NozzleMountPCBV1.h`, `StainerGantryPCBV1.h`, `Magazine_PCB_V2.h`, `StageXY_Connections.h`). Verify against the PCB schematics before driving outputs; no schematics are in this repository.

## Master (STM32F446ZET6)

| Function | Peripheral | Pins | Notes |
|---|---|---|---|
| ESP32 link (PiSerial) | USART3 | PB10 TX / PB11 RX | 115200 8N1 |
| Gantry link | USART6 | PC6 TX / PC7 RX | 115200; 8E1 when flashing via ROM bootloader |
| Nozzle link | UART4 | PC10 TX / PC11 RX | 115200; 8E1 when flashing |
| TMC Z, T, M14, M15 | USART2 | PA2 / PA3 | 500 kbaud, EN PE12 |
| TMC X, Y, M17, M18 | UART5 | PE8 TX / PE7 RX | 500 kbaud, EN PG5 |
| USB console | OTG FS | PA11 / PA12 | |
| Remote I2C bus 1 | I2C1 | PB9 SDA / PB8 SCL | |
| Remote I2C bus 3 + MPU6050 | I2C3 | PC9 SDA / PA8 SCL | |
| IR1-6, IR12 | ADC1 | PA4-PA7, PC4, PC5, PC0 | |
| IR7-11 | ADC3 | PF3, PF6-PF9 | |
| Wash 1 | TIM4 CH4 | PD15 | PWM |
| Wash 2 | TIM1 CH1 | PE9 | PWM |
| Mixing 1 / Mixing 2 | TIM1 CH4 / CH2 | PE14 / PE11 | PWM (OTA pin map of 2026-10-06) |
| Drain 1 / Drain 2 / Fan | GPIO | PG9 / PB14 / PB15 | digital |
| Gantry BOOT0 / NRST | GPIO | PB1 / PA10 (open drain) | |
| Nozzle BOOT0 / NRST | GPIO | PF14 / PE10 (open drain) | |
| Step pins | GPIO | PF10, PE3, PF12, PG0, PG14, PD0, PG3, PD13 | driven by the step timer |

## Nozzle (STM32F407VET6)

| Function | Peripheral | Pins |
|---|---|---|
| Master link | UART4 | PC10 TX / PC11 RX |
| TMC SX, SY | USART6 | PC6 / PC7 |
| TMC BX, BY | USART2 | PA2 / PA3 |
| TMC WX, WY | UART5 | PC12 TX / PD2 RX |
| IR / Hall home inputs (13) | ADC1 | PA0, PA1, PA4-PA7, PC4, PB0, PB1, PC0-PC3 |
| Motor, fans 1/2 | TIM1 | PE9, PE11, PE13 |
| Fan 3 / Fan 4 (25 kHz) + tach | TIM3 CH4 / TIM9 CH1, EXTI | PC9 / PE5, tach PC8 / PE6 |
| DHT11 | GPIO | PB1 |

## Gantry, UART transport (STM32F407VET6)

| Function | Peripheral | Pins |
|---|---|---|
| Master link | UART4 | PC10 TX / PC11 RX |
| Magazine 1 / 2 | USART6 / USART3 | PC6, PC7 / PB10, PB11 |
| TMC GX, GZ, GR | UART5 | PC12 TX / PD2 RX |
| TMC GY | USART2 | PA2 / PA3 |
| Accelerometer 1 | I2C1 | PB7 / PB8 |
| LiDAR, Z Hall (0x15) | I2C3 | PC9 / PA8 |

The Gantry I2C magazine build is out of scope (UART only).

## Magazine holder (STM32F446ZET6)

| Function | Peripheral | Pins |
|---|---|---|
| Gantry link | USART1 | PB6 TX / PB7 RX |
| IR sensors | ADC | 24 channels |
| NeoPixel | TIM + DMA (planned) | PE2 |
| Solenoids, IR enable | GPIO | PB12, PC9, PE5 |

The application links at 0x08020000 behind the magazine bootloader. Sector 7 holds LED calibration.

## Gantry X Hall (STM32F446ZET6) and Z Hall (STM32F401RCT6)

| Function | Peripheral | Pins |
|---|---|---|
| Hall sensors | ADC | 10 channels (PA0-PA7, PC4, PC5) |
| Gantry bus (slave) | I2C1 | PB9 SDA / PB8 SCL |
| LED | GPIO | PC6 |
