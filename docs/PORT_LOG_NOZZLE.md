# Nozzle application port log (milestone 5)

**Source:** the same snapshot as the Master: Livo-Stainer `perf/timer-step-engine` @ `271a10b`, Nozzle firmware V2.5.0. The application files in `firmware/App/` are shared by all boards, so the Nozzle port needed **no application source changes** beyond the two Master edits in [PORT_LOG_MASTER.md](PORT_LOG_MASTER.md). Enabling it took one line: `Nozzle` was added to `PORTED` in `tools/generate_cubeide_project.py`.

Build: `Nozzle-Debug` 117 KB flash and `Nozzle-Release` 123 KB flash, 36 KB RAM each. The Arduino Nozzle was 128 KB flash.

## Nozzle-specific items reviewed

| Item | Native behaviour |
|---|---|
| Motor UARTs | Six TMC2209 drivers over three UARTs: SX/SY on USART6 (PC6/PC7), BX/BY on USART2 (PA2/PA3), WX/WY on UART5 (PC12/PD2), all at 500 kbaud. Master link on UART4 (PC10/PC11). All pairs are in the UART pin table |
| Stepping | All six axes on the TIM7 step interrupt (`STEPISR` reports 6). The bed-gate holds keep the Arduino freeze semantics |
| Fan 1, Fan 2, motor | `analogWrite` on PE11/PE13/PE9 (TIM1 CH2/CH3/CH1), 1 kHz, same arithmetic as STM32duino |
| Fan 3, Fan 4 (25 kHz) | `DCFan` through the `HardwareTimer` shim to `pwmWritePercent`: PC9 is TIM3 CH4 and PE5 is TIM9 CH1, the timers `DCFan` names |
| Fan tachometers | `attachInterrupt` on PC8 and PE6, falling edge (EXTI9_5) |
| IR / Hall inputs | 12 channels on ADC1 through the DMA scan (≤ 16 per ADC). IR9 (PB1) stays digital for the DHT11; `AnalogSensorReader` skips it as before |
| DHT11 | Vendored Adafruit driver, unchanged; it times pulses by counting `digitalRead` loop iterations. The native `digitalRead` is a register read, faster than STM32duino's, so iteration counts are higher. Bit decoding compares low against high counts, so the ratio, and the decoded data, are unaffected. The 1 ms timeout is computed from `SystemCoreClock` as before (see start-up order below) |

## Start-up order (applies to every board)

STM32duino configures HAL and the clock tree in a priority-101 constructor, before any C++ global is constructed. The native `main.cpp` now does the same (`premain`, verified as the first `.init_array` entry). Globals such as `DHT nozzleDHT` therefore see the final 168 MHz `SystemCoreClock`, exactly as in the Arduino build.

## Known shared behaviour

A DHT read masks interrupts for about 4–5 ms (the driver's `InterruptLock`). Step pulses and UART reception pause for that time, as they did in the Arduino build. The firmware only reads the DHT when production is not busy.

## Warnings

44 `-Wextra` warnings, all in unmodified application code. Beyond the Master's categories, the only new ones are unused Nozzle helpers (`nzRunToSwitch`, `nzMotorStateName`) and two misleading indentations (`Execution2019Handler.cpp:6610`, `:7404`), which are statements on the same line as an `if`/`for`. All are style only.

## Not verified

Nothing has run on hardware. See [validation/nozzle-bench.md](../validation/nozzle-bench.md).
