# Porting plan

Goal: replace STM32duino with native HAL/LL drivers for better runtime debugging, deterministic timing and further optimisation, without changing the instrument's behaviour or command protocol. The Master board goes first.

Each milestone ends with every configuration building with zero warnings. Each milestone that changes behaviour also needs a bench check against the Arduino firmware.

| # | Milestone | Content | Exit criteria |
|---|---|---|---|
| 1 | **Platform skeleton** (done, not flashed) | Project with 14 configurations; clocks; startup/linker per MCU; GPIO; time base; UART (IRQ ring buffers, 8N1/8E1); USB CDC; `Print`/`Stream`; bring-up app | All configurations build. On hardware: USB enumerates and `ID` answers on USB and the upstream UART for each board |
| 2 | **Remaining drivers** | ADC (scan + DMA, ADC1/ADC3), PWM (TIM channel table, complementary outputs), I2C master with bus recovery, I2C slave (Hall/magazine), EXTI, step timer (from the Arduino repo's `StepEngine`), IWDG, flash write (magazine bootloader) | Driver self-tests on the bench: ADC raw values match Arduino `analogRead`, PWM frequency/duty measured, I2C scan finds MPU6050/LiDAR |
| 3 | **Compatibility pieces** | `WString` (Arduino `String` subset used by the app), `Stream` helpers, TMC2209 UART driver replacing TMCStepper, planner replacing AccelStepper (shared with `StepEngine`) | Unit-level checks of `String` parsing and TMC register datagrams against TMCStepper output |
| 4 | **Master application** | Port `LivoStainer` sources for Master: command router, protocol parser, pumps/cascades, bed link (`BS2`), service diagnostics, AN3155 flasher | Command-for-command comparison with the Arduino Master on the bench; bed run with `LOOPSTAT`; ESP32 link and OTA flashing of Gantry/Nozzle |
| 5 | **Nozzle** | Nozzle application, DHT11, fans with tach, Hall homing | Bench comparison incl. bed production with Master |
| 6 | **Gantry (UART)** | Gantry application, magazine UART transport, LiDAR (VL53L0X), accelerometers | Load/unload sequence on the bench |
| 7 | **Magazine, Hall boards** | Magazine app behind bootloader, NeoPixel via TIM+DMA, Hall I2C slaves | Magazine OTA via Gantry relay; Hall position telemetry |
| 8 | **Optimisation** | DMA UART RX, ADC DMA everywhere, remove `String` from hot paths, split `Execution2019Handler`, IWDG on all boards | `LOOPSTAT` before/after; no protocol changes |

## Source baseline

The application will be ported from one fixed commit of `RadoratoryTechnologies/Livo-Stainer` so behaviour can be compared exactly. **To be decided** before milestone 4:

- **`perf/timer-step-engine` @ `271a10b`:** has the timer step engine and latency fixes; not yet bench-validated.
- **`OTA` once its Master build is fixed:** on 2026-10-07, `stain1DispenseSpeedFor()` was referenced but not committed.

Later Arduino-side changes must be carried across deliberately and listed in [DECISIONS.md](DECISIONS.md).

## Rules for the port

- Keep the text protocol, acknowledgements, version reporting and diagnostic frames byte-compatible with the Arduino firmware unless a change is agreed.
- Keep the T-motor step count as the single time base for dispatch and wash events.
- No board is flashed, and no physical motion is run, without an explicit request.
