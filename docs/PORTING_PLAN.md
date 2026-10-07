# Porting plan

Goal: replace STM32duino with native HAL/LL drivers for better runtime debugging, deterministic timing and further optimisation, without changing the instrument's behaviour or command protocol. The Master board goes first.

Each milestone ends with every configuration building with zero warnings. Each milestone that changes behaviour also needs a bench check against the Arduino firmware.

| # | Milestone | Content | Exit criteria |
|---|---|---|---|
| 1 | **Platform skeleton** (done, not flashed) | Project with 14 configurations; clocks; startup/linker per MCU; GPIO; time base; UART (IRQ ring buffers, 8N1/8E1); USB CDC; `Print`/`Stream`; bring-up app | All configurations build. On hardware: USB enumerates and `ID` answers on USB and the upstream UART for each board |
| 2 | **Remaining drivers** (code done, not run on hardware) | ADC continuous scan + DMA (ADC1/ADC3), PWM (`analogWrite`, fan PWM), I2C master with bus recovery and I2C slave, EXTI, native TIM7 step engine, IWDG. Pin tables generated from STM32duino's ST tables. Flash writes use HAL FLASH directly (enabled) | Bring-up `ADC <pin>` raw values match Arduino `analogRead` on the same input. `I2CSCAN` finds MPU6050 (0x68) on Master and LiDAR (0x29) on Gantry. PWM frequency and duty are verified in milestone 4 through the application's output commands, because PWM pins drive pumps and fans |
| 3 | **Compatibility pieces** (done, host-tested) | `WString` (Arduino `String` subset). `Devices/Tmc2209` replacing TMCStepper. Clean-room `MotionPlanner` replacing AccelStepper, driving both `StepEngine` (interrupt) and `PolledStepper` (main loop, bed T motor) | Host tests (`tools/run_host_tests.ps1`): `String` 45/45; TMC2209 datagrams byte-identical to TMCStepper 0.7.3 for 8 motor configurations plus reads (123/123); planner step times identical to AccelStepper 1.64 in 15 scenarios incl. retarget, reversal, stop and limit changes (80/80) |
| 4 | **Master application** (ported and building; bench comparison pending, see `docs/PORT_LOG_MASTER.md`) | Port `LivoStainer` sources for Master: command router, protocol parser, pumps/cascades, bed link (`BS2`), service diagnostics, AN3155 flasher | Command-for-command comparison with the Arduino Master on the bench; bed run with `LOOPSTAT`; ESP32 link and OTA flashing of Gantry/Nozzle |
| 5 | **Nozzle** | Nozzle application, DHT11, fans with tach, Hall homing | Bench comparison incl. bed production with Master |
| 6 | **Gantry (UART)** | Gantry application, magazine UART transport, LiDAR (VL53L0X), accelerometers | Load/unload sequence on the bench |
| 7 | **Magazine, Hall boards** | Magazine app behind bootloader, NeoPixel via TIM+DMA, Hall I2C slaves | Magazine OTA via Gantry relay; Hall position telemetry |
| 8 | **Optimisation** | DMA UART RX, ADC DMA everywhere, remove `String` from hot paths, split `Execution2019Handler`, IWDG on all boards | `LOOPSTAT` before/after; no protocol changes |

## Source baseline

The application is ported from **`RadoratoryTechnologies/Livo-Stainer` `perf/timer-step-engine` @ `271a10b`** (user decision, 2026-10-07): the Arduino Master V2.6.0, including the Task 1 timer step engine. Later changes on the Arduino side, such as the `OTA` branch's AIR-2 and MG stain-2 work, are not included.

Later Arduino-side changes must be carried across deliberately and listed in [DECISIONS.md](DECISIONS.md).

## Rules for the port

- Keep the text protocol, acknowledgements, version reporting and diagnostic frames byte-compatible with the Arduino firmware unless a change is agreed.
- Keep the T-motor step count as the single time base for dispatch and wash events.
- No board is flashed, and no physical motion is run, without an explicit request.
