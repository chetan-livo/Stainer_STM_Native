# Stainer_STM_Native

Native STM32 (HAL/LL/CMSIS) firmware for the Livo Stainer controller boards, replacing the STM32duino/Arduino build. One STM32CubeIDE project builds every board through its own build configuration. CubeMX is not used; clocks, pins and peripherals are configured in code.

| Folder | Contents |
|---|---|
| `firmware/` | STM32CubeIDE project (`.project`, `.cproject`), sources and ST drivers |
| `firmware/Platform/` | Native platform layer: clocks, startup, linker scripts, GPIO, time base, UART, USB CDC, `Print`/`Stream`, ADC, PWM, I2C, EXTI, step engine, watchdog |
| `firmware/Boards/` | Per-board configuration (pins, peripherals) |
| `firmware/Devices/` | External chip drivers (TMC2209) |
| `firmware/App/` | Livo application, ported from Livo-Stainer `271a10b` (built for Master so far) |
| `firmware/Compat/` | Arduino/library header names mapped onto the platform |
| `firmware/BringUp/` | Bring-up app for boards not yet ported |
| `firmware/Drivers/`, `firmware/Middlewares/` | Unmodified ST code from STM32Cube_FW_F4 V1.28.3 (CMSIS, HAL/LL, USB device library) |
| `docs/` | Plan, architecture, board matrix, decision log |
| `tools/` | Project generator and headless build script |
| `tests/host/` | Host unit and equivalence tests |
| `validation/` | Bring-up and bench procedures |

## Build configurations

| Board | MCU | Configurations |
|---|---|---|
| Master | STM32F446ZET6 | `Master-Debug`, `Master-Release` |
| Nozzle | STM32F407VET6 | `Nozzle-Debug`, `Nozzle-Release` |
| Gantry (UART magazine transport) | STM32F407VET6 | `Gantry-Debug`, `Gantry-Release` |
| Magazine holder 1 / 2 (app behind bootloader, 0x08020000) | STM32F446ZET6 | `Magazine1-*`, `Magazine2-*` |
| Gantry X Hall | STM32F446ZET6 | `GantryXHall-*` |
| Gantry Z Hall | STM32F401RCT6 | `GantryZHall-*` |

Debug configurations build with `-Og -g3`; Release with `-O2 -g` (symbols kept for post-mortem debugging).

## Open in STM32CubeIDE

1. **File → Import → General → Existing Projects into Workspace**, select the `firmware` folder. Leave **Copy projects into workspace** unchecked.
2. Choose the board in **Project → Build Configurations → Set Active**, then **Build**.
3. Outputs are in `firmware/<Configuration>/`: `.elf`, `.bin`, `.hex`, `.map`.

Tested with STM32CubeIDE 2.2.0.

## Command-line build

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1                  # every configuration
powershell -ExecutionPolicy Bypass -File tools/build.ps1 Master-Release   # one
```

Pin tables are generated too: `python tools/generate_pinmaps.py` (needs the STM32duino 3.0.0 core installed). Build settings are generated: edit `tools/generate_cubeide_project.py`, then run `python tools/generate_cubeide_project.py`. Do not edit `.cproject` by hand in parallel, because the next regeneration overwrites it.

## Host tests

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_host_tests.ps1
```

This needs LLVM-MinGW (`winget install MartinStorsjo.LLVM-MinGW.UCRT`). The equivalence tests also need the Arduino libraries TMCStepper 0.7.3 and AccelStepper 1.64 in `Documents/Arduino/libraries`. They are references only, never linked into firmware.

## Status

Milestones 1 (platform skeleton), 2 (ADC, PWM, I2C, EXTI, step engine, watchdog) and 3 (`String`, TMC2209 driver, clean-room motion planner) build for all 14 configurations with zero platform warnings, and all host tests pass. Milestone 4: the **Master application** is ported and builds (`Master-*`: about 140 KB flash, 36 KB RAM); the bench comparison is pending ([validation/master-bench.md](validation/master-bench.md)). Milestone 5: the **Nozzle application** is ported and builds with no application changes; its bench comparison is pending ([validation/nozzle-bench.md](validation/nozzle-bench.md)). **No board has been flashed yet.** See [docs/PORTING_PLAN.md](docs/PORTING_PLAN.md) for the milestones and [validation/bring-up.md](validation/bring-up.md) for the first hardware check.
