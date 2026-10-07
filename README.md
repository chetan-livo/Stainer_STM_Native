# Stainer_STM_Native

Native STM32 (HAL/LL/CMSIS) firmware for the Livo Stainer controller boards, replacing the STM32duino/Arduino build. One STM32CubeIDE project builds every board through its own build configuration. CubeMX is not used; clocks, pins and peripherals are configured in code.

| Folder | Contents |
|---|---|
| `firmware/` | STM32CubeIDE project (`.project`, `.cproject`), sources and ST drivers |
| `firmware/Platform/` | Native platform layer: clocks, startup, linker scripts, GPIO, time base, UART, USB CDC, `Print`/`Stream`, ADC, PWM, I2C, EXTI, step engine, watchdog |
| `firmware/Boards/` | Per-board configuration (pins, peripherals) |
| `firmware/App/` | Application. Milestone 1 holds only the bring-up app |
| `firmware/Drivers/`, `firmware/Middlewares/` | Unmodified ST code from STM32Cube_FW_F4 V1.28.3 (CMSIS, HAL/LL, USB device library) |
| `docs/` | Plan, architecture, board matrix, decision log |
| `tools/` | Project generator and headless build script |
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

## Status

Milestones 1 (platform skeleton) and 2 (ADC, PWM, I2C, EXTI, step engine, watchdog drivers) build for all 14 configurations with zero warnings. **No board has been flashed yet.** See [docs/PORTING_PLAN.md](docs/PORTING_PLAN.md) for the milestones and [validation/bring-up.md](validation/bring-up.md) for the first hardware check.
