# Milestone 1 bring-up check (not yet run)

Purpose: prove the clock tree, startup, linker map, UART and USB on real hardware before porting the application. The bring-up firmware only configures the USB pins and the board's upstream UART. Every other pin, including motor enables and outputs, stays in its reset state (input). **This replaces the board's production firmware until it is reflashed.**

## Per board

1. Build the board's `-Debug` configuration.
2. Flash it with ST-LINK, either from CubeIDE (**Run → Debug As → STM32 C/C++ Application**) or with STM32CubeProgrammer at the image's address: 0x08000000, or 0x08020000 for Magazine.
3. Connect USB. A COM port "Livo Stainer <Board>" (VID 0483, PID 5740) must appear.
4. Open it in a terminal at any baud rate with DTR enabled. Expect one line:
   `Stainer_STM_Native BOARD=<Board> MCU=<part> SYSCLK=<180|168|84>MHz UID=... BUILD=... UP_MS=...`
5. Type `ID` and press Enter: the same line appears. Any other text comes back prefixed `ECHO `.
6. Upstream UART (115200 8N1): the board prints the same identity line at boot and answers `ID`.

   | Board | Port | Pins |
   |---|---|---|
   | Master | USART3 | PB10 TX / PB11 RX (ESP32 link) |
   | Nozzle, Gantry | UART4 | PC10 TX / PC11 RX (Master link) |
   | Magazine | USART1 | PB6 TX / PB7 RX |
   | Hall boards | none (USB only) | |

7. In the debugger, check `SystemCoreClock` and that `platform_last_error` stays 0.

## Restore

Reflash the board's Arduino release image. On the Master use the ESP32 service installer, or ST-LINK.

## Results

| Board | Date | USB | Upstream UART | Notes |
|---|---|---|---|---|
| | | | | |
