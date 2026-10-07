# Nozzle native build: bench comparison (milestone 5 exit, not yet run)

Compare the native Nozzle (`Nozzle-Release`) with the Arduino Nozzle V2.5.0 from Livo-Stainer `perf/timer-step-engine` @ `271a10b`. Run it first with the Arduino Master, then with the native Master once [master-bench.md](master-bench.md) passes.

**Safety:** motors, fans and dispensing run. Use a bench instrument with water or disconnected reagent lines, and keep the emergency stop within reach.

## 1. Flash and identify

1. Flash `firmware/Nozzle-Release/Stainer_STM_Native-Nozzle.bin` at 0x08000000. Use ST-LINK, or through the Master (`FLASH N <size> <crc>` from the ESP32 relay, or `GFLASH`-style USB tooling for the Nozzle).
2. **Expected:**
   - USB COM port "Livo Stainer Nozzle".
   - `N; ID` answers `FW:V2.5.0,BUILD:NATIVE ...`.
   - The Master shows `Nozzle: Ready`.
3. `N; STEPISR` reports `MOTORS:6`. `N; LOOPSTAT` shows `STEP_ISR_MOTORS:6` and `ISR_MAX_PERIOD_US` close to 20.

## 2. Sensors

- IR1-IR13 raw values (`N; PIR 1`, then `PIR 0`): compare with the Arduino build for the same slide positions.
- Hall homing inputs: run each homing sequence (`NZHO`) at least 5 times. Home positions must be repeatable and match the Arduino build.
- DHT11: `N; RDHT` returns plausible temperature and humidity at least 5 times in a row, with no timeouts.

## 3. Fans and motor outputs

Measure with a scope or tachometer.

| Command | Pin | Expect |
|---|---|---|
| `DCF1 128` / `DCF2 128` | PE11 / PE13 | 1 kHz, 50 % |
| `DCM 128` | PE9 | 1 kHz, 50 % |
| Ethanol fan (Fan 4) and WX stroke fan (Fan 3) | PE5 / PC9 | 25 kHz at the commanded percentage, triggered by the production or automation commands that drive them |

For Fans 3 and 4, also check that the reported RPM changes with duty (tach on PE6 / PC8).

## 4. Motion

For SX, SY, WX, WY, BX and BY:
- the same moves with `N; STEPISR 0` and `STEPISR 1` end at the same position;
- strokes match the Arduino build in position;
- with `STEPISR 1`, duration is at or below the Arduino build's.

## 5. Production with the Master

Run a feed, first with the Arduino Master, then with the native Master. Check:
- the `BS2` barrier never faults;
- `IR4` slide detection admits each slide once;
- SX/SY/WX/WY events fire at the same bed positions (`PARA CHECK` trace);
- the ethanol and dry fan windows switch as before;
- `FEEDSTOP`/`FEED` pause and resume behave as before.

## 6. Service

`DTEST` from the Diagnostics Console completes. Rows are CRC-valid and the motor exercise rows pass.

## Results

| Section | Date | Arduino reference | Native | Pass/Fail | Notes |
|---|---|---|---|---|---|
| | | | | | |
