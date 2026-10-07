# Master native build: bench comparison (milestone 4 exit, not yet run)

Compare the native Master (`Master-Release`, or `Master-Debug` for debugging) with the Arduino Master V2.6.0 built from Livo-Stainer `perf/timer-step-engine` @ `271a10b`. Gantry, Nozzle and ESP32 keep their current Arduino/ESP firmware. Record results in the table at the end.

**Safety:** these steps move motors, run pumps and switch DC loads. Use a bench instrument with reagent lines on water or disconnected. Keep the usual emergency stop within reach.

## 1. Flash and identify

1. Flash `firmware/Master-Release/Stainer_STM_Native-Master.bin` at 0x08000000. Use ST-LINK, or the ESP32 local service installer (Master target).
2. **Expected:**
   - USB COM port "Livo Stainer Master" enumerates.
   - Boot prints `Device Started: Master`, the device ID and `FW:V2.6.0`.
   - `ID` (USB, or `M; ID` from the ESP32) answers `...,FW:V2.6.0,BUILD:NATIVE <date> <time>,UID:...`.
   - The ESP32 web UI shows the Master online with its usual ID polling.

## 2. Links and console

- `G; ID` and `N; ID` return the Gantry and Nozzle replies through the Master (`[G]`/`[N]` echoes).
- `COMMSTAT`, `LOOPSTAT` and `STEPISR` answer.
  - `STEPISR` must report `MOTORS:7`.
  - `LOOPSTAT` must show `STEP_ISR_MOTORS:7` and `ISR_MAX_PERIOD_US` close to 20.
- Send a burst of ESP32 commands (e.g. a profile push). Nothing is dropped: compare `COMMSTAT` with the Arduino build.

## 3. Sensors

- IR1-IR12 raw values (`PIR 1`, then `PIR 0`): with the same slides present or absent, compare with the Arduino build. Expect the same counts within normal noise.
- Limit inputs (`PLS 1`) toggle as before.
- MPU6050 (`PAM 1`) reports plausible acceleration. If it doesn't, check the bus with the bring-up firmware's `I2CSCAN`.

## 4. Outputs

Measure with a scope on the PWM pins.

| Command | Pin | Expect |
|---|---|---|
| `DCW1 128` / `DCW1 0` | PD15 | 1 kHz, 50 % / off |
| `DCW2 64` | PE9 | 1 kHz, 25 % |
| `DCM1 255` | PE14 | 1 kHz, 100 % |
| `DCM2`, `DCS1`, `DCS2` | non-PWM pins | Digital: values ≥ 128 are on |
| `DCD 1` / `DCD 0` | drain, digital | On / off |
| `MIXD` / `MIXR` | PB14 / PB15 | PWM only on the commanded pin |

For `MIXD`/`MIXR`, PE11 must not mirror the PWM.

## 5. Motion

For each pump and axis (X, Y, Z, M14, M15, M17, M18):
1. Run the same move with `STEPISR 0` and with `STEPISR 1`.
2. The end position (`ACK`) must match.
3. With `STEPISR 1`, durations are at or below the Arduino build's (exact timing, see Task 1).
4. Weigh M14, M15, X, Y and Z dispenses; volumes must match the Arduino build.

## 6. Production sequences

1. Run a feed with Gantry and Nozzle on their current firmware.
2. Check:
   - **Bed:** the `BS2` barrier never faults.
   - **Rate:** `LOOPSTAT` `BED_STEPS_PER_S` against `BED_CMD_STEPS_PER_S`.
   - **Cascades:** B1/S2/SX/WY run.
   - **Commands:** `FEEDSTOP`/`FEED` pause and resume behave as before.
   - **Reagent lock:** `REAGENTLOCK 1` blocks dispensing.
3. Run `MLOAD`, `MUNLOAD`, `DRAIN` and `PARA CHECK`, and compare their logs with the Arduino build.

## 7. Service and flashing

- Diagnostics: run a service-mode `DTEST` from the Diagnostics Console. Rows must be CRC-valid and complete.
- Flash a known Gantry or Nozzle image through the Master (`FLASH G|N` from the ESP32, or `GFLASH` over USB). Expect `FLASHDONE` and that board reporting its version.

## Results

| Section | Date | Arduino reference | Native | Pass/Fail | Notes |
|---|---|---|---|---|---|
| | | | | | |
