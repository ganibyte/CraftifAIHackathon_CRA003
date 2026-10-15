# WearTrack firmware (PRD v1.0)

ESP32 DevKit + MPU6050 wearable: step counting, fall detection (with cancel), posture monitoring,
and a **non-connectable BLE beacon** (`WT_XX`) carrying a 14-byte status payload. ESP-IDF v5.5, NimBLE.

## Build, flash, monitor
```bash
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build flash monitor          # UART 115200
```
Optional settings: `idf.py menuconfig` -> **WearTrack** (pins, expected WHO_AM_I, PM on/off, gyro standby).

### Wiring
| MPU6050 | ESP32 |
|---|---|
| VCC / GND | 3V3 / GND |
| SDA / SCL | GPIO21 / GPIO22 (400 kHz) |
| INT | GPIO23 (free-fall, active high) |
| BOOT button | GPIO0 (active low) |
| LED | GPIO2 |

## Layout
```
main/                    ESP-IDF layer (hardware + RTOS)
  main.c                   sensor task, tiers, timing monitor, evidence logs
  imu_mpu6050.c/.h         I2C 400 kHz, WHO_AM_I, +/-8 g, FIFO, free-fall INT
  ble_beacon.c/.h          NimBLE non-connectable advertising, payload refresh
  board.c/.h               LED patterns, BOOT short/long press
components/wt_core/      hardware-independent C (no ESP-IDF calls, FR-20)
  include/wt_config.h      ALL thresholds / durations (FR-13)
  wt_step / wt_fall / wt_posture / wt_calib / wt_activity / wt_signal / wt_payload / wt_crc8
test/host/               host unit tests:  make -C test/host
tools/decode_payload.py  decode + CRC-check a captured payload
docs/TRACEABILITY.md     requirement -> code -> evidence
```

## Host unit tests (no hardware)
```bash
make -C test/host        # builds with gcc and runs 50 checks
```
Covers CRC-8, payload/advertising layout (28 bytes), tilt math, calibration restart, step counting
(walk, isolated peaks, 3-step retroactive credit, random shaking), the 3-stage fall FSM (success,
upright-after-impact, movement, timeout, short free-fall, 50 Hz sampling), posture hysteresis and the
activity/cancel logic.

## UART output (115200)
```
BT MAC .. / BLE name WT_xx / CALIB done upright=(x,y,z) / BLE advertising started
STEP n t=us          FALL s1|s2|s3|ALERT t=us      CANCEL t=us      POSTURE on|off t=us
TIMING cycle_us=.. violation=0|1 period_us=..      SUMMARY (every 10 s)
```
Timing-evidence lines for AC-7 (each prints `ok=1` when the TR is met):
`TR3 ff_to_fast_us`, `TR5 inactivity_ms`, `TR6 confirm_to_adv_us / impact_to_adv_ms`,
`TR7 step_to_adv_ms`, `TR8 cancel_to_adv_us`.

## BLE payload
Manufacturer data `FF FF` + 14 bytes, big-endian: version, sequence, flags, state, steps(4), tilt, falls,
uptime_min(2), violations, CRC-8 (poly 0x07, init 0). Decode a captured value with
`python3 tools/decode_payload.py <hex>`.

## Acceptance checklist (on hardware)
AC-1..AC-6: run the procedures in PRD sec. 8 with a phone scanner (nRF Connect) and read the UART log.
AC-7: leave it running for 5 minutes; `TR*` lines must say `ok=1` and `SUMMARY violations=0`.
AC-8 (power): record current for both builds, then compare.
```bash
idf.py build                                   # optimized firmware (DFS + modem sleep, 50 Hz FIFO tiers)
idf.py -B build_baseline -D SDKCONFIG=sdkconfig_baseline \
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.baseline" build   # 100 Hz polling baseline, no PM
```
The `PWR config:` line printed at boot is the recorded power configuration.

## Honest status / things to verify on the board
* **Not built or flashed in the authoring environment** (no ESP-IDF available). The pure core is compiled and
  tested on the host (50/50 pass); the ESP-IDF layer was only syntax-checked against stub headers. Expect to
  fix small API/Kconfig mismatches on the first `idf.py build`.
* All thresholds are starting points (PRD note). Step detection in particular needs real gait data: a pure
  rhythmic shake is physically similar to fast walking; the interval-regularity and clutter rules in
  `wt_config.h` (`STEP_REGULARITY`, `STEP_NOISE_PEAKS*`) are the knobs to tune for AC-2.
* **Interpretations of the PRD** (change in `main.c` / `wt_activity.c` if you disagree):
  - State stays `ALERT` and the fall flag stays set until cancel (BOOT short press within 10 s) or recalibration;
    the 100 ms advertising interval lasts 30 s, then the normal 1 s interval continues with the alert payload.
  - Impact-to-alert budget (TR-6): a 150 ms post-impact settle window (`POST_IMPACT_SETTLE_MS`) precedes the 2 s
    inactivity window; movement after the settle window discards the candidate.
  - The 200 Hz tier lasts 3 s from the last free-fall interrupt and is kept alive while a fall candidate is
    in progress, so the 2 s inactivity window can finish at 200 Hz. A software free-fall trigger backs up the INT.
  - Step/posture filters always run at the normal rate (the 200 Hz stream is decimated 4:1).
  - `POSTURE_LPF_TAU_S` = 0.4 s (about 1 s settling) so bad posture asserts at 30 s +/-1 s (TR-10).
  - Auto light sleep with BLE on a classic ESP32 needs an external 32 kHz crystal; with the default main-XTAL
    low-power clock expect DFS + BT modem sleep. FR-17 is P3 and the final configuration is logged at boot.
* `WT_IMU_GYRO_STANDBY` (menuconfig) saves ~3 mA but is off by default; enable and verify FIFO output on your board.
