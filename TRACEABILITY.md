# Traceability: requirement -> implementation -> evidence

| Req | Where | Evidence |
|---|---|---|
| FR-1 | `imu_mpu6050.c` (`imu_init`), `main.c` (`enter_fault`), `board.c` | UART `ERROR: sensor fault`, LED 5 Hz, flag bit3, BLE continues; recovery retry every 5 s |
| FR-2 | `imu_mpu6050.c`, `main.c` (`NORMAL_*`) | +/-8 g (ACCEL_CFG 0x10), 50 Hz, FIFO; `TIMING period_us` ~200000 |
| FR-3 | `wt_calib.c`, `main.c`, `board.c` | UART `CALIB start/restart/done`; long press -> `RECALIB requested` |
| FR-4 | `wt_step.c` | host test "walked 20 steps -> counted 20" |
| FR-5 | `wt_step.c` | host tests (isolated peaks = 0, 3 bumps = +3 at once, shaking <= 3) |
| FR-6 | `wt_activity.c` | host test "activity"; payload byte 3 |
| FR-7 | `wt_fall.c` | host test "fall" (stages, tilt gate, timeout) |
| FR-8 | `wt_activity.c`, `main.c` | `FALL ALERT`, flag bit0, fall counter, 100 ms adv for 30 s |
| FR-9 | `main.c` (`on_btn_short`) | `CANCEL t=`, `TR8 ... ok=1` |
| FR-10 | `wt_fall.c` | `FALL discard reason=` lines; host tests |
| FR-11 | `main.c` (`set_tier`, `on_ff_irq`) | `TIER FAST ...`, `TR3 ff_to_fast_us`, `TIMING period_us` ~50000 |
| FR-12 | `wt_posture.c` | host test "posture ON after 30720 ms / OFF after 5160 ms"; `POSTURE on/off` |
| FR-13 | `components/wt_core/include/wt_config.h` | single header, all FR-13 names present |
| FR-14 | `main.c` (`app_main`), `ble_beacon.c` | UART MAC + name; non-connectable (`BLE_GAP_CONN_MODE_NON`) |
| FR-15 | `wt_payload.c` (`wt_adv_build`) | host test: 28-byte adv, company ID FF FF |
| FR-16 | `ble_beacon.c`, `main.c` (`publish`) | 1000/100 ms intervals; immediate refresh on state change |
| FR-17 | `main.c` (`power_config`), `sdkconfig.defaults` | `PWR config:` line at boot |
| FR-18 | `main.c` (`on_tick`) | `TIMING ... violation=`; flag bit4; saturating counter |
| FR-19 | `board.c` | heartbeat 20 ms / 5 s, solid in ALERT, 5 Hz on fault |
| FR-20 | `components/wt_core` | no ESP-IDF includes; `make -C test/host` |
| UR-1..4 | `main.c`, `ble_beacon.c` | UART log formats in README |
| TR-1 | `main.c` | `TIMING period_us` |
| TR-2 | `main.c` | `TIMING cycle_us`, `violation` |
| TR-3 | `main.c` | `TR3 ff_to_fast_us ... ok=` |
| TR-4 | 200 Hz samples feed `wt_fall_feed` (5 ms) | `TIER FAST rate=200Hz` |
| TR-5 | `wt_fall.c`, `main.c` | `TR5 inactivity_ms ... ok=` |
| TR-6 | `main.c` (`publish`) | `TR6 confirm_to_adv_us / impact_to_adv_ms ... ok=` |
| TR-7 | `main.c` (`publish`) | `TR7 step_to_adv_ms ... ok=` |
| TR-8 | `board.c`, `main.c` | `TR8 cancel_to_adv_us ... ok=` |
| TR-9 | `main.c` (BLE starts before IMU/calibration) | `BLE advertising started t=` vs boot `t=` |
| TR-10 | `wt_posture.c` | host test posture timings |
| TR-11 | `ble_beacon.c`, `wt_config.h` | scanner timestamps (nRF Connect / sniffer) |
| AC-8 | `sdkconfig.baseline`, `Kconfig.projbuild` | two builds + `PWR config:` lines |
