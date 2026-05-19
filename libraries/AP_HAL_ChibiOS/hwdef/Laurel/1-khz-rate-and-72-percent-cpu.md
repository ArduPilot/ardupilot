# Laurel 1 kHz Backend and CPU Load Report

Date: 2026-05-14 (updated for dual-core branch; original data collected 2026-05-12)
Scope: current settings in this branch and observed runtime after flash + param-storage erase.

## 1. Current Laurel defaults

From `libraries/AP_HAL_ChibiOS/hwdef/Laurel/defaults.parm`:

- `SCHED_LOOP_RATE 150`
- `INS_FAST_SAMPLE 1`
- `INS_GYRO_RATE 0`
- `FSTRATE_DIV 3`
- `FSTRATE_ENABLE 0`
- `BRD_OPTIONS 1`
- `LOG_BACKEND_TYPE 0`
- `FRAME_CLASS 1`, `FRAME_TYPE 1`
- `SERIAL3_PROTOCOL 23`, `SERIAL4_PROTOCOL 2`
- `EK3_IMU_MASK 1`
- `SCHED_OPTIONS 1`

Meaning of the active rate/scheduler combination:

- `INS_GYRO_RATE=0` keeps the Invensense backend at 1 kHz (not 2 kHz)
- `FSTRATE_ENABLE=0` disables the dedicated fast-rate thread; attitude runs in the main loop
- `FSTRATE_DIV=3` sets the rate thread period (1000/3 ≈ 333 Hz) for when FSTRATE is enabled
- `SCHED_LOOP_RATE=150` sets the main scheduler target to 150 Hz

## 2. Current Laurel compile-time defines

From `libraries/AP_HAL_ChibiOS/hwdef/Laurel/hwdef.dat`:

- `define HAL_EKF_IMU_MASK_DEFAULT 1`
- `define HAL_FS_MOUNT_RETRY_MS 30000`
- `define DISABLE_WATCHDOG 1` (dual-core dev mode — remove for production)
- `define CH_CFG_SMP_MODE TRUE` / `define RP_CORE1_START TRUE` (dual-core SMP)

From `libraries/AP_HAL_ChibiOS/Storage.cpp`:
- RP2350 uses `healthy_timeout_ms = 30000U` (was 2000U on non-RP2350)

## 3. Parameter persistence

Params persist in flash storage at `0x10008000 .. 0x10009FFF` (8 KB).
After reflashing, runtime params may still come from saved storage, not `defaults.parm`.
Defaults apply only when storage is erased:

```
monitor flash erase_address pad 0x10008000 0x2000
```

## 4. Observed runtime profile (dual-core branch)

After flash + param-storage erase, perf_report() (runs every 10 s) shows:

- `Perf: main=152Hz rate=988Hz c0=74% c1=5%`

Note: `c0` is the core0 (ChibiOS scheduler) CPU load; `c1` is core1 utilisation
(fraction of real time core1 spent executing dispatched functions via c1_busy_us).
At idle/bench (unarmed, no GPS/compass), c1 stays low (~5%) because the
attitude/covariance sidechannel dispatch is minimal without active flight.

Single-core branch equivalent (for comparison): `Perf: main=148Hz rate=988Hz load=73%`

## 5. Key branch changes affecting diagnostics

- `ArduCopter/Copter.cpp`: `perf_report()` now shows `c0=XX% c1=XX%` when `RP_CORE1_START=TRUE`
- `libraries/AP_HAL_ChibiOS/hwdef/common/board_rp2350.c`: dual-core dispatch (c1_run_sync,
  c1_att_dispatch_async, c1_cov_dispatch_async) active under `#if RP_CORE1_START == TRUE`
- `libraries/AP_HAL_ChibiOS/hwdef/common/common_rp2350_smp.ld`:
  `__c1_main_stack_size__ = __main_stack_size__`, `__c1_process_stack_size__ = 0x4000`
- `libraries/AP_HAL_ChibiOS/Scheduler.cpp`: `HAL_FS_MOUNT_RETRY_MS` controls mount retry timing

## 6. One-line current state

Laurel dual-core branch: 1 kHz IMU backend, 150 Hz main scheduler, fast-rate thread
disabled, core1 parked in WFE until dispatch. Bench load: c0≈74%, c1≈5%.
