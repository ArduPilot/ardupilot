/*
  rp2350_core_affinity.h — per-thread core assignment for RP2350 SMP.

  Change a single define to move a thread (and its peripheral IRQs) between
  cores. See libraries/AP_HAL_ChibiOS/hwdef/Laurel/ANY_THREAD_ANY_CORE.md.
  Valid values: 0 (Core0) or 1 (Core1).
  Default: all on Core0 except rate and EKF which belong on Core1.
  IRQ routing is automatic: ChibiOS HAL routes a peripheral's IRQ to whichever
  core calls the driver start function (spiStart, i2cStart, sdStart, etc.).
  Threads do their own peripheral init on first wakeup, so the IRQ follows the
  thread automatically — no manual NVIC configuration needed.
*/
#pragma once

#if defined(RP2350) && CH_CFG_SMP_MODE == TRUE
#define HAL_CORE_TIMER    0
#define HAL_CORE_RCOUT    0
#define HAL_CORE_RCIN     0
#define HAL_CORE_IO       0
#define HAL_CORE_STORAGE  0   // MUST stay Core0: XIP lockout protocol parks Core1 FROM Core0
#define HAL_CORE_SPI0     1   // Best Config: SPI on Core1 + DCM/8 + ekf_decim_min=2
#define HAL_CORE_SPI1     1
#define HAL_CORE_I2C0     0
#define HAL_CORE_I2C1     0
#define HAL_CORE_UART     0
#define HAL_CORE_USB      0
#define HAL_CORE_RATE     1   // rate thread — keep on Core1
#define HAL_CORE_EKF      1   // EKF thread  — keep on Core1

#endif // RP2350 && CH_CFG_SMP_MODE
