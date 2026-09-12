/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   RP2350 SMP core assignments for the bus and driver threads.
 */
/*
  rp2350_core_affinity.h -- per-thread core assignment for RP2350 SMP.

  Change a single define to move a bus thread (and its peripheral IRQs) between
  cores. See libraries/AP_HAL_ChibiOS/hwdef/Laurel/ANY_THREAD_ANY_CORE.md.
  Valid values: 0 (Core0) or 1 (Core1). Only threads that something actually
  reads a define for are listed: RCOUT in Scheduler.cpp, and the SPI/I2C bus
  threads in Device.cpp. Every other thread is created on Core0 by a plain
  chThdCreateStatic(), so adding a define for one here would silently do
  nothing.
  Default: all on Core0 except the SPI0 (IMU) bus thread and rcout, which share
  Core1 with the rate thread. The rate thread itself is pinned by the vehicle
  through thread_create_pinned_to_core(), not from here, and there is no
  separate EKF thread - EKF3 runs inline in the Core0 main loop.
  IRQ routing is automatic: ChibiOS HAL routes a peripheral's IRQ to whichever
  core calls the driver start function (spiStart, i2cStart, sdStart, etc.).
  Threads do their own peripheral init on first wakeup, so the IRQ follows the
  thread automatically -- no manual NVIC configuration needed.
*/
#pragma once

#if defined(RP2350) && CH_CFG_SMP_MODE == TRUE
#define HAL_CORE_RCOUT    1   // with the rate thread: motor demands stay in SRAM
#define HAL_CORE_SPI0     1   // IMU bus, alongside the rate thread
#define HAL_CORE_SPI1     0
#define HAL_CORE_I2C0     0
#define HAL_CORE_I2C1     0

#endif // RP2350 && CH_CFG_SMP_MODE
