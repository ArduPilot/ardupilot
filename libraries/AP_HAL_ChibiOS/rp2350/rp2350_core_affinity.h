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

   RP2350 SMP core assignments for named threads.
 */
/*
  The core each named thread runs on: 0 or 1. thread_create_alloc() looks a
  new thread's name up here, which covers threads made with
  hal.scheduler->thread_create(), the SPIn/I2Cn bus threads and the UART
  threads, and the rcout thread looks up "rcout". A thread that is not listed
  starts on the core that creates it. The timer, monitor, RCIN, IO and storage
  threads are created statically on core0 and are not looked up, so listing
  them does nothing.

  A board replaces the whole table with its own in hwdef.dat, for example
    define AP_THREAD_CORE_TABLE { "rate", 1 }, { "rcout", 1 }, { "SPI0", 0 }
  A board that puts "rate" on core0 with the main loop also needs
    define AP_RATE_THREAD_SEPARATE_CORE 0

  ChibiOS routes a peripheral's interrupt to the core that calls the driver
  start function (spiStart, i2cStart, sdStart, ...). A bus is restarted by
  whichever thread next changes its configuration, so the interrupt reaches
  the bus thread's core the first time that thread does so.
  See libraries/AP_HAL_ChibiOS/rp2350/DEVELOPMENT.md.
*/
#pragma once

#ifndef AP_THREAD_CORE_TABLE
#define AP_THREAD_CORE_TABLE \
    { "rate",   1 },    /* the vehicle rate thread */ \
    { "rcout",  1 },    /* with the rate thread: motor demands stay in SRAM */ \
    { "SPI0",   1 },    /* IMU bus, alongside the rate thread */ \
    { "OSD_c1", 1 },    /* PIO OSD scan-out */ \
    /* the other buses stay on core0 even when first used from core1 */ \
    { "SPI1",   0 }, \
    { "I2C0",   0 }, \
    { "I2C1",   0 }
#endif
