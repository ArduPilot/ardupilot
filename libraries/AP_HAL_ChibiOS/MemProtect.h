/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  trap CPU reads and writes to the reserved first 1k of memory on H7 using a
  no-access MPU region. The MemManage handler records where the access came
  from, disarms the region and returns, so the access completes and the vehicle
  keeps flying. The storage thread logs the record and re-arms the region.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if AP_BOARDCONFIG_MCU_MEMPROTECT_TRACE_ENABLED

#include <stdint.h>

// give up permanently after this many traps so a hot loop can't soak the CPU
#ifndef AP_MEMPROTECT_MAX_HITS
#define AP_MEMPROTECT_MAX_HITS 100
#endif

// stack words examined by the heuristic backtrace
#ifndef AP_MEMPROTECT_SCAN_WORDS
#define AP_MEMPROTECT_SCAN_WORDS 128
#endif

// return addresses kept. 6 is the most the log format has room for
#ifndef AP_MEMPROTECT_BT_LEN
#define AP_MEMPROTECT_BT_LEN 6
#endif

#define AP_MEMPROTECT_SIZE 1024U

#ifdef __cplusplus
extern "C" {
#endif

/*
  diagnostics for one trapped access. Written by MemManage_Handler at exception
  priority 0, read by the monitor thread. The handler publishes the payload with
  a barrier before setting pending, so only the flags need to be volatile.
 */
struct memprotect_state_t {
    uint32_t fault_addr;
    uint32_t pc;
    uint32_t lr;
    uint32_t exc_return;
    uint32_t backtrace[AP_MEMPROTECT_BT_LEN];
    char thread_name[16];
    uint8_t backtrace_count;
    uint8_t ipsr;

    volatile uint16_t hit_count;
    volatile bool pending;      // handler has new data for the storage thread
    volatile bool armed;        // MPU region is enabled
    volatile bool latched_off;  // hit AP_MEMPROTECT_MAX_HITS, never re-arm
    volatile bool initialised;  // memprotect_init() has run
};

extern struct memprotect_state_t memprotect_state;

// enable the MPU and arm the region. Call once at startup
void memprotect_init(void);

/*
  arm and disarm the region. Only the monitor thread may call these, so that
  there is a single owner of the region state
 */
void memprotect_arm(void);
void memprotect_disarm(void);

#ifdef __cplusplus
}
#endif

#endif  // AP_BOARDCONFIG_MCU_MEMPROTECT_TRACE_ENABLED
