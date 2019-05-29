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
#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void stm32_timer_set_input_filter(stm32_tim_t *tim, uint8_t channel, uint8_t filter_mode);
void stm32_timer_set_channel_input(stm32_tim_t *tim, uint8_t channel, uint8_t input_source);

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
// print stack usage
void show_stack_usage(void);
#endif

// allocation functions in malloc.c    
size_t mem_available(void);
void *malloc_dma(size_t size);
void *malloc_sdcard_dma(size_t size);
void *malloc_fastmem(size_t size);
thread_t *thread_create_alloc(size_t size, const char *name, tprio_t prio, tfunc_t pf, void *arg);

// flush all dcache
void memory_flush_all(void);
    
// UTC system clock handling    
void stm32_set_utc_usec(uint64_t time_utc_usec);
uint64_t stm32_get_utc_usec(void);

// hook for FAT timestamps    
uint32_t get_fattime(void);

// one-time programmable area
#if defined(STM32F4)
#define OTP_BASE 0x1fff7800
#define OTP_SIZE 512
#elif defined(STM32F7)
#define OTP_BASE 0x1ff0f000
#define OTP_SIZE 1024
#endif

enum rtc_boot_magic {
    RTC_BOOT_OFF  = 0,
    RTC_BOOT_HOLD = 0xb0070001,
    RTC_BOOT_FAST = 0xb0070002
};
    
// see if RTC registers is setup for a fast reboot
enum rtc_boot_magic check_fast_reboot(void);

// set RTC register for a fast reboot
void set_fast_reboot(enum rtc_boot_magic v);

// enable peripheral power if needed
void peripheral_power_enable(void);

// initialise allocation subsystem
void malloc_init(void);

/*
  read mode of a pin. This allows a pin config to be read, changed and
  then written back
 */
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4)
iomode_t palReadLineMode(ioline_t line);
#endif

// set n RTC backup registers starting at given idx
void set_rtc_backup(uint8_t idx, const uint32_t *v, uint8_t n);

// get RTC backup registers starting at given idx
void get_rtc_backup(uint8_t idx, uint32_t *v, uint8_t n);

#ifdef __cplusplus
}
#endif

