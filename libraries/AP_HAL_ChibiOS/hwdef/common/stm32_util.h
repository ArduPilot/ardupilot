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
void *malloc_axi_sram(size_t size);
void *malloc_fastmem(size_t size);
thread_t *thread_create_alloc(size_t size, const char *name, tprio_t prio, tfunc_t pf, void *arg);

struct memory_region {
    void *address;
    uint32_t size;
    uint32_t flags;
};
#if CH_CFG_USE_HEAP == TRUE
uint8_t malloc_get_heaps(memory_heap_t **_heaps, const struct memory_region **regions);
#endif

// flush all dcache
void memory_flush_all(void);
    
// UTC system clock handling    
void stm32_set_utc_usec(uint64_t time_utc_usec);
uint64_t stm32_get_utc_usec(void);

// hook for FAT timestamps    
uint32_t get_fattime(void);

// one-time programmable area
#if defined(FLASH_OTP_BASE)
#define OTP_BASE FLASH_OTP_BASE
#define OTP_SIZE (FLASH_OTP_END-FLASH_OTP_BASE)
#elif defined(STM32F4)
#define OTP_BASE 0x1fff7800
#define OTP_SIZE 512
#elif defined(STM32F7)
#define OTP_BASE 0x1ff0f000
#define OTP_SIZE 1024
#endif

enum rtc_boot_magic {
    RTC_BOOT_OFF  = 0,
    RTC_BOOT_HOLD = 0xb0070001,
    RTC_BOOT_FAST = 0xb0070002,
    RTC_BOOT_CANBL = 0xb0080000, // ORd with 8 bit local node ID
    RTC_BOOT_FWOK = 0xb0093a26 // indicates FW ran for 30s
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
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32F3) || defined(STM32G4)
iomode_t palReadLineMode(ioline_t line);

enum PalPushPull {
    PAL_PUSHPULL_NOPULL=0,
    PAL_PUSHPULL_PULLUP=1,
    PAL_PUSHPULL_PULLDOWN=2
};

void palLineSetPushPull(ioline_t line, enum PalPushPull pp);
#endif

// set n RTC backup registers starting at given idx
void set_rtc_backup(uint8_t idx, const uint32_t *v, uint8_t n);

// get RTC backup registers starting at given idx
void get_rtc_backup(uint8_t idx, uint32_t *v, uint8_t n);

void stm32_cacheBufferInvalidate(const void *p, size_t size);
void stm32_cacheBufferFlush(const void *p, size_t size);

#ifdef HAL_GPIO_PIN_FAULT
// printf for fault handlers
void fault_printf(const char *fmt, ...);
#endif

// halt hook for printing panic message
void system_halt_hook(void);

// hook for stack overflow
void stack_overflow(thread_t *tp);

/*
  check how much stack is free given a stack base. Assumes the fill
  byte is 0x55
 */
uint32_t stack_free(void *stack_base);

// allow stack view code to show free ISR stack
extern uint32_t __main_stack_base__;
extern uint32_t __main_stack_end__;
extern uint32_t __main_thread_stack_base__;
extern uint32_t __main_thread_stack_end__;

#ifdef __cplusplus
}
#endif

