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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"
#include <ch.h>
#include "RCOutput.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"
#include "hwdef/common/flash.h"
#include <AP_ROMFS/AP_ROMFS.h>
#include "sdcard.h"

#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;
#if CH_CFG_USE_HEAP == TRUE

/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    // from malloc.c in hwdef
    return mem_available();
}

/*
    Special Allocation Routines
*/

void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        return malloc_dma(size);
    } else if (mem_type == AP_HAL::Util::MEM_FAST) {
        return malloc_fastmem(size);
    } else {
        return calloc(1, size);
    }
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr != NULL) {
        chHeapFree(ptr);
    }
}


#ifdef ENABLE_HEAP

void *Util::allocate_heap_memory(size_t size)
{
    void *buf = malloc(size);
    if (buf == nullptr) {
        return nullptr;
    }

    memory_heap_t *heap = (memory_heap_t *)malloc(sizeof(memory_heap_t));
    if (heap != nullptr) {
        chHeapObjectInit(heap, buf, size);
    }

    return heap;
}

/*
  realloc implementation thanks to wolfssl, used by AP_Scripting
 */
void *Util::std_realloc(void *addr, size_t size)
{
    if (size == 0) {
       free(addr);
       return nullptr;
    }
    if (addr == nullptr) {
        return malloc(size);
    }
    void *new_mem = malloc(size);
    if (new_mem != nullptr) {
        memcpy(new_mem, addr, chHeapGetSize(addr) > size ? size : chHeapGetSize(addr));
        free(addr);
    }
    return new_mem;
}

void *Util::heap_realloc(void *heap, void *ptr, size_t new_size)
{
    if (heap == nullptr) {
        return nullptr;
    }
    if (new_size == 0) {
        if (ptr != nullptr) {
            chHeapFree(ptr);
        }
        return nullptr;
    }
    if (ptr == nullptr) {
        return chHeapAlloc((memory_heap_t *)heap, new_size);
    }
    void *new_mem = chHeapAlloc((memory_heap_t *)heap, new_size);
    if (new_mem != nullptr) {
        memcpy(new_mem, ptr, chHeapGetSize(ptr) > new_size ? new_size : chHeapGetSize(ptr));
        chHeapFree(ptr);
    }
    return new_mem;
}
#endif // ENABLE_HEAP

#endif // CH_CFG_USE_HEAP

/*
  get safety switch state
 */
Util::safety_state Util::safety_switch_state(void)
{
#if HAL_USE_PWM == TRUE
    return ((RCOutput *)hal.rcout)->_safety_switch_state();
#else
    return SAFETY_NONE;
#endif
}

#ifdef HAL_PWM_ALARM
struct Util::ToneAlarmPwmGroup Util::_toneAlarm_pwm_group = HAL_PWM_ALARM;

bool Util::toneAlarm_init()
{
    _toneAlarm_pwm_group.pwm_cfg.period = 1000;
    pwmStart(_toneAlarm_pwm_group.pwm_drv, &_toneAlarm_pwm_group.pwm_cfg);

    return true;
}

void Util::toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms)
{
    if (is_zero(frequency) || is_zero(volume)) {
        pwmDisableChannel(_toneAlarm_pwm_group.pwm_drv, _toneAlarm_pwm_group.chan);
    } else {
        pwmChangePeriod(_toneAlarm_pwm_group.pwm_drv,
                        roundf(_toneAlarm_pwm_group.pwm_cfg.frequency/frequency));

        pwmEnableChannel(_toneAlarm_pwm_group.pwm_drv, _toneAlarm_pwm_group.chan, roundf(volume*_toneAlarm_pwm_group.pwm_cfg.frequency/frequency)/2);
    }
}
#endif // HAL_PWM_ALARM

/*
  set HW RTC in UTC microseconds
*/
void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    stm32_set_utc_usec(time_utc_usec);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    return stm32_get_utc_usec();
}

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)

#if defined(HAL_NO_GCS) || defined(HAL_BOOTLOADER_BUILD)
#define Debug(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while (0)
#else
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

Util::FlashBootloader Util::flash_bootloader()
{
    uint32_t fw_size;
    const char *fw_name = "bootloader.bin";

    EXPECT_DELAY_MS(11000);

    const uint8_t *fw = AP_ROMFS::find_decompress(fw_name, fw_size);
    if (!fw) {
        Debug("failed to find %s\n", fw_name);
        return FlashBootloader::NOT_AVAILABLE;
    }
    // make sure size is multiple of 32
    fw_size = (fw_size + 31U) & ~31U;

    const uint32_t addr = hal.flash->getpageaddr(0);
    if (!memcmp(fw, (const void*)addr, fw_size)) {
        Debug("Bootloader up-to-date\n");
        AP_ROMFS::free(fw);
        return FlashBootloader::NO_CHANGE;
    }

    Debug("Erasing\n");
    uint32_t erased_size = 0;
    uint8_t erase_page = 0;
    while (erased_size < fw_size) {
        uint32_t page_size = hal.flash->getpagesize(erase_page);
        if (page_size == 0) {
            AP_ROMFS::free(fw);
            return FlashBootloader::FAIL;
        }
        hal.scheduler->expect_delay_ms(1000);
        if (!hal.flash->erasepage(erase_page)) {
            Debug("Erase %u failed\n", erase_page);
            AP_ROMFS::free(fw);
            return FlashBootloader::FAIL;
        }
        erased_size += page_size;
        erase_page++;
    }

    Debug("Flashing %s @%08x\n", fw_name, (unsigned int)addr);
    const uint8_t max_attempts = 10;
    hal.flash->keep_unlocked(true);
    for (uint8_t i=0; i<max_attempts; i++) {
        hal.scheduler->expect_delay_ms(1000);
        bool ok = hal.flash->write(addr, fw, fw_size);
        if (!ok) {
            Debug("Flash failed! (attempt=%u/%u)\n",
                                i+1,
                                max_attempts);
            hal.scheduler->delay(100);
            continue;
        }
        Debug("Flash OK\n");
        hal.flash->keep_unlocked(false);
        AP_ROMFS::free(fw);
        return FlashBootloader::OK;
    }

    hal.flash->keep_unlocked(false);
    Debug("Flash failed after %u attempts\n", max_attempts);
    AP_ROMFS::free(fw);
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifer - board type and serial number
 */
bool Util::get_system_id(char buf[40])
{
    uint8_t serialid[12];
    char board_name[14];

    memcpy(serialid, (const void *)UDID_START, 12);
    strncpy(board_name, CHIBIOS_SHORT_BOARD_NAME, 13);
    board_name[13] = 0;

    // this format is chosen to match the format used by HAL_PX4
    snprintf(buf, 40, "%s %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
             board_name,
             (unsigned)serialid[3], (unsigned)serialid[2], (unsigned)serialid[1], (unsigned)serialid[0],
             (unsigned)serialid[7], (unsigned)serialid[6], (unsigned)serialid[5], (unsigned)serialid[4],
             (unsigned)serialid[11], (unsigned)serialid[10], (unsigned)serialid[9],(unsigned)serialid[8]);
    buf[39] = 0;
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    len = MIN(12, len);
    memcpy(buf, (const void *)UDID_START, len);
    return true;
}

#ifdef USE_POSIX
/*
  initialise filesystem
 */
bool Util::fs_init(void)
{
    return sdcard_retry();
}
#endif

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    return stm32_was_watchdog_reset();
}

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
/*
  display stack usage as text buffer for @SYS/threads.txt
 */
size_t Util::thread_info(char *buf, size_t bufsize)
{
  thread_t *tp;
  size_t total = 0;

  // a header to allow for machine parsers to determine format
  int n = snprintf(buf, bufsize, "ThreadsV1\n");
  if (n <= 0) {
      return 0;
  }
  buf += n;
  bufsize -= n;
  total += n;

  tp = chRegFirstThread();

  do {
      uint32_t stklimit = (uint32_t)tp->wabase;
      uint8_t *p = (uint8_t *)tp->wabase;
      while (*p == CH_DBG_STACK_FILL_VALUE) {
          p++;
      }
      uint32_t stack_left = ((uint32_t)p) - stklimit;
      n = snprintf(buf, bufsize, "%-13.13s PRI=%3u STACK_LEFT=%u\n", tp->name, unsigned(tp->prio), unsigned(stack_left));
      if (n <= 0) {
          break;
      }
      buf += n;
      bufsize -= n;
      total += n;
      tp = chRegNextThread(tp);
  } while (tp != NULL);

  return total;
}
#endif // CH_DBG_ENABLE_STACK_CHECK == TRUE

