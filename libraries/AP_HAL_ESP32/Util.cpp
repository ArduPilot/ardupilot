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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit and David "Buzz" Bussenschutt
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"

#include "RCOutput.h"

#include <AP_ROMFS/AP_ROMFS.h>
#include "SdCard.h"

#include <esp_timer.h>
#include <multi_heap.h>
#include <esp_heap_caps.h>

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"


extern const AP_HAL::HAL& hal;

using namespace ESP32;


/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    return heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

}

/*
    Special Allocation Routines
*/

void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{

    // https://docs.espressif.com/projects/esp-idf/en/v4.0.2/api-reference/system/mem_alloc.html
    // esp32 has DRAM, IRAM and D/IRAM that can be used as either

    /*
    DRAM (Data RAM) is memory used to hold data. This is the most common kind of memory accessed as heap.

    IRAM (Instruction RAM) usually holds executable data only. If accessed as generic memory, all accesses must be 32-bit aligned.

    D/IRAM is RAM which can be used as either Instruction or Data RAM.
    */

    //The ESP-IDF malloc() implementation internally calls heap_caps_malloc(size, MALLOC_CAP_8BIT) in order to allocate DRAM that is byte-addressable.

    //For most purposes, the standard libc malloc() and free() functions can be used for heap allocation without any special consideration.
    //	return malloc(size);

    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        return heap_caps_calloc(1, size, MALLOC_CAP_DMA);
        //} else if (mem_type == AP_HAL::Util::MEM_FAST) {
        //   return heap_caps_calloc(1, size, MALLOC_CAP_32BIT); //WARNING 32bit memory cannot use unless 32bit access
    } else {
        return heap_caps_calloc(1, size, MALLOC_CAP_8BIT);
    }
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr != NULL) {
        heap_caps_free(ptr);
    }
}


#ifdef ENABLE_HEAP

void *Util::allocate_heap_memory(size_t size)
{
    void *buf = calloc(1, size);
    if (buf == nullptr) {
        return nullptr;
    }

    multi_heap_handle_t *heap = (multi_heap_handle_t *)calloc(1, sizeof(multi_heap_handle_t));
    if (heap != nullptr) {
        auto hp = multi_heap_register(buf, size);
        memcpy(heap, &hp, sizeof(multi_heap_handle_t));
    }

    return heap;
}

void *Util::heap_realloc(void *heap, void *ptr, size_t old_size, size_t new_size)
{
    if (heap == nullptr) {
        return nullptr;
    }

    return multi_heap_realloc(*(multi_heap_handle_t *)heap, ptr, new_size);
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
        return calloc(1, size);
    }
    void *new_mem = calloc(1, size);
    if (new_mem != nullptr) {
        //memcpy(new_mem, addr, chHeapGetSize(addr) > size ? size : chHeapGetSize(addr));
        memcpy(new_mem, addr, size );
        free(addr);
    }
    return new_mem;
}

#endif // ENABLE_HEAP


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
    //stm32_set_utc_usec(time_utc_usec);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    return esp_timer_get_time();
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
    //    ....esp32 too
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifer - board type and serial number
 */


bool Util::get_system_id(char buf[50])
{
    //uint8_t serialid[12];
    char board_name[] = HAL_ESP32_BOARD_NAME" ";

    uint8_t base_mac_addr[6] = {0};
    esp_err_t ret = esp_efuse_mac_get_custom(base_mac_addr);
    if (ret != ESP_OK) {
        ret = esp_efuse_mac_get_default(base_mac_addr);
    }

    char board_mac[20] = "                   ";
    snprintf(board_mac,20, "%x %x %x %x %x %x",
             base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);

    // null terminate both
    //board_name[13] = 0;
    board_mac[19] = 0;

    // tack strings togehter
    snprintf(buf, 40, "%s %s", board_name, board_mac);
    // and null terminate that too..
    buf[39] = 0;
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    len = MIN(12, len);


    uint8_t base_mac_addr[6] = {0};
    esp_err_t ret = esp_efuse_mac_get_custom(base_mac_addr);
    if (ret != ESP_OK) {
        ret = esp_efuse_mac_get_default(base_mac_addr);
    }

    memcpy(buf, (const void *)base_mac_addr, len);

    return true;
}

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    return false;
    esp_reset_reason_t reason = esp_reset_reason();

    return reason == ESP_RST_PANIC
           || reason == ESP_RST_PANIC
           || reason == ESP_RST_TASK_WDT
           || reason == ESP_RST_WDT;
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

    //    char buffer[1024];
    //    vTaskGetRunTimeStats(buffer);
    //    snprintf(buf, bufsize,"\n\n%s\n", buffer);

    // total = ..

    return total;
}
#endif // CH_DBG_ENABLE_STACK_CHECK == TRUE

