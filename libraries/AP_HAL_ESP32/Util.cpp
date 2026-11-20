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
#ifdef HAL_ESP32_TONEALARM_PIN
#include <driver/gpio.h>
#include <driver/ledc.h>
#endif


#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <AP_Common/ExpandingString.h>

#include "esp_mac.h"

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

#ifdef HAL_ESP32_TONEALARM_PIN

namespace {
static constexpr ledc_mode_t TONEALARM_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t TONEALARM_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t TONEALARM_LEDC_CHANNEL = LEDC_CHANNEL_5;
static constexpr ledc_timer_bit_t TONEALARM_LEDC_RESOLUTION = LEDC_TIMER_8_BIT;
static constexpr uint32_t TONEALARM_DEFAULT_FREQUENCY_HZ = 4000;
static constexpr uint32_t TONEALARM_MAX_DUTY = (1U << 8) - 1U;
static constexpr uint8_t TONEALARM_TYPE_BUILTIN = 1U << 0;
}

bool Util::toneAlarm_init(uint8_t types)
{
    // AP_Notify::BuzzerType::BUILTIN is bit 0. Keep AP_Notify out of the HAL.
    if ((types & TONEALARM_TYPE_BUILTIN) == 0) {
        _toneAlarm_initialized = false;
        return true;
    }

    ledc_timer_config_t timer_config {};
    timer_config.speed_mode = TONEALARM_LEDC_MODE;
    timer_config.timer_num = TONEALARM_LEDC_TIMER;
    timer_config.duty_resolution = TONEALARM_LEDC_RESOLUTION;
    timer_config.freq_hz = TONEALARM_DEFAULT_FREQUENCY_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    if (ledc_timer_config(&timer_config) != ESP_OK) {
        return false;
    }

    ledc_channel_config_t channel_config {};
    channel_config.gpio_num = HAL_ESP32_TONEALARM_PIN;
    channel_config.speed_mode = TONEALARM_LEDC_MODE;
    channel_config.channel = TONEALARM_LEDC_CHANNEL;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = TONEALARM_LEDC_TIMER;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    if (ledc_channel_config(&channel_config) != ESP_OK) {
        return false;
    }

    _toneAlarm_initialized = true;
    return true;
}

void Util::toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms)
{
    if (!_toneAlarm_initialized) {
        return;
    }

    (void)duration_ms;

    if (frequency <= 0.0f || volume <= 0.0f) {
        ledc_set_duty(TONEALARM_LEDC_MODE, TONEALARM_LEDC_CHANNEL, 0);
        ledc_update_duty(TONEALARM_LEDC_MODE, TONEALARM_LEDC_CHANNEL);
        return;
    }

    uint32_t frequency_hz = uint32_t(roundf(frequency));
    if (frequency_hz == 0) {
        frequency_hz = 1;
    }

#ifdef HAL_ESP32_TONEALARM_MIN_FREQ_HZ
    while (frequency_hz < HAL_ESP32_TONEALARM_MIN_FREQ_HZ) {
        frequency_hz *= 2U;
    }
#endif
#ifdef HAL_ESP32_TONEALARM_MAX_FREQ_HZ
    while (frequency_hz > HAL_ESP32_TONEALARM_MAX_FREQ_HZ) {
        frequency_hz /= 2U;
    }
#endif

    ledc_set_freq(TONEALARM_LEDC_MODE, TONEALARM_LEDC_TIMER, frequency_hz);

    const float volume_limited = volume > 1.0f ? 1.0f : volume;
    const uint32_t duty = uint32_t(roundf(volume_limited * TONEALARM_MAX_DUTY * 0.5f));
    ledc_set_duty(TONEALARM_LEDC_MODE, TONEALARM_LEDC_CHANNEL, duty);
    ledc_update_duty(TONEALARM_LEDC_MODE, TONEALARM_LEDC_CHANNEL);
}
#endif // HAL_ESP32_TONEALARM_PIN

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

#if !HAL_GCS_ENABLED
#define Debug(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while (0)
#else
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

Util::FlashBootloader Util::flash_bootloader()
{
    //    ....esp32 too
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifier - board type and serial number
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

    // tack strings together
    snprintf(buf, 40, "%s %s", board_name, board_mac);
    // and null terminate that too..
    buf[39] = 0;
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    uint8_t base_mac_addr[6] = {0};
    esp_err_t ret = esp_efuse_mac_get_custom(base_mac_addr);
    if (ret != ESP_OK) {
        ret = esp_efuse_mac_get_default(base_mac_addr);
    }

    len = MIN(len, ARRAY_SIZE(base_mac_addr));
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

/*
  display stack usage as text buffer for @SYS/threads.txt
 */
void Util::thread_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("ThreadsV1\n");

    //    char buffer[1024];
    //    vTaskGetRunTimeStats(buffer);
    //    snprintf(buf, bufsize,"\n\n%s\n", buffer);
}


