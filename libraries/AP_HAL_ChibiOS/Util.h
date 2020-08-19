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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include "AP_HAL_ChibiOS.h"
#include <ch.h>

class ChibiOS::Util : public AP_HAL::Util {
public:
    static Util *from(AP_HAL::Util *util) {
        return static_cast<Util*>(util);
    }

    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return false; }
    uint32_t available_memory() override;

    // Special Allocation Routines
    void *malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type) override;
    void free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type) override;

#ifdef ENABLE_HEAP
    // heap functions, note that a heap once alloc'd cannot be dealloc'd
    virtual void *allocate_heap_memory(size_t size) override;
    virtual void *heap_realloc(void *heap, void *ptr, size_t new_size) override;
    virtual void *std_realloc(void *ptr, size_t new_size) override;
#endif // ENABLE_HEAP

    /*
      return state of safety switch, if applicable
     */
    enum safety_state safety_switch_state(void) override;

    // get system ID as a string
    bool get_system_id(char buf[40]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;

#ifdef HAL_PWM_ALARM
    bool toneAlarm_init() override;
    void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override;
#endif

#ifdef USE_POSIX
    /*
      initialise (or re-initialise) filesystem storage
     */
    bool fs_init(void) override;
#endif

    // return true if the reason for the reboot was a watchdog reset
    bool was_watchdog_reset() const override;

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
    // request information on running threads
    size_t thread_info(char *buf, size_t bufsize) override;
#endif
    
private:
#ifdef HAL_PWM_ALARM
    struct ToneAlarmPwmGroup {
        pwmchannel_t chan;
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
    };

    static ToneAlarmPwmGroup _toneAlarm_pwm_group;
#endif

    /*
      set HW RTC in UTC microseconds
     */
    void set_hw_rtc(uint64_t time_utc_usec) override;

    /*
      get system clock in UTC microseconds
     */
    uint64_t get_hw_rtc() const override;
#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    FlashBootloader flash_bootloader() override;
#endif

#ifdef ENABLE_HEAP
    static memory_heap_t scripting_heap;
#endif // ENABLE_HEAP

    // stm32F4 and F7 have 20 total RTC backup registers. We use the first one for boot type
    // flags, so 19 available for persistent data
    static_assert(sizeof(persistent_data) <= 19*4, "watchdog persistent data too large");
};
