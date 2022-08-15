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

class ExpandingString;

#ifndef HAL_ENABLE_SAVE_PERSISTENT_PARAMS
// on F7 and H7 we will try to save key persistent parameters at the
// end of the bootloader sector. This enables temperature calibration
// data to be saved persistently in the factory
#define HAL_ENABLE_SAVE_PERSISTENT_PARAMS !defined(HAL_BOOTLOADER_BUILD) && !defined(HAL_BUILD_AP_PERIPH) && (defined(STM32F7) || defined(STM32H7))
#endif

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

    bool toneAlarm_init(uint8_t types) override;
#if HAL_USE_PWM == TRUE
    bool toneAlarm_init(const PWMConfig& pwm_cfg, PWMDriver* pwm_drv, pwmchannel_t chan, bool active_high);
#endif
    void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override;
    static uint8_t _toneAlarm_types;

    // return true if the reason for the reboot was a watchdog reset
    bool was_watchdog_reset() const override;

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
    // request information on running threads
    void thread_info(ExpandingString &str) override;
#endif
#if CH_CFG_USE_SEMAPHORES
    // request information on dma contention
    void dma_info(ExpandingString &str) override;
#endif
#if CH_CFG_USE_HEAP == TRUE
    void mem_info(ExpandingString &str) override;
#endif

#if HAL_ENABLE_SAVE_PERSISTENT_PARAMS
    // apply persistent parameters to current parameters
    void apply_persistent_params(void) const;
#endif

#if HAL_ENABLE_SAVE_PERSISTENT_PARAMS
    // save/load key persistent parameters in bootloader sector
    bool load_persistent_params(ExpandingString &str) const override;
#endif
#if HAL_UART_STATS_ENABLED
    // request information on uart I/O
    virtual void uart_info(ExpandingString &str) override;
#endif
#if HAL_USE_PWM == TRUE
    void timer_info(ExpandingString &str) override;
#endif
    // returns random values
    bool get_random_vals(uint8_t* data, size_t size) override;

    // returns true random values
    bool get_true_random_vals(uint8_t* data, size_t size, uint32_t timeout_us) override;

    // set armed state
    void set_soft_armed(const bool b) override;

private:
#if HAL_USE_PWM == TRUE
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

#if HAL_ENABLE_SAVE_PERSISTENT_PARAMS
    // save/load key persistent parameters in bootloader sector
    bool get_persistent_params(ExpandingString &str) const;
#endif

    // log info on stack usage
    void log_stack_info(void) override;

#if AP_CRASHDUMP_ENABLED
    // get last crash dump
    size_t last_crash_dump_size() const override;
    void* last_crash_dump_ptr() const override;
#endif

};
