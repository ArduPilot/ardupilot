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
#include "Semaphores.h"
#include "ToneAlarm.h"

// this checks an address is in main memory and 16 bit aligned
#define IS_DMA_SAFE(addr) ((uint32_t(addr) & 0xF0000001) == 0x20000000)

class ChibiOS::Util : public AP_HAL::Util {
public:
    static Util *from(AP_HAL::Util *util) {
        return static_cast<Util*>(util);
    }

    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
    AP_HAL::Semaphore *new_semaphore(void) override { return new ChibiOS::Semaphore; }
    uint32_t available_memory() override;

    // Special Allocation Routines
    void *malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type);
    void free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type);

    /*
      return state of safety switch, if applicable
     */
    enum safety_state safety_switch_state(void) override;

    // IMU temperature control
    void set_imu_temp(float current);
    void set_imu_target_temp(int8_t *target);

#ifdef HAL_PWM_ALARM
    bool toneAlarm_init();
    void toneAlarm_set_tune(uint8_t tone);
    void _toneAlarm_timer_tick();

    static ToneAlarm& get_ToneAlarm() { return _toneAlarm; }
#endif

private:
#ifdef HAL_PWM_ALARM
    static ToneAlarm _toneAlarm;
#endif
    void* try_alloc_from_ccm_ram(size_t size);
    uint32_t available_memory_in_ccm_ram(void);

#if HAL_WITH_IO_MCU && HAL_HAVE_IMU_HEATER
    struct {
        int8_t *target;
        float integrator;
        uint16_t count;
        float sum;
        uint32_t last_update_ms;
    } heater;
#endif
};
