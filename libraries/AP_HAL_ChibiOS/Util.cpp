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
#include "hwdef/common/flash.h"
#include <AP_ROMFS/AP_ROMFS.h>

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
        return try_alloc_from_ccm_ram(size);
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


void* Util::try_alloc_from_ccm_ram(size_t size)
{
    void *ret = malloc_ccm(size);
    if (ret == nullptr) {
        //we failed to allocate from CCM so we are going to try common SRAM
        ret = calloc(1, size);
    }
    return ret;
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

void Util::set_imu_temp(float current)
{
#if HAL_WITH_IO_MCU && HAL_HAVE_IMU_HEATER
    if (!heater.target || *heater.target == -1 || !AP_BoardConfig::io_enabled()) {
        return;
    }

    // average over temperatures to remove noise
    heater.count++;
    heater.sum += current;
    
    // update once a second
    uint32_t now = AP_HAL::millis();
    if (now - heater.last_update_ms < 1000) {
        return;
    }
    heater.last_update_ms = now;

    current = heater.sum / heater.count;
    heater.sum = 0;
    heater.count = 0;

    // experimentally tweaked for Pixhawk2
    const float kI = 0.3f;
    const float kP = 200.0f;
    float target = (float)(*heater.target);

    // limit to 65 degrees to prevent damage
    target = constrain_float(target, 0, 65);
    
    float err = target - current;

    heater.integrator += kI * err;
    heater.integrator = constrain_float(heater.integrator, 0, 70);

    float output = constrain_float(kP * err + heater.integrator, 0, 100);
    
    // hal.console->printf("integrator %.1f out=%.1f temp=%.2f err=%.2f\n", heater.integrator, output, current, err);

    iomcu.set_heater_duty_cycle(output);
#endif // HAL_WITH_IO_MCU && HAL_HAVE_IMU_HEATER
}

void Util::set_imu_target_temp(int8_t *target)
{
#if HAL_WITH_IO_MCU && HAL_HAVE_IMU_HEATER
    heater.target = target;
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

#ifndef HAL_NO_FLASH_SUPPORT

bool Util::flash_bootloader()
{
    uint32_t fw_size;
    const char *fw_name = "bootloader.bin";

    uint8_t *fw = AP_ROMFS::find_decompress(fw_name, fw_size);
    if (!fw) {
        hal.console->printf("failed to find %s\n", fw_name);
        return false;
    }

    const uint32_t addr = stm32_flash_getpageaddr(0);
    if (!memcmp(fw, (const void*)addr, fw_size)) {
        hal.console->printf("Bootloader up-to-date\n");
        free(fw);
        return true;
    }

    hal.console->printf("Erasing\n");
    if (!stm32_flash_erasepage(0)) {
        hal.console->printf("Erase failed\n");
        free(fw);
        return false;
    }
    hal.console->printf("Flashing %s @%08x\n", fw_name, (unsigned int)addr);
    const uint8_t max_attempts = 10;
    for (uint8_t i=0; i<max_attempts; i++) {
        void *context = hal.scheduler->disable_interrupts_save();
        const int32_t written = stm32_flash_write(addr, fw, fw_size);
        hal.scheduler->restore_interrupts(context);
        if (written == -1 || written < fw_size) {
            hal.console->printf("Flash failed! (attempt=%u/%u)\n",
                                i+1,
                                max_attempts);
            hal.scheduler->delay(1000);
            continue;
        }
        hal.console->printf("Flash OK\n");
        free(fw);
        return true;
    }

    hal.console->printf("Flash failed after %u attempts\n", max_attempts);
    free(fw);
    return false;
}
#endif //#ifndef HAL_NO_FLASH_SUPPORT

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
