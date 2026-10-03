/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  MAVLink commands for deliberately crashing or locking up the
  autopilot, used for testing watchdog and crash dump handling
 */

#include "GCS_config.h"

#if HAL_GCS_ENABLED && AP_MAVLINK_FAILURE_CREATION_ENABLED

#include "GCS.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <StorageManager/StorageManager.h>

extern const AP_HAL::HAL& hal;

// F1 has a different EXTI register layout
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(STM32F1)
#include <hal.h>
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
#define EXTI_SWIER_REG EXTI->SWIER
#else
#define EXTI_SWIER_REG EXTI->SWIER1
#endif
#if defined(STM32_EXTI_ENHANCED)
#define EXTI_CPU EXTI_D1
#else
#define EXTI_CPU EXTI
#endif

static uint32_t storm_mask;

// re-pend our EXTI line so the ISR never stops running
static void interrupt_storm_cb(void *arg)
{
    EXTI_SWIER_REG = storm_mask;
}

/*
  create an EXTI software interrupt storm on an unused EXTI channel,
  starving all lower priority interrupts and threads. Used to test
  that lockups due to interrupt starvation produce a crash dump
 */
static bool create_interrupt_storm()
{
    chSysLock();
    for (uint8_t pad=0; pad<16; pad++) {
        const uint32_t mask = 1U<<pad;
        const ioline_t line = PAL_LINE(GPIOA, pad);
        const palevent_t *pep = pal_lld_get_line_event(line);
        if (pep->cb != nullptr ||
            ((EXTI->RTSR1 | EXTI->FTSR1 | EXTI_CPU->IMR1 | EXTI_CPU->EMR1) & mask) != 0) {
            continue;
        }
        storm_mask = mask;
        palSetLineCallbackI(line, interrupt_storm_cb, nullptr);
        palEnableLineEventI(line, PAL_EVENT_MODE_RISING_EDGE);
        EXTI_SWIER_REG = mask;
        chSysUnlock();
        // we may never get here
        return true;
    }
    chSysUnlock();
    return false;
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(STM32F1)

/*
  handle a crash trigger, sent as MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
  with params 42/24/71 and the trigger type in param4. Returns
  MAV_RESULT_UNSUPPORTED if param4 is not a crash trigger
 */
MAV_RESULT GCS_MAVLINK::handle_crash_trigger(const mavlink_command_int_t &packet)
{
    if (is_equal(packet.param4, 93.0f)) {
        // this is a magic sequence to force the main loop to
        // lockup. This is for testing the stm32 watchdog
        // functionality
        while (true) {
            send_text(MAV_SEVERITY_WARNING,"entering lockup");
            hal.scheduler->delay(250);
        }
    }
    if (is_equal(packet.param4, 94.0f)) {
        // the following text is unlikely to make it out...
        send_text(MAV_SEVERITY_WARNING,"dereferencing a bad thing");

#if CONFIG_HAL_BOARD != HAL_BOARD_ESP32
// esp32 can't do this bit, skip it, return an error
        void *foo = (void*)0xE000ED38;

        typedef void (*fptr)();
        fptr gptr = (fptr) (void *) foo;
        gptr();
#endif
        return MAV_RESULT_FAILED;
    }
    if (is_equal(packet.param4, 95.0f)) {
        // the following text is unlikely to make it out...
        send_text(MAV_SEVERITY_WARNING,"calling AP_HAL::panic(...)");

        AP_HAL::panic("panicing");

        // keep calm and carry on
    }
    if (is_equal(packet.param4, 96.0f)) {
        // deliberately corrupt parameter storage
        send_text(MAV_SEVERITY_WARNING,"wiping parameter storage header");
        StorageAccess param_storage{StorageManager::StorageParam};
        uint8_t zeros[40] {};
        param_storage.write_block(0, zeros, sizeof(zeros));
        return MAV_RESULT_ACCEPTED;
    }
    if (is_equal(packet.param4, 97.0f)) {
        // create a really long loop
        send_text(MAV_SEVERITY_WARNING,"Creating long loop");
        // 250ms:
        for (uint8_t i=0; i<250; i++) {
            hal.scheduler->delay_microseconds(1000);
        }
        return MAV_RESULT_ACCEPTED;
    }
    if (is_equal(packet.param4, 98.0f)) {
        send_text(MAV_SEVERITY_WARNING,"Creating internal error");
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return MAV_RESULT_ACCEPTED;
    }
    if (is_equal(packet.param4, 100.0f)) {
        send_text(MAV_SEVERITY_WARNING,"Creating mutex deadlock");
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&GCS_MAVLINK::deadlock_sem, void));
        while (!_deadlock_sem.taken) {
            hal.scheduler->delay(1);
        }
        WITH_SEMAPHORE(_deadlock_sem.sem);
        send_text(MAV_SEVERITY_WARNING,"deadlock passed");
        return MAV_RESULT_ACCEPTED;
    }
    if (is_equal(packet.param4, 101.0f)) {
        // the capital-U and ~ here are actually important for
        // testing a MissionPlanner bug!
        AP_BoardConfig::config_error("YOU~RE WELCOME!");
    }
    if (is_equal(packet.param4, 102.0f)) {
        // attempt to write to address 0x5 (in the bottom 1kB on H7)
        // which we either memory-protect or check for
        // non-zeroness.  We don't want to use 0x0 as that *even
        // more magic*.  So choose an offset which looks like
        // we're dereferencing nullptr:
        uint8_t *foo = (uint8_t*)0x05;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if !defined(__clang__)  // avoid -Wunknown-warning-option
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
        *foo = 0xab;
#pragma GCC diagnostic pop

        return MAV_RESULT_ACCEPTED;
    }
    if (is_equal(packet.param4, 103.0f)) {
        // attempt to read from address 0x5 (in the bottom 1kB on
        // H7) which we either memory-protect or check for
        // non-zeroness.  We don't want to use 0x0 as that *even
        // more magic*.  So choose an offset which looks like
        // we're dereferencing nullptr:
        uint8_t *foo = (uint8_t*)0x05;

        // we use send_text here to ensure we don't get elided.
        // String is kept short for space reasons.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if !defined(__clang__)  // avoid -Wunknown-warning-option
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
        send_text(MAV_SEVERITY_INFO, "x: %u", (unsigned)*foo);
#pragma GCC diagnostic pop

        return MAV_RESULT_ACCEPTED;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(STM32F1)
    if (is_equal(packet.param4, 104.0f)) {
        // the following text is unlikely to make it out...
        send_text(MAV_SEVERITY_WARNING,"Creating interrupt storm");
        if (!create_interrupt_storm()) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }
#endif

    return MAV_RESULT_UNSUPPORTED;
}

/*
  take a semaphore and do not release it, triggering a deadlock
 */
void GCS_MAVLINK::deadlock_sem(void)
{
    if (!_deadlock_sem.taken) {
        _deadlock_sem.taken = true;
        _deadlock_sem.sem.take_blocking();
    }
}

#endif  // HAL_GCS_ENABLED && AP_MAVLINK_FAILURE_CREATION_ENABLED
