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
  Flymaple port by Mike McCauley
 */

#ifndef __AP_HAL_FLYMAPLE_SCHEDULER_H__
#define __AP_HAL_FLYMAPLE_SCHEDULER_H__

#include "AP_HAL_FLYMAPLE.h"

#define FLYMAPLE_SCHEDULER_MAX_TIMER_PROCS 4

class AP_HAL_FLYMAPLE_NS::FLYMAPLEScheduler : public AP_HAL::Scheduler {
public:
    FLYMAPLEScheduler();
    void     init();
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);

    void     register_io_process(AP_HAL::MemberProc);

    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     system_initialized();

    void     reboot(bool hold_in_bootloader);

private:
    static volatile bool _in_timer_proc;

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    bool _initialized;

    /* _timer_procs_timer_event() and _run_timer_procs are static so they can be
     * called from an interrupt. */
    static void _timer_procs_timer_event();
    static void _run_timer_procs(bool called_from_isr);

    static void _failsafe_timer_event();
    static AP_HAL::Proc _failsafe;

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[FLYMAPLE_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
};

#endif // __AP_HAL_FLYMAPLE_SCHEDULER_H__
