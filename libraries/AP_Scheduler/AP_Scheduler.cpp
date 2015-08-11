/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Scheduler.h"
#include <AP_Param/AP_Param.h>

#if defined(__AVR__)
#define TASK_NAME(id) PSTR("unknown")
#else
#define TASK_NAME(id) _tasks[id].name
#endif

extern const AP_HAL::HAL& hal;

int8_t AP_Scheduler::current_task = -1;

const AP_Param::GroupInfo AP_Scheduler::var_info[] PROGMEM = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table. The option ShowAll makes all executions be logged as well as slips.
    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns,4:ShowAll
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),
    AP_GROUPEND
};

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks) 
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _task_execution_log_buffer = new AP_Scheduler::TaskExecutionLog[_num_tasks];
    _tick_counter = 0;
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint16_t time_available)
{
    uint32_t run_started_usec = hal.scheduler->micros();
    uint32_t now = run_started_usec;

    uint8_t log_count = 0;
    uint8_t current_log_index = 0;

    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        uint16_t interval_ticks = pgm_read_word(&_tasks[i].interval_ticks);
        if (dt >= interval_ticks) {
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = pgm_read_word(&_tasks[i].max_time_micros);

            current_log_index = log_count;
            _task_execution_log_buffer[current_log_index].flag = 0;

            if (dt >= interval_ticks*2) {
                // we've slipped a whole run of this task!
                if (_debug > 1) {
                    _task_execution_log_buffer[current_log_index].task_id = i;
                    _task_execution_log_buffer[current_log_index].flag |= SCHEDULER_TASK_SLIPPED;
                    _task_execution_log_buffer[current_log_index].ticks_since_last_run = dt;
                    log_count++;
                }
            }
            
            if (_task_time_allowed <= time_available) {
                // run it
                _task_time_started = now;
                task_fn_t func;
                pgm_read_block(&_tasks[i].function, &func, sizeof(func));
                current_task = i;
                func();
                current_task = -1;
                
                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;
                
                // work out how long the event actually took
                now = hal.scheduler->micros();
                uint32_t time_taken = now - _task_time_started;

                // if the event overran or _debug set to log all executions
                if ((time_taken > _task_time_allowed && _debug > 2) || _debug > 3) {
                    _task_execution_log_buffer[current_log_index].task_id = i;
                    _task_execution_log_buffer[current_log_index].time_taken = time_taken;

                    _task_execution_log_buffer[current_log_index].flag |= SCHEDULER_TASK_EXECUTED;
                    if (time_taken > _task_time_allowed) {
                        _task_execution_log_buffer[current_log_index].flag |= SCHEDULER_TASK_OVERRUN;
                    }

                    if (current_log_index == log_count) log_count++;
                }

                if (time_taken >= time_available) {
                    goto update_spare_ticks;
                }
                time_available -= time_taken;
            }
        }
    }

    // update number of spare microseconds
    _spare_micros += time_available;

update_spare_ticks:
    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }

    // output debug log, if any
    for (unsigned int i = 0; i < log_count; i++) {
        TaskExecutionLog * log = _task_execution_log_buffer + i;
        uint32_t time_allowed = pgm_read_word(&_tasks[log->task_id].max_time_micros);

        if (log->flag & SCHEDULER_TASK_SLIPPED) {
            uint16_t interval_ticks = pgm_read_word(&_tasks[log->task_id].interval_ticks);
            hal.console->printf_P(PSTR("Scheduler slip task[%u-%s] (%u/%u/%u)\n"),
                                  (unsigned) log->task_id,
                                  TASK_NAME(log->task_id),
                                  (unsigned) log->ticks_since_last_run,
                                  (unsigned) interval_ticks,
                                  (unsigned) time_allowed);
        }

        if (log->flag & SCHEDULER_TASK_EXECUTED) {
            if ((log->flag & SCHEDULER_TASK_OVERRUN) && _debug == 3) {
                hal.console->printf_P(PSTR("Scheduler overrun task[%u-%s] (%u/%u)\n"),
                                      (unsigned) log->task_id,
                                      TASK_NAME(log->task_id),
                                      (unsigned) log->time_taken,
                                      (unsigned) time_allowed);
            } else {
                hal.console->printf_P(PSTR("Scheduler task[%u-%s] (%u/%u)\n"),
                                      (unsigned) log->task_id,
                                      TASK_NAME(log->task_id),
                                      (unsigned) log->time_taken,
                                      (unsigned) time_allowed);
            }
        }
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void)
{
    uint32_t dt = hal.scheduler->micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average(uint32_t tick_time_usec) const
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
    return used_time / (float)tick_time_usec;
}
