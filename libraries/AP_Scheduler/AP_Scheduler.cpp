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
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define SCHEDULER_DEFAULT_LOOP_RATE 400
#else
#define SCHEDULER_DEFAULT_LOOP_RATE  50
#endif

#define debug_helper(level, fmt, args...)   do { if ((level) <= _debug.get()) { hal.console->printf(fmt, ##args); }} while (0)


int8_t AP_Scheduler::current_task = -1;

const AP_Param::GroupInfo AP_Scheduler::var_info[] = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),

    // @Param: LOOP_RATE
    // @DisplayName: Scheduling main loop rate
    // @Description: This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart. Values over 400 are considered highly experimental.
    // @Values: 50:50Hz,100:100Hz,200:200Hz,250:250Hz,300:300Hz,400:400Hz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("LOOP_RATE",  1, AP_Scheduler, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE),

    AP_GROUPEND
};

// constructor
AP_Scheduler::AP_Scheduler()
{
    AP_Param::setup_object_defaults(this, var_info);

    // only allow 50 to 2000 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz.set(50);
    } 
    if (_loop_rate_hz > 2000) {
        _loop_rate_hz.set(2000);
    }
}

// initialise the scheduler
void AP_Scheduler::init(AP_Task *tasks, uint8_t num_tasks) 
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _loop_period_us = 1000000UL / _loop_rate_hz;
    _last_run_us = AP_HAL::micros();
    
    if(_debug <= 1) return;
    for (uint8_t i=0; i < _num_tasks; i++) {
        _tasks[i].perf_counter = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _tasks[i].name);
    }
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint32_t time_available)
{    
/*
    // the scheduler can run faster than the given frequency would allow 
    // e.g. if the embeding loop is not blocking => we have to estimate the time
    if(AP_HAL::micros() - _last_run_us < _loop_period_us) {
        return;
    }
    _last_run_us = AP_HAL::micros();
*/
    _tick_counter++;

    for (uint8_t i = 0; i < _num_tasks; i++) {
        const uint16_t dticks = _tick_counter - _tasks[i].last_run_ticks; // (x * _loop_period_us) would be the time which passed
        const uint16_t tmp = _loop_rate_hz / _tasks[i].rate_hz;   // how often the task can be called
        const uint16_t interval_ticks = tmp < 1 ? 1 : tmp;
        const uint16_t bias_slipped_ticks = interval_ticks;
        
        //printf("%d: passed ticks %d, req ticks %d\n", i, dticks, tmp);
        
        if(dticks < interval_ticks) continue;
        // this task is due to run. Do we have enough time to run it?
        _task_time_allowed = _tasks[i].max_time_micros;

        // we've slipped a whole run of this task!
        if (dticks >= bias_slipped_ticks) {
            debug_helper(2, "Scheduler slip task[%u-%s] (%u/%u/%u)\n",
                    (unsigned)i,
                    _tasks[i].name,
                    (unsigned)dticks,
                    (unsigned)interval_ticks,
                    (unsigned)_task_time_allowed);
        }
        
        // run it
        if (_task_time_allowed <= time_available) {
            _task_time_started = AP_HAL::micros();
            current_task = i;
            auto * const perf_cntr = _tasks[i].perf_counter;
            const bool bPerfCntrs = _debug > 1 && perf_cntr;
            if (bPerfCntrs) {
                hal.util->perf_begin(perf_cntr);
            }
            
            _tasks[i].function();

            if (bPerfCntrs) {
                hal.util->perf_end(perf_cntr);
            }
            current_task = -1;

            // record the tick counter when we ran. This drives
            // when we next run the event
            _tasks[i].last_run_ticks = _tick_counter;

            // work out how long the event actually took
            const uint32_t time_taken = AP_HAL::micros() - _task_time_started;

            // the event overran!
            if (time_taken > _task_time_allowed) {
                debug_helper(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
                        (unsigned)i,
                        _tasks[i].name,
                        (unsigned)time_taken,
                        (unsigned)_task_time_allowed);
            }
            
            if (time_taken >= time_available) {
                update_spare_ticks();
                return;
            }
            time_available -= time_taken;
        }
    }

    // update number of spare microseconds
    _spare_micros += time_available;
    update_spare_ticks();
}

void AP_Scheduler::update_spare_ticks() {
    // update number of spare microseconds
    _spare_ticks++;
    if(_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec()
{
    const uint32_t dt = AP_HAL::micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average()
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    const uint32_t loop_us = get_loop_period_us();
    const uint32_t used_time = loop_us - (_spare_micros/_spare_ticks);
    return used_time / (float)loop_us;
}
