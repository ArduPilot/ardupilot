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
#pragma once

#include "AP_Scheduler_config.h"

#if AP_SCHEDULER_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>
#include "PerfInfo.h"       // loop perf monitoring
#include <AP_InternalError/AP_InternalError.h>

#if AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED
#define AP_SCHEDULER_NAME_INITIALIZER(_clazz,_name) .name = #_clazz "::" #_name,
#define AP_FAST_NAME_INITIALIZER(_clazz,_name) .name = #_clazz "::" #_name "*",
#else
#define AP_SCHEDULER_NAME_INITIALIZER(_clazz,_name) .name = #_name,
#define AP_FAST_NAME_INITIALIZER(_clazz,_name) .name = #_name "*",
#endif
#define LOOP_RATE 0

/*
  useful macro for creating scheduler task table
 */
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,        \
    .priority = _priority \
}

/*
  useful macro for creating the fastloop task table
 */
#define FAST_TASK_CLASS(classname, classptr, func) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_FAST_NAME_INITIALIZER(classname, func)\
    .rate_hz = 0,\
    .max_time_micros = 0,\
    .priority = AP_Scheduler::FAST_TASK_PRI0 \
}

/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */

class AP_Scheduler
{
public:
    AP_Scheduler();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scheduler);

    static AP_Scheduler *get_singleton();
    static AP_Scheduler *_singleton;

    FUNCTOR_TYPEDEF(task_fn_t, void);

    struct Task {
        task_fn_t function;
        const char *name;
        float rate_hz;
        uint16_t max_time_micros;
        uint8_t priority; // task priority
    };

    enum class Options : uint8_t {
        RECORD_TASK_INFO = 1 << 0
    };

    enum FastTaskPriorities {
        FAST_TASK_PRI0 = 0,
        FAST_TASK_PRI1 = 1,
        FAST_TASK_PRI2 = 2,
        MAX_FAST_TASK_PRIORITIES = 3
    };

    // initialise scheduler
    void init(const Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit);

    // called by vehicle's main loop - which should be the only thing
    // that function does
    void loop();

    // call to update any logging the scheduler might do; call at 1Hz
    void update_logging();

    // write out PERF message to logger
    void Log_Write_Performance();

    // call when one tick has passed
    void tick(void);

    // return current tick counter
    uint16_t ticks() const { return _tick_counter; }
    uint32_t ticks32() const { return _tick_counter32; }

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(uint32_t time_available);

    // return the number of microseconds available for the current task
    uint16_t time_available_usec(void) const;

    // return debug parameter
    uint8_t debug_flags(void) { return _debug; }

    // return load average, as a number between 0 and 1. 1 means
    // 100% load. Calculated from how much spare time we have at the
    // end of a run()
    float load_average();

    // get the active main loop rate
    uint16_t get_loop_rate_hz(void) {
        if (_active_loop_rate_hz == 0) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _active_loop_rate_hz = _loop_rate_hz;
        }
        return _active_loop_rate_hz;
    }
    // get the time-allowed-per-loop in microseconds
    uint32_t get_loop_period_us() {
        if (_loop_period_us == 0) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _loop_period_us = 1000000UL / _loop_rate_hz;
        }
        return _loop_period_us;
    }
    // get the time-allowed-per-loop in seconds
    float get_loop_period_s() {
        if (is_zero(_loop_period_s)) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _loop_period_s = 1.0f / _loop_rate_hz;
        }
        return _loop_period_s;
    }

    // get the filtered main loop time in seconds
    float get_filtered_loop_time(void) const {
        return perf_info.get_filtered_time();
    }

    // get the filtered active main loop rate
    float get_filtered_loop_rate_hz() {
        return perf_info.get_filtered_loop_rate_hz();
    }

    // get the time in seconds that the last loop took
    float get_last_loop_time_s(void) const {
        return _last_loop_time_s;
    }

    // get the time in microseconds that the current loop started
    uint64_t get_loop_start_time_us(void) const {
        return _loop_sample_time_us;
    }

    // get the amount of extra time being added on each loop
    uint32_t get_extra_loop_us(void) const {
        return extra_loop_us;
    }

    HAL_Semaphore &get_semaphore(void) { return _rsem; }

    void task_info(ExpandingString &str);

    static const struct AP_Param::GroupInfo var_info[];

    // loop performance monitoring:
    AP::PerfInfo perf_info;

private:
    // used to enable scheduler debugging
    AP_Int8 _debug;

    // overall scheduling rate in Hz
    AP_Int16 _loop_rate_hz;

    // loop rate in Hz as set at startup
    uint16_t _active_loop_rate_hz;

    // scheduler options
    AP_Int8 _options;
    
    // calculated loop period in usec
    uint16_t _loop_period_us;

    // calculated loop period in seconds
    float _loop_period_s;
    
    // list of tasks to run
    const struct Task *_vehicle_tasks;
    uint8_t _num_vehicle_tasks;

    // list of common tasks to run
    const struct Task *_common_tasks;
    uint8_t _num_common_tasks;

    // total number of tasks in _tasks and _common_tasks list
    uint8_t _num_tasks;

    // number of 'ticks' that have passed (number of times that
    // tick() has been called
    uint16_t _tick_counter;
    uint32_t _tick_counter32;

    // tick counter at the time we last ran each task
    uint16_t *_last_run;

    // number of microseconds allowed for the current task
    uint32_t _task_time_allowed;

    // the time in microseconds when the task started
    uint32_t _task_time_started;

    // number of spare microseconds accumulated
    uint32_t _spare_micros;

    // number of ticks that _spare_micros is counted over
    uint8_t _spare_ticks;

    // start of previous loop
    uint32_t _loop_timer_start_us;

    // time of last loop in seconds
    float _last_loop_time_s;

    // start of current loop
    uint64_t _loop_sample_time_us;

    // bitmask bit which indicates if we should log PERF message
    uint32_t _log_performance_bit;

    // maximum task slowdown compared to desired task rate before we
    // start giving extra time per loop
    const uint8_t max_task_slowdown = 4;

    // counters to handle dynamically adjusting extra loop time to
    // cope with low CPU conditions
    uint32_t task_not_achieved;
    uint32_t task_all_achieved;
    
    // extra time available for each loop - used to dynamically adjust
    // the loop rate in case we are well over budget
    uint32_t extra_loop_us;


    // semaphore that is held while not waiting for ins samples
    HAL_Semaphore _rsem;
};

namespace AP {
    AP_Scheduler &scheduler();
};

#endif  // AP_SCHEDULER_ENABLED
