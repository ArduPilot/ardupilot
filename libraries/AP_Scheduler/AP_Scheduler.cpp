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
#include "AP_Scheduler.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include <AP_HAL/SIMState.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif
#include <stdio.h>

#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduSub) || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define SCHEDULER_DEFAULT_LOOP_RATE 400
#else
#define SCHEDULER_DEFAULT_LOOP_RATE  50
#endif

#define debug(level, fmt, args...)   do { if ((level) <= _debug.get()) { hal.console->printf(fmt, ##args); }} while (0)

extern const AP_HAL::HAL& hal;

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

    // @Param: OPTIONS
    // @DisplayName: Scheduling options
    // @Description: This controls optional aspects of the scheduler.
    // @Bitmask: 0:Enable per-task perf info
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  2, AP_Scheduler, _options, 0),

    AP_GROUPEND
};

// constructor
AP_Scheduler::AP_Scheduler()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many schedulers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Get the AP_Scheduler singleton
 */
AP_Scheduler *AP_Scheduler::_singleton;
AP_Scheduler *AP_Scheduler::get_singleton()
{
    return _singleton;
}

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit)
{
    // grab the semaphore before we start anything
    _rsem.take_blocking();

    // only allow 50 to 2000 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz.set(50);
    } else if (_loop_rate_hz > 2000) {
        _loop_rate_hz.set(2000);
    }
    _last_loop_time_s = 1.0 / _loop_rate_hz;

    _vehicle_tasks = tasks;
    _num_vehicle_tasks = num_tasks;

    AP_Vehicle* vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->get_common_scheduler_tasks(_common_tasks, _num_common_tasks);
    }

    _num_tasks = _num_vehicle_tasks + _num_common_tasks;

   _last_run = new uint16_t[_num_tasks];
    _tick_counter = 0;

    // setup initial performance counters
    perf_info.set_loop_rate(get_loop_rate_hz());
    perf_info.reset();

    if (_options & uint8_t(Options::RECORD_TASK_INFO)) {
        perf_info.allocate_task_info(_num_tasks);
    }

    _log_performance_bit = log_performance_bit;

    // sanity check the task lists to ensure the priorities are
    // never decrease
    uint8_t old = 0;
    for (uint8_t i=0; i<_num_common_tasks; i++) {
        if (_common_tasks[i].priority < old){
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        old = _common_tasks[i].priority;
    }
    old = 0;
    for (uint8_t i=0; i<_num_vehicle_tasks; i++) {
        if (_vehicle_tasks[i].priority < old) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        old = _vehicle_tasks[i].priority;
    }
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
/*
  fill stack with NaN so we can catch use of uninitialised stack
  variables in SITL
 */
static void fill_nanf_stack(void)
{
    float v[1024];
    fill_nanf(v, ARRAY_SIZE(v));
}
#endif

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_usec = AP_HAL::micros();
    uint32_t now = run_started_usec;

    uint8_t vehicle_tasks_offset = 0;
    uint8_t common_tasks_offset = 0;

    for (uint8_t i=0; i<_num_tasks; i++) {
        // determine which of the common task / vehicle task to run
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < _num_vehicle_tasks &&
            common_tasks_offset < _num_common_tasks) {
            // still have entries on both lists; compare the
            // priorities.  In case of a tie the vehicle-specific
            // entry wins.
            const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
            const Task &common_task = _common_tasks[common_tasks_offset];
            if (vehicle_task.priority <= common_task.priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
            // out of common tasks to run
            run_vehicle_task = true;
        } else if (common_tasks_offset < _num_common_tasks) {
            // out of vehicle tasks to run
            run_vehicle_task = false;
        } else {
            // this is an error; the outside loop should have terminated
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }

        const AP_Scheduler::Task &task = run_vehicle_task ? _vehicle_tasks[vehicle_tasks_offset] : _common_tasks[common_tasks_offset];
        if (run_vehicle_task) {
            vehicle_tasks_offset++;
        } else {
            common_tasks_offset++;
        }

        if (task.priority > MAX_FAST_TASK_PRIORITIES) {
            const uint16_t dt = _tick_counter - _last_run[i];
            // we allow 0 to mean loop rate
            uint32_t interval_ticks = (is_zero(task.rate_hz) ? 1 : _loop_rate_hz / task.rate_hz);
            if (interval_ticks < 1) {
                interval_ticks = 1;
            }
            if (dt < interval_ticks) {
                // this task is not yet scheduled to run again
                continue;
            }
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = task.max_time_micros;

            if (dt >= interval_ticks*2) {
                perf_info.task_slipped(i);
            }

            if (dt >= interval_ticks*max_task_slowdown) {
                // we are going beyond the maximum slowdown factor for a
                // task. This will trigger increasing the time budget
                task_not_achieved++;
            }

            if (_task_time_allowed > time_available) {
                // not enough time to run this task.  Continue loop -
                // maybe another task will fit into time remaining
                continue;
            }
        } else {
            _task_time_allowed = get_loop_period_us();
        }

        // run it
        _task_time_started = now;
        hal.util->persistent_data.scheduler_task = i;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        fill_nanf_stack();
#endif
        task.function();
        hal.util->persistent_data.scheduler_task = -1;

        // record the tick counter when we ran. This drives
        // when we next run the event
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        now = AP_HAL::micros();
        uint32_t time_taken = now - _task_time_started;
        bool overrun = false;
        if (time_taken > _task_time_allowed) {
            overrun = true;
            // the event overran!
            debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
                  (unsigned)i,
                  task.name,
                  (unsigned)time_taken,
                  (unsigned)_task_time_allowed);
        }

        perf_info.update_task_info(i, time_taken, overrun);

        if (time_taken >= time_available) {
            time_available = 0;
            break;
        }
        time_available -= time_taken;
    }

    // update number of spare microseconds
    _spare_micros += time_available;

    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void) const
{
    uint32_t dt = AP_HAL::micros() - _task_time_started;
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

void AP_Scheduler::loop()
{
    // wait for an INS sample
    hal.util->persistent_data.scheduler_task = -3;
    _rsem.give();
    AP::ins().wait_for_sample();
    _rsem.take_blocking();
    hal.util->persistent_data.scheduler_task = -1;

    const uint32_t sample_time_us = AP_HAL::micros();
    
    if (_loop_timer_start_us == 0) {
        _loop_timer_start_us = sample_time_us;
        _last_loop_time_s = get_loop_period_s();
    } else {
        _last_loop_time_s = (sample_time_us - _loop_timer_start_us) * 1.0e-6;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    {
        /*
          for testing low CPU conditions we can add an optional delay in SITL
        */
        auto *sitl = AP::sitl();
        uint32_t loop_delay_us = sitl->loop_delay.get();
        hal.scheduler->delay_microseconds(loop_delay_us);
    }
#endif

    // tell the scheduler one tick has passed
    tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    const uint32_t loop_us = get_loop_period_us();
    uint32_t now = AP_HAL::micros();
    uint32_t time_available = 0;
    const uint32_t loop_tick_us = now - sample_time_us;
    if (loop_tick_us < loop_us) {
        // get remaining time available for this loop
        time_available = loop_us - loop_tick_us;
    }

    // add in extra loop time determined by not achieving scheduler tasks
    time_available += extra_loop_us;

    // run the tasks
    run(time_available);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // move result of AP_HAL::micros() forward:
    hal.scheduler->delay_microseconds(1);
#endif

    if (task_not_achieved > 0) {
        // add some extra time to the budget
        extra_loop_us = MIN(extra_loop_us+100U, 5000U);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            // we have gone through 50 loops without a task taking too
            // long. CPU pressure has eased, so drop the extra time we're
            // giving each loop
            task_all_achieved = 0;
            // we are achieving all tasks, slowly lower the extra loop time
            extra_loop_us = MAX(0U, extra_loop_us-50U);
        }
    }

    // check loop time
    perf_info.check_loop_time(sample_time_us - _loop_timer_start_us);
        
    _loop_timer_start_us = sample_time_us;

#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
    hal.simstate->update();
#endif
}

void AP_Scheduler::update_logging()
{
    if (debug_flags()) {
        perf_info.update_logging();
    }
    if (_log_performance_bit != (uint32_t)-1 &&
        AP::logger().should_log(_log_performance_bit)) {
        Log_Write_Performance();
    }
    perf_info.set_loop_rate(get_loop_rate_hz());
    perf_info.reset();
    // dynamically update the per-task perf counter
    if (!(_options & uint8_t(Options::RECORD_TASK_INFO)) && perf_info.has_task_info()) {
        perf_info.free_task_info();
    } else if ((_options & uint8_t(Options::RECORD_TASK_INFO)) && !perf_info.has_task_info()) {
        perf_info.allocate_task_info(_num_tasks);
    }
}

// Write a performance monitoring packet
void AP_Scheduler::Log_Write_Performance()
{
    const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us          : AP_HAL::micros64(),
        num_long_running : perf_info.get_num_long_running(),
        num_loops        : perf_info.get_num_loops(),
        max_time         : perf_info.get_max_time(),
        mem_avail        : hal.util->available_memory(),
        load             : (uint16_t)(load_average() * 1000),
        internal_error_last_line : AP::internalerror().last_error_line(),
        internal_errors  : AP::internalerror().errors(),
        internal_error_count : AP::internalerror().count(),
        spi_count        : pd.spi_count,
        i2c_count        : pd.i2c_count,
        i2c_isr_count    : pd.i2c_isr_count,
        extra_loop_us    : extra_loop_us,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
}

// display task statistics as text buffer for @SYS/tasks.txt
void AP_Scheduler::task_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("TasksV2\n");

    // dynamically enable statistics collection
    if (!(_options & uint8_t(Options::RECORD_TASK_INFO))) {
        _options.set(_options | uint8_t(Options::RECORD_TASK_INFO));
        return;
    }

    if (perf_info.get_task_info(0) == nullptr) {
        return;
    }

    // baseline the total time taken by all tasks
    float total_time = 1.0f;
    for (uint8_t i = 0; i < _num_tasks + 1; i++) {
        const AP::PerfInfo::TaskInfo* ti = perf_info.get_task_info(i);
        if (ti != nullptr && ti->tick_count > 0) {
            total_time += ti->elapsed_time_us;
        }
    }

    uint8_t vehicle_tasks_offset = 0;
    uint8_t common_tasks_offset = 0;

    for (uint8_t i = 0; i < _num_tasks; i++) {
        const AP::PerfInfo::TaskInfo* ti = perf_info.get_task_info(i);
        const char *task_name;

        // determine which of the common task / vehicle task to run
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < _num_vehicle_tasks &&
            common_tasks_offset < _num_common_tasks) {
            // still have entries on both lists; compare the
            // priorities.  In case of a tie the vehicle-specific
            // entry wins.
            const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
            const Task &common_task = _common_tasks[common_tasks_offset];
            if (vehicle_task.priority <= common_task.priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
            // out of common tasks to run
            run_vehicle_task = true;
        } else if (common_tasks_offset < _num_common_tasks) {
            // out of vehicle tasks to run
            run_vehicle_task = false;
        } else {
            // this is an error; the outside loop should have terminated
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }

        if (run_vehicle_task) {
            task_name = _vehicle_tasks[vehicle_tasks_offset++].name;
        } else {
            task_name = _common_tasks[common_tasks_offset++].name;
        }

        ti->print(task_name, total_time, str);
    }
}

namespace AP {

AP_Scheduler &scheduler()
{
    return *AP_Scheduler::get_singleton();
}

};
