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
#pragma once

#include <stdint.h>
#include <stdio.h>

#include <AP_Common/AP_Common.h> // for XSTRING macro
#include <AP_HAL/Util.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
  
  Author: Andrew Tridgell, January 2013
 */
template <class Vehicle>
struct AP_Task {
    typedef void (Vehicle::*task_fn_t)();

    task_fn_t   _function;
    const char* _name;
    float       _rate_hz;
    uint16_t    _max_time_micros;

    // We define this as a constexpr, because of concerns regarding RAM usage in PX4
    static constexpr AP_Task<Vehicle> create(task_fn_t function, float rate_hz, uint16_t max_time_micros) {
        // XSTRING is a macro for stringification
        return { function, XSTRING(function), rate_hz, max_time_micros };
    }
};

template <class Vehicle>
class AP_Scheduler {
    typedef void (Vehicle::*task_fn_t)();
  
public:
    // constructor, which takes the pointer of the parent object. The second parameter defines the standard loop rate
    AP_Scheduler(Vehicle *parent, const uint16_t loop_rate = 50)
    {
        _tasks = nullptr;
        _last_run = nullptr;

        AP_Param::setup_object_defaults(this, var_info);
        _loop_rate_hz.set_and_save(loop_rate);

        // only allow 50 to 400 Hz
        if (_loop_rate_hz < 50) {
            _loop_rate_hz.set(50);
        } else {
            if (_loop_rate_hz > 400) {
                _loop_rate_hz.set(400);
            }
        }
    }
    
    // destructor is necessary for this class
    ~AP_Scheduler()
    {
        if (_last_run) {
            delete [] _last_run;
        }
        if (_perf_counters) {
            delete [] _perf_counters;
        }
    }
    
    // Initialize scheduler
    void init(const AP_Task<Vehicle> *tasks, uint8_t num_tasks)
    {
        if (tasks == nullptr || num_tasks == 0) {
            return;
        }

        _tasks = tasks;
        _num_tasks = num_tasks;

        _last_run = new uint16_t[_num_tasks];
        memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
        _tick_counter = 0;
    }

    // call when one tick has passed
    void tick(void)
    {
        _tick_counter++;
    }

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(uint32_t time_available)
    {
        // break condition
        if (_tasks == nullptr) {
            return;
        }

        if (_debug > 1 && _perf_counters == nullptr) {
            _perf_counters = new AP_HAL::Util::perf_counter_t[_num_tasks];
            if (_perf_counters != nullptr) {
                for (uint8_t i = 0; i < _num_tasks; i++) {
                    _perf_counters[i] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _tasks[i]._name);
                }
            }
        }

        for (uint8_t i = 0; i < _num_tasks; i++) {
            const uint16_t dt = _tick_counter - _last_run[i];
            const uint16_t pre_interval_ticks = _loop_rate_hz / _tasks[i]._rate_hz;
            const uint16_t interval_ticks = pre_interval_ticks > 1 ? pre_interval_ticks : 1;

            if (dt < interval_ticks) {
                continue;
            }

            // we've slipped a whole run of this task!
            if (_debug > 4 && dt >= interval_ticks*2) {
                print_task_info("Scheduler slip task", i, dt, interval_ticks);
            }

            if (_tasks[i]._max_time_micros <= time_available) {
                // run it
                _task_time_started = AP_HAL::micros();
                current_task = i;
                if (_debug > 1 && _perf_counters[i]) {
                    hal.util->perf_begin(_perf_counters[i]);
                }

                // call that function
                if (_parent) {
                    task_fn_t func = _tasks[i]._function;
                    (_parent->*func)();
                }

                if (_debug > 1 && _perf_counters != nullptr && _perf_counters[i]) {
                    hal.util->perf_end(_perf_counters[i]);
                }
                current_task = -1;

                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;

                // work out how long the event actually took
                const uint32_t time_taken = AP_HAL::micros() - _task_time_started;

                // the event overrun!
                if (_debug > 4 && time_taken > _tasks[i]._max_time_micros) {
                    print_task_info("Scheduler overrun task", i, dt, time_taken);
                }

                if (time_taken >= time_available) {
                    update_spare_ticks(0);
                    return;
                }
                time_available -= time_taken;
            }
        }

        _spare_micros += time_available;

        // update number of spare microseconds
        update_spare_ticks(time_available);
    }

    // return the number of microseconds available for the current task or -1 if there is currently no task executed
    int32_t time_available_usec(void)
    {
        const int32_t dt = static_cast<int32_t>(AP_HAL::micros() - _task_time_started);

        if (current_task < 0) {
            return -1;
        }

        if (dt > _tasks[current_task]._max_time_micros) {
            return 0;
        }

        return _tasks[current_task]._max_time_micros - dt;
    }

    // return debug parameter
    uint8_t debug(void) { return _debug; }

    // return load average, as a number between 0 and 1. 1 means
    // 100% load. Calculated from how much spare time we have at the
    // end of a run()
    float load_average(const uint32_t tick_time_usec) const
    {
        if (_spare_ticks == 0) {
            return 0.0f;
        }
        const uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
        return static_cast<float>(used_time) / static_cast<float>(tick_time_usec);
    }

    // get the configured main loop rate
    uint16_t get_loop_rate_hz(void) const {
        return _loop_rate_hz;
    }
    
    static const struct AP_Param::GroupInfo var_info[];

    // current running task, or -1 if none. Used to debug stuck tasks
    static int8_t current_task;

protected:
    // updates the tick members
    void update_spare_ticks(const uint32_t time_available)
    {
        _spare_ticks++;
        if (_spare_ticks == 32) {
            _spare_ticks /= 2;
            _spare_micros /= 2;
        }
    }

    // standard output of task information
    void print_task_info(const char *cstr, const unsigned iter, const unsigned dt, const unsigned var) const
    {
        ::printf("%s[%u-%s] (%u/%u/%u)\n",
                  cstr,
                  (unsigned)iter,
                  _tasks[iter]._name,
                  (unsigned)dt,
                  (unsigned)var,
                  (unsigned)_tasks[iter]._max_time_micros);
    }
    
private:  
    // Object pointer to parent class
    Vehicle* _parent = nullptr;
  
    // used to enable scheduler debugging
    AP_Int8 _debug;

    // overall scheduling rate in Hz
    AP_Int16 _loop_rate_hz;  // The value of this variable can be changed with the non-initialization. (Ex. Tuning by GDB)
    
    // progmem list of tasks to run
    const struct AP_Task<Vehicle> *_tasks;

    // number of tasks in _tasks list
    uint8_t _num_tasks;

    // number of 'ticks' that have passed (number of times that
    // tick() has been called
    uint16_t _tick_counter;

    // tick counter at the time we last ran each task
    uint16_t *_last_run;

    // the time in microseconds when the task started
    uint32_t _task_time_started;

    // number of spare microseconds accumulated
    uint32_t _spare_micros;

    // number of ticks that _spare_micros is counted over
    uint8_t _spare_ticks;

    // performance counters
    AP_HAL::Util::perf_counter_t *_perf_counters;
};

template <class Vehicle> int8_t AP_Scheduler<Vehicle>::current_task = -1;

template <class Vehicle>
const AP_Param::GroupInfo AP_Scheduler<Vehicle>::var_info[] = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),

    // @Param: LOOP_RATE
    // @DisplayName: Scheduling main loop rate
    // @Description: This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart
    // @Values: 50:50Hz,100:100Hz,200:200Hz,250:250Hz,300:300Hz,400:400Hz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("LOOP_RATE",  1, AP_Scheduler, _loop_rate_hz, 50),

    AP_GROUPEND
};
