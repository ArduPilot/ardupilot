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

#include <AP_Param/AP_Param.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>


template <class Vehicle>
struct AP_Task {
    typedef void (Vehicle::*task_fn_t)();
        
    task_fn_t   function;
    const char* name;
    float       rate_hz;
    uint16_t    max_time_micros;
    uint16_t    last_run;
};

// We define this as a constexpr, because of concerns regarding RAM usage in PX4
template <class Vehicle>
static constexpr AP_Task<Vehicle> make_task(void (Vehicle::*task_fn_t)(), float rate_hz = 0, uint16_t max_time_micros = 0, const char* name = "") {
    return { task_fn_t, name, rate_hz, max_time_micros, 0 };
}

template <class Vehicle>
class AP_Scheduler {
    typedef void (Vehicle::*task_fn_t)();
    
public:
    AP_Scheduler(Vehicle &parent);

    // initialise scheduler
    void init(AP_Task<Vehicle> *tasks, uint8_t num_tasks);

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(uint32_t time_available);

    // return the number of microseconds available for the current task
    uint16_t time_available_usec(void);

    // return debug parameter
    uint8_t debug(void) { return _debug; }

    // return load average, as a number between 0 and 1. 1 means
    // 100% load. Calculated from how much spare time we have at the
    // end of a run()
    float load_average();

    // get the active main loop rate
    uint16_t get_loop_rate_hz(void) {
        if (_active_loop_rate_hz == 0) {
            _active_loop_rate_hz = _loop_rate_hz;
        }
        return _active_loop_rate_hz;
    }
    // get the time-allowed-per-loop in microseconds
    uint32_t get_loop_period_us() {
        if (_loop_period_us == 0) {
            _loop_period_us = 1000000UL / _loop_rate_hz;
        }
        return _loop_period_us;
    }
    // get the time-allowed-per-loop in seconds
    float get_loop_period_s() {
        if (is_zero(_loop_period_s)) {
            _loop_period_s = 1.0 / _loop_rate_hz;
        }
        return _loop_period_s;
    }

    static const struct AP_Param::GroupInfo var_info[];

    // current running task, or -1 if none. Used to debug stuck tasks
    static int8_t current_task;

protected:
    void update_spare_ticks();
    
private:
    // Object pointer to parent class (which member funtions shall be executed)
    Vehicle &_parent;
    
    // used to enable scheduler debugging
    AP_Int8 _debug;

    // overall scheduling rate in Hz
    AP_Int16 _loop_rate_hz;

    // loop rate in Hz as set at startup
    AP_Int16 _active_loop_rate_hz;
    
    // calculated loop period in usec
    uint16_t _loop_period_us = 0;

    // calculated loop period in seconds
    float _loop_period_s = 0;
    
    // progmem list of tasks to run
    AP_Task<Vehicle> *_tasks = nullptr;

    // number of tasks in _tasks list
    uint8_t _num_tasks = 0;

    // number of 'ticks' that have passed (number of times that
    // tick() has been called
    uint16_t _tick_counter = 0;

    // number of microseconds allowed for the current task
    uint32_t _task_time_allowed = 0;

    // the time in microseconds when the task started
    uint32_t _task_time_started = 0;

    // number of spare microseconds accumulated
    uint32_t _spare_micros = 0;

    // number of ticks that _spare_micros is counted over
    uint8_t _spare_ticks = 0;

    uint32_t _last_run_us = 0;
    
    // performance counters
    AP_HAL::Util::perf_counter_t *_perf_counters;
};

template <class Vehicle> int8_t AP_Scheduler<Vehicle>::current_task = -1;

#include "AP_Scheduler.tpp"
