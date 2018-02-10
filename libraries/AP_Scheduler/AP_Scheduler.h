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


#define __FILE FILE
#include <functional>
#undef __FILE

#include <AP_Param/AP_Param.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>


extern const AP_HAL::HAL& hal;


struct AP_Task {   
    const char*                     name;
    std::function<void()>           function;
    float                           rate_hz;
    uint16_t                        max_time_micros;
    uint32_t                        last_run_ticks;
    AP_HAL::Util::perf_counter_t    perf_counter;   
};

// We define this as a constexpr, because of concerns regarding RAM usage in PX4
template <class Obj>
constexpr AP_Task make_task (
    const char* name, 
    Obj *obj,
    void(Obj::*func)(), 
    float rate_hz = 0, 
    uint16_t max_time_micros = 0,
    uint16_t last_run = 0
)    
{
    const std::function<void()> wrapper = [obj, func]{ (*obj.*func)(); };
    return { 
        name, 
        wrapper, 
        rate_hz, 
        max_time_micros, 
        last_run
    };
}

class AP_Scheduler {
    
public:
    AP_Scheduler();

    // initialise scheduler
    void init(AP_Task *tasks, uint8_t num_tasks);

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
    // used to enable scheduler debugging
    AP_Int8 _debug;

    // overall scheduling rate in Hz
    AP_Int16 _loop_rate_hz;

    // loop rate in Hz as set at startup
    AP_Int16 _active_loop_rate_hz;
    
    // calculated loop period in usec
    uint16_t _loop_period_us;

    // calculated loop period in seconds
    float _loop_period_s;
    
    // progmem list of tasks to run
    AP_Task *_tasks;

    // number of tasks in _tasks list
    uint8_t _num_tasks;

    // number of 'ticks' that have passed (number of times that
    // tick() has been called
    uint16_t _tick_counter;

    // number of microseconds allowed for the current task
    uint32_t _task_time_allowed;

    // the time in microseconds when the task started
    uint32_t _task_time_started;

    // number of spare microseconds accumulated
    uint32_t _spare_micros;

    // number of ticks that _spare_micros is counted over
    uint8_t _spare_ticks;

    uint32_t _last_run_us;
};

