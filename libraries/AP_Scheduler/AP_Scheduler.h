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
#pragma once

#include <stdint.h>
#include <stdio.h>

#include <AP_Common/AP_Common.h> // for XSTRING macro
#include <AP_HAL/Util.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

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
    AP_Scheduler(Vehicle *parent, const uint16_t loop_rate = 50);
    
    // destructor is necessary for this class
    ~AP_Scheduler();
    
    // initialise scheduler
    void init(const AP_Task<Vehicle> *tasks, uint8_t num_tasks);

    // call when one tick has passed
    void tick(void);

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(const uint32_t time_available);

    // return the number of microseconds available for the current task or -1 if there is currently no task executed
    int32_t time_available_usec(void);

    // return debug parameter
    uint8_t debug(void) { return _debug; }

    // return load average, as a number between 0 and 1. 1 means
    // 100% load. Calculated from how much spare time we have at the
    // end of a run()
    float load_average(const uint32_t tick_time_usec) const;

    // get the configured main loop rate
    uint16_t get_loop_rate_hz(void) const {
        return _loop_rate_hz;
    }
    
    static const struct AP_Param::GroupInfo var_info[];

    // current running task, or -1 if none. Used to debug stuck tasks
    static int8_t current_task;

protected:
    // updates the tick members
    void update_spare_ticks(const uint32_t time_available);

    // standard output of task information
    void print_task_info(const char *, const unsigned, const unsigned, const unsigned) const;
    
private:  
    // Object pointer to parent class
    Vehicle* _parent = nullptr;
  
    // used to enable scheduler debugging
    AP_Int8 _debug;

    // overall scheduling rate in Hz
    AP_Int16 _loop_rate_hz;
    
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

#include "AP_Scheduler.tpp"
