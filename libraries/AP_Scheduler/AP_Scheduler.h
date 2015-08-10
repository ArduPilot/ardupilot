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
 *  Authors: 
 *  - Andrew Tridgell, January 2013
 *  - Daniel Frenzel, August 2015
 */

#ifndef AP_SCHEDULER_H
#define AP_SCHEDULER_H

#include <AP_Param.h>

#define SCHEDULER_TASK_SLIPPED 0x1
#define SCHEDULER_TASK_EXECUTED 0x2
#define SCHEDULER_TASK_OVERRUN 0x4

#ifndef __AVR__
#define AP_SCHEDULER_NAME_INITIALIZER(_name) .name = #_name,
#else
#define AP_SCHEDULER_NAME_INITIALIZER(_name)
#endif

/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */

#include <AP_HAL.h>
#include <AP_Vehicle.h>


class AP_Task;
class AP_Scheduler;

//////////////////////////////////////////////////////////////////////////
// TASK
//////////////////////////////////////////////////////////////////////////
FUNCTOR_TYPEDEF(task_fn_t, void);
class AP_Task {
public /*functions*/:
    AP_Task       (AP_Scheduler *parent_sched = NULL);
    
    AP_Scheduler* parent() const;
    void          parent(AP_Scheduler *);

    // executes the task if time is available and returns whether the task was executed 
    bool          run(uint16_t, uint16_t);
    // Return whether task is currently running
    bool          is_running() const;
    // returns how many times the task failed to be executed
    uint8_t       missed_runs() const;
    // the (optional) index of the task in the scheduler (for identification)
    void          index(uint8_t);
    uint8_t       index() const;
    // last runtime in µs for the execution of the task
    uint32_t      runtime() const;
    // if the task was in need for more time than available based on the runtime estimation
    bool          delayed() const;
    // ticks since last run
    uint16_t      ticks() const;
    
// For backward compatibility of the AP_Param table madness
public /*AP_Param accessor*/:
    struct Settings {
        task_fn_t function;
        #ifndef __AVR__
        const char *name;
        #endif
        uint16_t interval_ticks;
        uint16_t max_time_micros;
    } _settings;
    
private:
    // debugging
    uint32_t      _last_runtime;
    uint8_t       _index;
    uint8_t       _missed_runs;
    bool          _delayed;
    uint16_t      _dt;
    AP_Scheduler* _parent_sched;
    bool          _is_running;
};


//////////////////////////////////////////////////////////////////////////
// SCHEDULER
//////////////////////////////////////////////////////////////////////////
class AP_Scheduler {
friend class AP_Task;

public /*functions*/:
    // initialise scheduler
    void init(const AP_Task::Settings *tasks, uint8_t num_tasks);

    // call when one tick has passed
    void tick(void);

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(uint16_t time_available);

    // return the number of microseconds available for the current task
    uint16_t time_available_usec(void);

    // return debug parameter
    uint8_t debug(void) { return _debug; }

    // return load average, as a number between 0 and 1. 1 means
    // 100% load. Calculated from how much spare time we have at the
    // end of a run()
    float load_average(uint32_t tick_time_usec) const;

    static const struct AP_Param::GroupInfo var_info[];

    // returns whether a task is currently running
    int8_t task_active(int index) const;
    
    // returns the number of tasks in the scheduler
    int8_t tasks() const;

protected /*functions*/:
    void update_ticks();
    void logs_outp();
    void logs_buffer(AP_Task &);
    
private:
    // used to enable scheduler debugging
    AP_Int8 _debug;

    // progmem list of tasks to run
    AP_Task *_tasks;

    // number of tasks in _tasks list
    uint8_t _num_tasks;

    struct TaskExecutionLog {
        uint32_t time_taken;
        uint16_t ticks_since_last_run;
        uint8_t task_id;
        uint8_t flag;
    };

    TaskExecutionLog *_task_execution_log_buffer;

    // number of 'ticks' that have passed (number of times that
    // tick() has been called
    uint16_t _tick_counter;

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
    
    // number of entries in the debug log
    uint8_t _log_count;
    
    // parallelism enabled
    AP_Int8 _parallelism;
};

#endif // AP_SCHEDULER_H
