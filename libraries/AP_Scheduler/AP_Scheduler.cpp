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

#include <AP_HAL.h>
#include <AP_Scheduler.h>
#include <AP_Param.h>

#if defined(_OPENMP)
#include <omp.h>
#endif

#if defined(__AVR__)
    #define TASK_NAME(id) PSTR("unknown")
#else
    #define TASK_NAME(id) _tasks[id]._settings.name
#endif

extern const AP_HAL::HAL& hal;


//////////////////////////////////////////////////////////////////////////
// TASK
//////////////////////////////////////////////////////////////////////////
AP_Task::AP_Task(AP_Scheduler *parent_sched) {
    _parent_sched             = parent_sched;
  
    _settings.interval_ticks  = 0;
    _settings.max_time_micros = 0;
#ifndef __AVR__
    _settings.name            = "unknown";
#endif
    _missed_runs              = 0;
    _last_runtime             = 0;
    _delayed                  = false;
    _dt                       = 0;
    _is_running               = false;
}

AP_Scheduler* AP_Task::parent() const {
    return _parent_sched;
}

void AP_Task::parent(AP_Scheduler *sched) {
    _parent_sched = sched;
}

void AP_Task::index(uint8_t id) {
    _index = id;
}

bool AP_Task::run(uint16_t dt, uint16_t time_avble) { 
    uint32_t start    = hal.scheduler->micros(); 
    // should the task be executed?
    bool toggle_exec  = false;
    bool toggle_dbug  = false;
    // task state members
    _dt               = dt;
    _delayed          = false;
    _is_running       = false;
    
    // not time yet to be executed
    if(dt < _settings.interval_ticks) {
        toggle_exec = true;
    }
    // there was at least one slip in the executions of the task
    else if (dt >= 2 * _settings.interval_ticks) {      
        _missed_runs = (dt / _settings.interval_ticks) - 1;
        if(dt % _settings.interval_ticks == 0) {
            toggle_dbug = true;
        }
    }
    
    // there is not enough time for executions left :(
    if (_settings.max_time_micros > time_avble) {
        _delayed = true;
        toggle_exec = true;
        if(dt % _settings.interval_ticks == 0) {
            toggle_dbug = true;
        }
    }
    
    // add an entry to the debugging log
    if(toggle_dbug && _parent_sched) {
        _parent_sched->logs_buffer(*this);
        _parent_sched->logs_outp();
    }
    
    // Run that task
    if(!toggle_exec) {
        _is_running = true;
#ifdef __AVR__
        task_fn_t func;
        pgm_read_block(&_settings.function, &func, sizeof(func) );
        func();
#else
        _settings.function();
#endif
        _last_runtime = hal.scheduler->micros() - start;
        _is_running   = false;      // task not running anymore
        _missed_runs  = 0;          // reset the missed runs counter if executed :D
        return true;
    }
    
    return false;
}

bool AP_Task::is_running() const {
    return _is_running;
}

uint8_t AP_Task::index() const {
    return _index;
}

uint8_t AP_Task::missed_runs() const {
    return _missed_runs;
}

bool AP_Task::delayed() const {
    return _delayed;
}

uint32_t AP_Task::runtime() const {
    return _last_runtime;
}

uint16_t AP_Task::ticks() const {
    return _dt;
}

//////////////////////////////////////////////////////////////////////////
// AP_Scheduler
//////////////////////////////////////////////////////////////////////////

const AP_Param::GroupInfo AP_Scheduler::var_info[] PROGMEM = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table. The option ShowAll makes all executions be logged as well as slips.
    // @Values: 0:Disabled, 2:ShowSlips, 3:ShowOverruns, 4:ShowAll
    // @User: Advanced
    AP_GROUPINFO("DEBUG",          0, AP_Scheduler, _debug,       0),
    
    // @Param: PARALLELISM
    // @DisplayName: Scheduler using OpenMP
    // @Description: Set to non-zero to enable scheduler multi threading
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PARALLELISM",    0, AP_Scheduler, _parallelism, 0),
    AP_GROUPEND
};

// initialise the scheduler
void AP_Scheduler::init(const AP_Task::Settings *settings, uint8_t num_tasks) {  
    // // Do we want a panic here?
    if(!num_tasks || !settings) {
        return;
    }
    
    // init the debug log
    _log_count = 0;
    // Create the tasks and pass them the settings
    _tasks = new AP_Task[num_tasks];
    for(int i = 0; i < num_tasks; i++) {
        _tasks[i].parent(this);
        _tasks[i].index(i);
        _tasks[i]._settings = settings[i];
    }
    _num_tasks = num_tasks;    
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _task_execution_log_buffer = new AP_Scheduler::TaskExecutionLog[_num_tasks];
    _tick_counter = 0;
}

// one tick has passed
void AP_Scheduler::tick(void) {
    _tick_counter++;
}

int8_t AP_Scheduler::tasks() const {
    return _num_tasks;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint16_t time_available) {
    #if defined(_OPENMP)
        #pragma omp parallel for if(_parallelism)
    #endif
    for (uint8_t i = 0; i < _num_tasks; i++) {      
        // current thread
        AP_Task *cur_task  = &_tasks[i];
        
        // calculate ticks passed since last run
        uint16_t dt = _tick_counter - _last_run[i];
        // Run the task and calculate the time
        if(!cur_task->run(dt, time_available) ) {
            continue;
        }
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        if(cur_task->runtime() >= time_available) {
            logs_buffer(*cur_task);
            logs_outp();
            #if defined(_OPENMP)
                #pragma omp critical
            #endif
            update_ticks();
        }
        else {
            #if defined(_OPENMP)
                #pragma omp atomic
            #endif
            time_available -= cur_task->runtime();
        }
    }
    // Non parallel region: update number of spare microseconds
    _spare_micros += time_available;
}

void AP_Scheduler::logs_buffer(AP_Task &cur_task) {
   _task_execution_log_buffer[_log_count].flag = 0;

   // Debugging  
   switch(_debug) {
      case 0:
          break;
      case 1:
          break;
      // slipped runs
      case 2:
          _task_execution_log_buffer[_log_count].task_id = cur_task.index();
          _task_execution_log_buffer[_log_count].flag |= SCHEDULER_TASK_SLIPPED;
          _task_execution_log_buffer[_log_count].ticks_since_last_run = cur_task.ticks();
          // iterate up
          _log_count++;
          break;
      // everything
      case 3: case 4:        
          _task_execution_log_buffer[_log_count].task_id = cur_task.index();
          _task_execution_log_buffer[_log_count].time_taken = cur_task.runtime();
          _task_execution_log_buffer[_log_count].flag |= SCHEDULER_TASK_EXECUTED;
          if (cur_task.runtime() > _task_time_allowed) {
              _task_execution_log_buffer[_log_count].flag |= SCHEDULER_TASK_OVERRUN;
          }
          // iterate up
          _log_count++;
          break;
      default:
          break;
      
   }
}
 
int8_t AP_Scheduler::task_active(int index) const {
    if(index >= _num_tasks) {
        return -1;
    }
    
    return _tasks[index].is_running();
}
 
void AP_Scheduler::update_ticks() {
    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}
 
void AP_Scheduler::logs_outp() {    
    // output debug log, if any
    for (unsigned int i = 0; i < _log_count; i++) {
        TaskExecutionLog * log  = _task_execution_log_buffer + i;
        uint32_t time_allowed   = pgm_read_word(&_tasks[log->task_id]._settings.max_time_micros);

        if (log->flag & SCHEDULER_TASK_SLIPPED) {
            uint16_t interval_ticks = pgm_read_word(&_tasks[log->task_id]._settings.interval_ticks);
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
    // reset the log count after it was given out
    _log_count = 0;
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec() {
    uint32_t dt = hal.scheduler->micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average(uint32_t tick_time_usec) const {
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
    return used_time / (float)tick_time_usec;
}
