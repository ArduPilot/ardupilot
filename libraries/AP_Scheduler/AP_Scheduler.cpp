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

#if defined(__AVR__)
    #define TASK_NAME(id) PSTR("unknown")
#else
    #define TASK_NAME(id) _tasks[id]._settings.name
#endif

extern const AP_HAL::HAL& hal;

int8_t AP_Scheduler::current_task = -1;


//////////////////////////////////////////////////////////////////////////
// TASK
//////////////////////////////////////////////////////////////////////////
AP_Task::AP_Task() {
    _settings.interval_ticks  = 0;
    _settings.max_time_micros = 0;
    _settings.name            = "unknown";
    _missed_runs              = 0;
}

void AP_Task::index(uint8_t id) {
    _index = id;
}

bool AP_Task::is_running(uint16_t dt, uint16_t time_avble) { 
    uint32_t start  = hal.scheduler->micros(); 
    bool cancel     = false;
    
    _missed_runs    = 0;
    _exceeded       = false;
    
    if(dt < _settings.interval_ticks) {
        _last_time_taken = 0;
        _missed_runs = (dt / _settings.interval_ticks) - 1;
        cancel = true;
    }
    
    if (_settings.max_time_micros > time_avble) {
        _exceeded = true;
        cancel = true;
    }
    
    // Run that task
    if(!cancel) {
        _settings.function();
        _last_time_taken  = hal.scheduler->micros() - start;
        return true;
    }
    
    return false;
}

uint8_t AP_Task::index() const {
    return _index;
}

uint8_t AP_Task::missed_runs() const {
    return _missed_runs;
}

bool AP_Task::time_exceeded() const {
    return _exceeded;
}

uint8_t AP_Task::last_time_taken() const {
    return _last_time_taken;
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
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),
    AP_GROUPEND
};

// initialise the scheduler
void AP_Scheduler::init(const AP_Task::Settings *settings, uint8_t num_tasks) {
    _tasks = new AP_Task[num_tasks];
    for(int i = 0; i < num_tasks; i++) {
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

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint16_t time_available) {
    uint8_t  log_count    = 0;
    for (uint8_t i = 0; i < _num_tasks; i++) {
        current_task      = i;
        AP_Task cur_task  = _tasks[i];
        uint16_t dt       = _tick_counter - _last_run[i];
        
        // Run the task and calculate the time
        if(!cur_task.is_running(dt, time_available) ) {
            logs_buffer(cur_task, dt, log_count);     // debugging
            continue;
        }
        _last_run[i] = _tick_counter;
        current_task = -1;

        // work out how long the event actually took
        if (cur_task.last_time_taken() >= time_available) {
            update_ticks();
            logs_outp(log_count);                 // debugging
            return;
        }
        time_available -= cur_task.last_time_taken();
    }

    // update number of spare microseconds
    _spare_micros += time_available;
}

void AP_Scheduler::logs_buffer(AP_Task &cur_task, uint16_t dt, uint8_t &log_count) {
   _task_execution_log_buffer[log_count].flag = 0;

   // Debugging  
   switch(_debug) {
      case 0:
          break;
      case 1:
          break;
      // slipped runs
      case 2:
          if (!cur_task.missed_runs() ) {
              break;
          }
          
          _task_execution_log_buffer[log_count].task_id = cur_task.index();
          _task_execution_log_buffer[log_count].flag |= SCHEDULER_TASK_SLIPPED;
          _task_execution_log_buffer[log_count].ticks_since_last_run = dt;
          
          // iterate up
          log_count++;
          break;
      // everything
      case 3: case 4:
          // if the event overran or _debug set to log all executions
          if (!cur_task.time_exceeded() ) {
              break;
          }
          
          _task_execution_log_buffer[log_count].task_id = cur_task.index();
          _task_execution_log_buffer[log_count].time_taken = cur_task.last_time_taken();
          _task_execution_log_buffer[log_count].flag |= SCHEDULER_TASK_EXECUTED;
          
          if (cur_task.last_time_taken() > _task_time_allowed) {
              _task_execution_log_buffer[log_count].flag |= SCHEDULER_TASK_OVERRUN;
          }
          
          // iterate up
          log_count++;
          break;
      default:
          break;
      
   }
 }
 
void AP_Scheduler::update_ticks() {
    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}
 
void AP_Scheduler::logs_outp(uint8_t &log_count) {
    // output debug log, if any
    for (unsigned int i = 0; i < log_count; i++) {
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
    log_count = 0;
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
