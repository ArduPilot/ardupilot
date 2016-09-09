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

extern const AP_HAL::HAL& hal;
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

// constructor
template <class Vehicle>
AP_Scheduler<Vehicle>::AP_Scheduler(Vehicle *parent, const uint16_t loop_rate) : _parent(parent)
{
    _tasks = nullptr;
    _last_run = nullptr;
  
    AP_Param::setup_object_defaults(this, var_info);
    _loop_rate_hz.set_and_save(loop_rate);

    // only allow 50 to 400 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz.set(50);
    } else if (_loop_rate_hz > 400) {
        _loop_rate_hz.set(400);
    }
}

// initialise the scheduler
template <class Vehicle>
AP_Scheduler<Vehicle>::~AP_Scheduler() {
    if(_last_run) {
        delete [] _last_run;
    }
    if(_perf_counters) {
        delete [] _perf_counters;
    }
}

// initialise the scheduler
template <class Vehicle>
void AP_Scheduler<Vehicle>::init(const AP_Task<Vehicle> *tasks, uint8_t num_tasks)
{
    if(tasks == nullptr || num_tasks == 0) {
        return;
    }
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
}

// one tick has passed
template <class Vehicle>
void AP_Scheduler<Vehicle>::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
template <class Vehicle>
void AP_Scheduler<Vehicle>::run(uint32_t time_available)
{
    // break condition
    if(_tasks == nullptr) {
        return;
    }
  
    if (_debug > 3 && _perf_counters == nullptr) {
        _perf_counters = new AP_HAL::Util::perf_counter_t[_num_tasks];
        for (uint8_t i=0; i<_num_tasks && _perf_counters != nullptr; i++) {
            _perf_counters[i] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _tasks[i]._name);
        }
    }

    for (uint8_t i=0; i<_num_tasks && _tasks[i]._max_time_micros <= time_available; i++) {
        const uint16_t dt = _tick_counter - _last_run[i];
        const uint16_t interval_ticks = (_loop_rate_hz / _tasks[i]._rate_hz) < 1 ? 1 : (_loop_rate_hz / _tasks[i]._rate_hz);
                
        if(dt < interval_ticks) {
            continue;
        }
    
        // run it
        _task_time_started = AP_HAL::micros();
        current_task = i;
        if (_debug > 3 && _perf_counters[i]) {
            hal.util->perf_begin(_perf_counters[i]);
        }

        // call that function :)
        if(_parent) {
            task_fn_t func = _tasks[i]._function;
            (_parent->*func)();
        }
        
        if (_debug > 3 && _perf_counters[i]) {
            hal.util->perf_end(_perf_counters[i]);
        }
        current_task = -1;
        
        // record the tick counter when we ran. This drives
        // when we next run the event
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        const uint32_t time_taken = AP_HAL::micros() - _task_time_started;

        // we've slipped a whole run of this task!
        if (dt >= interval_ticks*2 && _debug > 1) {
            print_task_info("Scheduler slip task", i, dt, interval_ticks);
        }
        // the event overran!
        if (time_taken > _tasks[i]._max_time_micros && _debug > 4) {
            print_task_info("Scheduler overrun task", i, dt, time_taken);
        }

        if (time_taken >= time_available) {
            update_spare_ticks(0);
            return;
        }
        time_available -= time_taken;
    }

    // update number of spare microseconds
    update_spare_ticks(time_available);
}

template <class Vehicle>
void AP_Scheduler<Vehicle>::print_task_info(const char *cstr, const unsigned iter, const unsigned dt, const unsigned var) const {
            ::printf("%s[%u-%s] (%u/%u/%u)\n",
                      cstr,
                      (unsigned)iter,
                      _tasks[iter]._name,
                      (unsigned)dt,
                      (unsigned)var,
                      (unsigned)_tasks[iter]._max_time_micros);                      
}

template <class Vehicle>
void AP_Scheduler<Vehicle>::update_spare_ticks(const uint32_t time_available) {
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
template <class Vehicle>
int32_t AP_Scheduler<Vehicle>::time_available_usec(void)
{
    int32_t dt = static_cast<int32_t>(AP_HAL::micros() - _task_time_started);

    if(current_task < 0) {
        return -1;
    }

    if (dt > _tasks[current_task]._max_time_micros) {
        return 0;
    }

    return _tasks[current_task]._max_time_micros - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
template <class Vehicle>
float AP_Scheduler<Vehicle>::load_average(const uint32_t tick_time_usec) const
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
    return static_cast<float>(used_time) / static_cast<float>(tick_time_usec);
}
