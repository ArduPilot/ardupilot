/*
 * Copyright (C) 2025 ArduPilot
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MinimalScheduler.h"

// ============================================================================
// TickScheduler Implementation
// ============================================================================

TickScheduler::TickScheduler() :
    _tasks(nullptr),
    _num_tasks(0),
    _loop_rate_hz(0)
{
}

void TickScheduler::init(SchedulerTask *tasks, uint8_t num_tasks, uint16_t loop_rate_hz)
{
    if (tasks == nullptr || num_tasks == 0 || loop_rate_hz == 0) {
        return;
    }

    _tasks = tasks;
    _num_tasks = num_tasks;
    _loop_rate_hz = loop_rate_hz;

    // Calculate periods in ticks
    for (uint8_t i = 0; i < _num_tasks; i++) {
        SchedulerTask &t = _tasks[i];
        
        // Priority 1: Explicit Ticks
        if (t.interval_ticks > 0) {
            t._period = t.interval_ticks;
        } 
        // Priority 2: Hertz
        else if (t.rate_hz > 0) {
            if (t.rate_hz > _loop_rate_hz) {
                t.rate_hz = _loop_rate_hz;
            }
            t._period = _loop_rate_hz / t.rate_hz;
        } 
        // Invalid configuration
        else {
            t.enabled = false;
            t._period = 0;
            continue;
        }

        t._next = 0; // Initialize counter
    }

    spread_tasks();
}

bool TickScheduler::update()
{
    if (_tasks == nullptr) {
        return false;
    }

    bool task_ran = false;

    for (uint8_t i = 0; i < _num_tasks; i++) {
        SchedulerTask &t = _tasks[i];

        if (t._period == 0) {
            continue;
        }

        // Decrement ticks remaining
        if (t._next > 0) {
            t._next--;
        }

        // If counter expires
        if (t._next == 0) {
            // Reset counter
            t._next = t._period;

            // Run if enabled
            // Functor::operator bool() checks if the bound method is not null
            if (t.enabled && t.function) {
                t.function();
                task_ran = true;
            }
        }
    }
    
    return task_ran;
}

void TickScheduler::set_task_enabled(uint8_t task_index, bool enabled)
{
    if (task_index < _num_tasks) {
        _tasks[task_index].enabled = enabled;
    }
}

void TickScheduler::spread_tasks()
{
    // Spread tasks based on their calculated period, not just rate_hz
    // This correctly handles mix of Hz-defined and Tick-defined tasks
    for (uint8_t i = 0; i < _num_tasks; i++) {
        SchedulerTask &target = _tasks[i];
        if (target._period == 0) continue;

        uint8_t same_period_count = 0;
        uint8_t my_index = 0;

        for (uint8_t j = 0; j < _num_tasks; j++) {
            if (_tasks[j]._period == target._period) {
                if (j == i) my_index = same_period_count;
                same_period_count++;
            }
        }

        if (same_period_count > 0) {
            // _period is interval in ticks
            // _next is used as the initial offset
            uint32_t spacing = target._period / same_period_count;
            target._next = my_index * spacing;
        }
    }
}

// ============================================================================
// TimeScheduler Implementation
// ============================================================================

TimeScheduler::TimeScheduler() :
    _tasks(nullptr),
    _num_tasks(0),
    _initialized_time(false)
{
}

void TimeScheduler::init(SchedulerTask *tasks, uint8_t num_tasks)
{
    if (tasks == nullptr || num_tasks == 0) {
        return;
    }

    _tasks = tasks;
    _num_tasks = num_tasks;
    _initialized_time = false;

    // Calculate periods in microseconds
    for (uint8_t i = 0; i < _num_tasks; i++) {
        SchedulerTask &t = _tasks[i];
        
        // TimeScheduler ignores interval_ticks as it doesn't know the loop rate
        if (t.rate_hz == 0) {
            t.enabled = false;
            t._period = 0;
            continue;
        }
        t._period = 1000000UL / t.rate_hz;
        t._next = 0; 
    }

    spread_tasks();
}

bool TimeScheduler::update(uint32_t now_us)
{
    if (_tasks == nullptr) {
        return false;
    }

    bool task_ran = false;

    // On the very first update, synchronize all tasks to the current time
    // spread_tasks() has populated _next with the desired start offset (us)
    if (!_initialized_time) {
        for (uint8_t i = 0; i < _num_tasks; i++) {
            SchedulerTask &t = _tasks[i];
            if (t._period > 0) {
                // Initialize "last run time" (_next) so that the task fires 
                // 'offset' microseconds from now.
                // Logic: (now + offset) - last_run >= period
                // last_run = now + offset - period
                t._next = now_us + t._next - t._period;
            }
        }
        _initialized_time = true;
    }

    for (uint8_t i = 0; i < _num_tasks; i++) {
        SchedulerTask &t = _tasks[i];

        if (t._period == 0) {
            continue;
        }

        // Check if the time elapsed since the last run (t._next) exceeds the period
        if (AP_HAL::timeout_expired(t._next, now_us, t._period)) {
            // Move the baseline forward by one period to maintain phase
            t._next += t._period;

            // Catch-up logic:
            // If we are still expired after advancing, it means we missed one or more cycles.
            // Reset the baseline to the current time to avoid a burst of executions.
            // We set it to now_us so that the LAST run is considered to be 'now'.
            if (AP_HAL::timeout_expired(t._next, now_us, t._period)) {
                t._next = now_us;
            }

            // Run if enabled
            // Functor::operator bool() checks if the bound method is not null
            if (t.enabled && t.function) {
                t.function();
                task_ran = true;
            }
        }
    }

    return task_ran;
}

void TimeScheduler::set_task_enabled(uint8_t task_index, bool enabled)
{
    if (task_index < _num_tasks) {
        _tasks[task_index].enabled = enabled;
    }
}

void TimeScheduler::spread_tasks()
{
    // Spread tasks based on their calculated period
    for (uint8_t i = 0; i < _num_tasks; i++) {
        SchedulerTask &target = _tasks[i];
        if (target._period == 0) continue;

        uint8_t same_period_count = 0;
        uint8_t my_index = 0;

        for (uint8_t j = 0; j < _num_tasks; j++) {
            if (_tasks[j]._period == target._period) {
                if (j == i) my_index = same_period_count;
                same_period_count++;
            }
        }

        if (same_period_count > 0) {
            // _period is interval in microseconds
            // _next stores the offset temporarily until the first update()
            uint32_t spacing = target._period / same_period_count;
            target._next = my_index * spacing;
        }
    }
}