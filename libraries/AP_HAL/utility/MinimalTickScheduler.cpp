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

#include "MinimalTickScheduler.h"

// ============================================================================
// MinimalTickScheduler Implementation
// ============================================================================

void MinimalTickScheduler::init(MinimalTickSchedulerTask *tasks, uint8_t num_tasks, uint16_t loop_rate_hz)
{
    if (tasks == nullptr || num_tasks == 0 || loop_rate_hz == 0) {
        return;
    }

    _tasks = tasks;
    _num_tasks = num_tasks;
    _loop_rate_hz = loop_rate_hz;

    // Calculate periods in ticks
    for (uint8_t i = 0; i < _num_tasks; i++) {
        MinimalTickSchedulerTask &t = _tasks[i];

        // Priority 1: Explicit Ticks
        if (t.interval_ticks > 0) {
            t._period_ticks = t.interval_ticks;
        }
        // Priority 2: Hertz
        else if (t.rate_hz > 0.0f) {
            if (t.rate_hz > _loop_rate_hz) {
                t.rate_hz = _loop_rate_hz;
            }
            // Use float division to calculate ticks, then cast to integer
            t._period_ticks = (uint32_t)(_loop_rate_hz / t.rate_hz);

            // Safety: Ensure at least 1 tick
            if (t._period_ticks == 0) {
                t._period_ticks = 1;
            }
        }
        // Invalid configuration (Rate 0)
        else {
            // We do NOT force disable the task here.
            // This allows a task to be initialized as enabled=true but rate=0,
            // effectively dormant until set_task_rate() is called later.
            t._period_ticks = 0;
            continue;
        }

        // Initialize counter to period (not 0) so it waits one full cycle
        // before the first automatic run. This gives time for events to take over.
        t._ticks_remaining = t._period_ticks;
    }

    spread_tasks();
}

bool MinimalTickScheduler::update()
{
    if (_tasks == nullptr) {
        return false;
    }

    bool task_ran = false;

    for (uint8_t i = 0; i < _num_tasks; i++) {
        MinimalTickSchedulerTask &t = _tasks[i];

        if (t._period_ticks == 0) {
            continue;
        }

        // Decrement ticks remaining
        if (t._ticks_remaining > 0) {
            t._ticks_remaining--;
        }

        // If counter expires
        if (t._ticks_remaining == 0) {

            // Enforce running only one task per update call to prevent CPU spikes.
            // If a task is ready but another task has already run in this tick,
            // we skip execution. Because we DO NOT reset _ticks_remaining,
            // it stays at 0 and will be caught immediately in the next update().
            if (!task_ran) {
                // Reset counter
                t._ticks_remaining = t._period_ticks;

                // Run if enabled
                if (t.enabled && t.function) {
                    t.function();
                    task_ran = true;
                }
            }
        }
    }

    return task_ran;
}

void MinimalTickScheduler::set_task_enabled(uint8_t task_index, bool enabled)
{
    if (task_index < _num_tasks) {
        _tasks[task_index].enabled = enabled;
    }
}

void MinimalTickScheduler::set_task_rate(uint8_t task_index, float rate_hz)
{
    if (_tasks == nullptr || task_index >= _num_tasks || _loop_rate_hz == 0) {
        return;
    }

    MinimalTickSchedulerTask &t = _tasks[task_index];

    // Update the rate
    t.rate_hz = rate_hz;
    // Clear explicit ticks to ensure rate_hz takes precedence
    t.interval_ticks = 0;

    if (t.rate_hz <= 0.0f) {
        t._period_ticks = 0;
        return;
    }

    if (t.rate_hz > _loop_rate_hz) {
        t.rate_hz = _loop_rate_hz;
    }

    // Recalculate period
    t._period_ticks = (uint32_t)(_loop_rate_hz / t.rate_hz);
    if (t._period_ticks == 0) {
        t._period_ticks = 1;
    }

    // Safety: If the new period is shorter than the current wait time,
    // clamp the wait time to the new period.
    if (t._ticks_remaining > t._period_ticks) {
        t._ticks_remaining = t._period_ticks;
    }
}

void MinimalTickScheduler::set_loop_rate(uint16_t loop_rate_hz)
{
    if (loop_rate_hz == 0 || _loop_rate_hz == loop_rate_hz) {
        return;
    }

    _loop_rate_hz = loop_rate_hz;

    // Recalculate periods for all tasks based on new loop rate
    if (_tasks == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < _num_tasks; i++) {
        MinimalTickSchedulerTask &t = _tasks[i];

        // Priority 1: Explicit Ticks (ticks remain constant, but effective rate changes)
        // If the user wants 10 ticks, it stays 10 ticks regardless of loop rate.
        if (t.interval_ticks > 0) {
            t._period_ticks = t.interval_ticks;
        }
        // Priority 2: Hertz (recalculate ticks to maintain same Hz)
        else if (t.rate_hz > 0.0f) {
            float effective_rate = t.rate_hz;
            if (effective_rate > _loop_rate_hz) {
                effective_rate = _loop_rate_hz;
            }

            t._period_ticks = (uint32_t)(_loop_rate_hz / effective_rate);
            if (t._period_ticks == 0) {
                t._period_ticks = 1;
            }

            // Safety: clamp remaining ticks if new period is shorter
            if (t._ticks_remaining > t._period_ticks) {
                t._ticks_remaining = t._period_ticks;
            }
        }
    }
}

void MinimalTickScheduler::run_task_immediately(uint8_t task_index)
{
    if (_tasks == nullptr || task_index >= _num_tasks) {
        return;
    }

    MinimalTickSchedulerTask &t = _tasks[task_index];

    // Only run if the task is configured and enabled
    if (t.enabled && t.function && t._period_ticks > 0) {
        t.function();

        // Reset the countdown to the full period.
        // Special case: If period is 1 tick, resetting to 1 would cause update()
        // (which decrements immediately) to see 0 and run it again in the same cycle.
        // To safely call update() after this, we must ensure _ticks_remaining > 1
        // if we want to skip the immediate next update.
        if (t._period_ticks == 1) {
            t._ticks_remaining = 2;
        } else {
            t._ticks_remaining = t._period_ticks;
        }
    }
}

void MinimalTickScheduler::spread_tasks()
{
    // Spread tasks based on their calculated period
    for (uint8_t i = 0; i < _num_tasks; i++) {
        MinimalTickSchedulerTask &target = _tasks[i];
        if (target._period_ticks == 0) continue;

        uint8_t same_period_count = 0;
        uint8_t my_index = 0;

        for (uint8_t j = 0; j < _num_tasks; j++) {
            if (_tasks[j]._period_ticks == target._period_ticks) {
                if (j == i) my_index = same_period_count;
                same_period_count++;
            }
        }

        if (same_period_count > 0) {
            // Calculate offset based on period
            uint32_t spacing = target._period_ticks / same_period_count;

            // If we initialize to 'spacing', update() decrements immediately.
            // If spacing is 0 (first item), it runs immediately.
            // To spread them out correctly from t=0, we assign the remaining ticks directly.
            target._ticks_remaining = my_index * spacing;

            // Ensure nothing starts at 0 to allow event-driven tasks to take precedence if needed.
            if (target._ticks_remaining == 0) {
               target._ticks_remaining = target._period_ticks;
            }
        }
    }
}
