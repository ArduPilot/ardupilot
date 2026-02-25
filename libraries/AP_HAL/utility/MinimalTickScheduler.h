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

#pragma once

#include <AP_HAL/AP_HAL.h>

// ----------------------------------------------------------------------------
// MinimalTickScheduler Types
// ----------------------------------------------------------------------------

struct MinimalTickSchedulerTask {
    // User-defined parameters
    Functor<void> function;     // The functor to call
    float rate_hz;              // Desired rate in Hertz (used if interval_ticks is 0)
    bool enabled;               // Whether the task is currently active
    uint16_t interval_ticks;    // Run every N ticks. If 0, rate_hz is used to calculate this.

    // Internal scheduling data
    uint32_t _period_ticks;     // The calculated interval in ticks
    uint32_t _ticks_remaining;  // Counter decremented every update
};

/*
 * Tick-based Scheduler
 * Best for high-speed, jitter-sensitive loops where the calling thread
 * guarantees a fixed frequency (e.g., the main flight loop).
 * - Efficient (simple decrement)
 * - Supports both Hz and Ticks for task intervals
 * - Requires knowing the loop rate in advance
 */
class MinimalTickScheduler {
public:
    MinimalTickScheduler() {}

    // Initialize with a fixed table of tasks
    // @Param: tasks - pointer to the array of tasks (must persist)
    // @Param: num_tasks - number of tasks in the array
    // @Param: loop_rate_hz - the fixed frequency at which update() will be called
    void init(MinimalTickSchedulerTask *tasks, uint8_t num_tasks, uint16_t loop_rate_hz);

    // Template overload to automatically deduce array size
    template <uint8_t N>
    void init(MinimalTickSchedulerTask (&tasks)[N], uint16_t loop_rate_hz) {
        init(tasks, N, loop_rate_hz);
    }

    // Call this at the fixed loop_rate_hz
    // Returns true if any task was executed, false otherwise
    bool update();

    // Enable/Disable a task by index
    void set_task_enabled(uint8_t task_index, bool enabled);

    // Change the rate of a task at runtime
    // @Param: task_index - index of the task to modify
    // @Param: rate_hz - new rate in Hertz (will clear interval_ticks if set)
    void set_task_rate(uint8_t task_index, float rate_hz);

    // Update the main loop rate of the scheduler
    // Recalculates all task intervals to maintain correct frequencies
    // @Param: loop_rate_hz - new loop frequency in Hertz
    void set_loop_rate(uint16_t loop_rate_hz);

    // Run a task immediately and reset its schedule timer
    // This allows event-driven execution without disrupting the rate spacing
    // @Param: task_index - index of the task to run
    void run_task_immediately(uint8_t task_index);

private:
    MinimalTickSchedulerTask *_tasks;
    uint8_t _num_tasks;
    uint16_t _loop_rate_hz;

    void spread_tasks();
};
