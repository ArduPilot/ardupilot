#pragma once

#include <stdint.h>
#include <AP_Common/ExpandingString.h>

namespace AP {

class PerfInfo {
public:
    PerfInfo() {}

    // per-task timing information
    struct TaskInfo {
        uint16_t min_time_us;
        uint16_t max_time_us;
        uint32_t elapsed_time_us;
        uint32_t tick_count;
        uint16_t slip_count;
        uint16_t overrun_count;

        void update(uint16_t task_time_us, bool overrun);
        void print(const char* task_name, uint32_t total_time, ExpandingString& str) const;
    };

    /* Do not allow copies */
    CLASS_NO_COPY(PerfInfo);

    void reset();
    void ignore_this_loop();
    void check_loop_time(uint32_t time_in_micros);
    uint16_t get_num_loops() const;
    uint32_t get_max_time() const;
    uint32_t get_min_time() const;
    uint16_t get_num_long_running() const;
    uint32_t get_avg_time() const;
    uint32_t get_stddev_time() const;
    float    get_filtered_time() const;
    float get_filtered_loop_rate_hz() const;
    void set_loop_rate(uint16_t rate_hz);

    void update_logging() const;

    // allocate the array of task statistics for use by @SYS/tasks.txt
    void allocate_task_info(uint8_t num_tasks);
    void free_task_info();
    // whether or not we have task info allocated
    bool has_task_info() { return _task_info != nullptr; }
    // return a task info
    const TaskInfo* get_task_info(uint8_t task_index) const {
        return (_task_info && task_index < _num_tasks) ? &_task_info[task_index] : nullptr;
    }
    // called after each run of a task to update its statistics based on measurements taken by the scheduler
    void update_task_info(uint8_t task_index, uint16_t task_time_us, bool overrun);
    // record that a task slipped
    void task_slipped(uint8_t task_index) {
        if (_task_info && task_index < _num_tasks) {
            _task_info[task_index].overrun_count++;
        }
    }

private:
    uint16_t loop_rate_hz;
    uint16_t overtime_threshold_micros;
    uint16_t loop_count;
    uint32_t max_time; // in microseconds
    uint32_t min_time; // in microseconds
    uint64_t sigma_time;
    uint64_t sigmasquared_time;
    uint16_t long_running;
    uint32_t last_check_us;
    float filtered_loop_time;
    bool ignore_loop;
    // performance monitoring
    uint8_t _num_tasks;
    TaskInfo* _task_info;
};

};
