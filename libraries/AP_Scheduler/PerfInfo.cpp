#include "PerfInfo.h"

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>
#include "AP_Scheduler.h"

extern const AP_HAL::HAL& hal;

//
//  high level performance monitoring
//
//  we measure the main loop time
//

// reset - reset all records of loop time to zero
void AP::PerfInfo::reset()
{
    loop_count = 0;
    max_time = 0;
    min_time = 0;
    long_running = 0;
    sigma_time = 0;
    sigmasquared_time = 0;
    if (_task_info != nullptr) {
        memset(_task_info, 0, (_num_tasks) * sizeof(TaskInfo));
    }
}

// ignore_loop - ignore this loop from performance measurements (used to reduce false positive when arming)
void AP::PerfInfo::ignore_this_loop()
{
    ignore_loop = true;
}

// allocate the array of task statistics for use by @SYS/tasks.txt
void AP::PerfInfo::allocate_task_info(uint8_t num_tasks)
{
    _task_info = new TaskInfo[num_tasks];
    if (_task_info == nullptr) {
        DEV_PRINTF("Unable to allocate scheduler TaskInfo\n");
        _num_tasks = 0;
        return;
    }
    _num_tasks = num_tasks;
}

void AP::PerfInfo::free_task_info()
{
    delete[] _task_info;
    _task_info = nullptr;
    _num_tasks = 0;
}

// called after each run of a task to update its statistics based on measurements taken by the scheduler
void AP::PerfInfo::update_task_info(uint8_t task_index, uint16_t task_time_us, bool overrun)
{
    if (_task_info == nullptr) {
        return;
    }

    if (task_index >= _num_tasks) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    TaskInfo& ti = _task_info[task_index];
    ti.update(task_time_us, overrun);
}

void AP::PerfInfo::TaskInfo::update(uint16_t task_time_us, bool overrun)
{
    max_time_us = MAX(max_time_us, task_time_us);
    if (min_time_us == 0) {
        min_time_us = task_time_us;
    } else {
        min_time_us = MIN(min_time_us, task_time_us);
    }
    elapsed_time_us += task_time_us;
    tick_count++;
    if (overrun) {
        overrun_count++;
    }
}

void AP::PerfInfo::TaskInfo::print(const char* task_name, uint32_t total_time, ExpandingString& str) const
{
    uint16_t avg = 0;
    float pct = 0.0f;
    if (tick_count > 0) {
        pct = elapsed_time_us * 100.0f / total_time;
        avg = MIN(uint16_t(elapsed_time_us / tick_count), 9999);
    }
#if AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED
    const char* fmt = "%-32.32s MIN=%4u MAX=%4u AVG=%4u OVR=%3u SLP=%3u, TOT=%4.1f%%\n";
#else
    const char* fmt = "%-16.16s MIN=%4u MAX=%4u AVG=%4u OVR=%3u SLP=%3u, TOT=%4.1f%%\n";
#endif
    str.printf(fmt, task_name,
                unsigned(MIN(min_time_us, 9999)), unsigned(MIN(max_time_us, 9999)), unsigned(avg),
                unsigned(MIN(overrun_count, 999)), unsigned(MIN(slip_count, 999)), pct);
}

// check_loop_time - check latest loop time vs min, max and overtime threshold
void AP::PerfInfo::check_loop_time(uint32_t time_in_micros)
{
    loop_count++;

    // exit if this loop should be ignored
    if (ignore_loop) {
        ignore_loop = false;
        return;
    }

    if( time_in_micros > max_time) {
        max_time = time_in_micros;
    }
    if( min_time == 0 || time_in_micros < min_time) {
        min_time = time_in_micros;
    }
    if (time_in_micros > overtime_threshold_micros) {
        long_running++;
    }
    sigma_time += time_in_micros;
    sigmasquared_time += time_in_micros * time_in_micros;

    /* we keep a filtered loop time for use as G_Dt which is the
       predicted time for the next loop. We remove really excessive
       times from this calculation so as not to throw it off too far
       in case we get a single long loop

       Note that the time we use here is the time between calls to
       check_loop_time() not the time from loop start to loop
       end. This is because we are using the time for time between
       calls to controllers, which has nothing to do with cpu speed.
    */
    const uint32_t now = AP_HAL::micros();
    const uint32_t loop_time_us = now - last_check_us;
    last_check_us = now;
    if (loop_time_us < overtime_threshold_micros + 10000UL) {
        filtered_loop_time = 0.99f * filtered_loop_time + 0.01f * loop_time_us * 1.0e-6f;
    }
}

// get_num_loops: return number of loops used for recording performance
uint16_t AP::PerfInfo::get_num_loops() const
{
    return loop_count;
}

// get_max_time - return maximum loop time (in microseconds)
uint32_t AP::PerfInfo::get_max_time() const
{
    return max_time;
}

// get_min_time - return minumum loop time (in microseconds)
uint32_t AP::PerfInfo::get_min_time() const
{
    return min_time;
}

// get_num_long_running - get number of long running loops
uint16_t AP::PerfInfo::get_num_long_running() const
{
    return long_running;
}

// get_avg_time - return average loop time (in microseconds)
uint32_t AP::PerfInfo::get_avg_time() const
{
    return (sigma_time / loop_count);
}

// get_stddev_time - return stddev of average loop time (in us)
uint32_t AP::PerfInfo::get_stddev_time() const
{
    return sqrtf((sigmasquared_time - (sigma_time*sigma_time)/loop_count) / loop_count);
}

// get_filtered_time - return low pass filtered loop time in seconds
float AP::PerfInfo::get_filtered_time() const
{
    return filtered_loop_time;
}

// return low pass filtered loop rate in hz
float AP::PerfInfo::get_filtered_loop_rate_hz() const
{
    const float filt_time_s = get_filtered_time();
    if (filt_time_s <= 0) {
        return loop_rate_hz;
    }
    return 1.0 / filt_time_s;
}


void AP::PerfInfo::update_logging() const
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "PERF: %u/%u [%lu:%lu] F=%uHz sd=%lu Ex=%lu",
                    (unsigned)get_num_long_running(),
                    (unsigned)get_num_loops(),
                    (unsigned long)get_max_time(),
                    (unsigned long)get_min_time(),
                    (unsigned)(0.5+get_filtered_loop_rate_hz()),
                    (unsigned long)get_stddev_time(),
                    (unsigned long)AP::scheduler().get_extra_loop_us());
}

void AP::PerfInfo::set_loop_rate(uint16_t rate_hz)
{
    // allow a 20% overrun before we consider a loop "slow":
    overtime_threshold_micros = 1000000/rate_hz * 1.2f;

    if (loop_rate_hz != rate_hz) {
        loop_rate_hz = rate_hz;
        filtered_loop_time = 1.0f / rate_hz;
    }
}
