#include "PerfInfo.h"

#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

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
    log_dropped = DataFlash_Class::instance()->num_dropped();
    sigma_time = 0;
    sigmasquared_time = 0;
}

// ignore_loop - ignore this loop from performance measurements (used to reduce false positive when arming)
void AP::PerfInfo::ignore_this_loop()
{
    ignore_loop = true;
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
    if (time_in_micros > overtime_threshold_us) {
        long_running++;
    }
    sigma_time += time_in_micros;
    sigmasquared_time += time_in_micros * time_in_micros;
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

// get_num_dropped - get number of dropped log messages
uint32_t AP::PerfInfo::get_num_dropped() const
{
    return log_dropped;
}

// get_avg_time - return average loop time (in microseconds)
uint32_t AP::PerfInfo::get_avg_time() const
{
    return (sigma_time / loop_count);
}

// get_stddev_time - return stddev of average loop time (in us)
uint32_t AP::PerfInfo::get_stddev_time() const
{
    return sqrt((sigmasquared_time - (sigma_time*sigma_time)/loop_count) / loop_count);
}

void AP::PerfInfo::update_logging()
{
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "PERF: %u/%u max=%lu min=%lu avg=%lu sd=%lu",
                    (unsigned)get_num_long_running(),
                    (unsigned)get_num_loops(),
                    (unsigned long)get_max_time(),
                    (unsigned long)get_min_time(),
                    (unsigned long)get_avg_time(),
                    (unsigned long)get_stddev_time());
}
