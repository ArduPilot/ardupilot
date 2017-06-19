#include "Sub.h"

//
//  high level performance monitoring
//
//  we measure the main loop time
//

// 400hz loop update rate
#define PERF_INFO_OVERTIME_THRESHOLD_MICROS 3000

static uint16_t perf_info_loop_count;
static uint32_t perf_info_max_time;
static uint32_t perf_info_min_time;
static uint16_t perf_info_long_running;
static uint32_t perf_info_log_dropped;
static bool perf_ignore_loop = false;

// perf_info_reset - reset all records of loop time to zero
void Sub::perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_min_time = 0;
    perf_info_long_running = 0;
    perf_info_log_dropped = DataFlash.num_dropped();
}

// perf_ignore_loop - ignore this loop from performance measurements (used to reduce false positive when arming)
void Sub::perf_ignore_this_loop()
{
    perf_ignore_loop = true;
}

// perf_info_check_loop_time - check latest loop time vs min, max and overtime threshold
void Sub::perf_info_check_loop_time(uint32_t time_in_micros)
{
    perf_info_loop_count++;

    // exit if this loop should be ignored
    if (perf_ignore_loop) {
        perf_ignore_loop = false;
        return;
    }

    if (time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if (perf_info_min_time == 0 || time_in_micros < perf_info_min_time) {
        perf_info_min_time = time_in_micros;
    }
    if (time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS) {
        perf_info_long_running++;
    }
}

// perf_info_get_long_running_percentage - get number of long running loops as a percentage of the total number of loops
uint16_t Sub::perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
uint32_t Sub::perf_info_get_max_time()
{
    return perf_info_max_time;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
uint32_t Sub::perf_info_get_min_time()
{
    return perf_info_min_time;
}

// perf_info_get_num_long_running - get number of long running loops
uint16_t Sub::perf_info_get_num_long_running()
{
    return perf_info_long_running;
}

// perf_info_get_num_dropped - get number of dropped log messages
uint32_t Sub::perf_info_get_num_dropped()
{
    return perf_info_log_dropped;
}
