/**
 * @file perf_info.pde
 *
 * @brief high level performance monitoring measuring the main loop time
 */

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500

uint16_t perf_info_loop_count;
uint32_t perf_info_max_time;
uint16_t perf_info_long_running;

/**
 * perf_info_reset
 *
 * @return void
 *
 * @brief reset all records of loop time to zero
 */
void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_long_running = 0;
}

/**
 * perf_info_check_loop_time
 *
 * @param uint32_t time_in_micros Time in micro seconds
 * @return void
 *
 * @brief check latest loop time vs min, max and overtime threshold. Called to update performance tracker.
 */
void perf_info_check_loop_time(uint32_t time_in_micros)
{
    perf_info_loop_count++;
    if( time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if( time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS ) {
        perf_info_long_running++;
    }
}

/**
 * perf_info_get_num_loops
 *
 * @return uint16_t Number of loops that have been sampled
 *
 * @brief get number of loops that have been sampled so far.  Use with 
 * perf_info_get_num_long_running() to work out percentage of loops over time.
 */
uint16_t perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

/**
 * perf_info_get_max_time
 *
 * @return uint32_t
 *
 * @brief return maximum loop time experienced (in microseconds)
 */
uint32_t perf_info_get_max_time()
{
    return perf_info_max_time;
}

/**
 * perf_info_get_num_long_running
 *
 * @return uint16_t number of long running loops
 *
 * @brief get number of long running loops
 */
uint16_t perf_info_get_num_long_running()
{
    return perf_info_long_running;
}
