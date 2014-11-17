// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  high level performance monitoring
//
//  we measure the main loop time
//

#if MAIN_LOOP_RATE == 400
 # define PERF_INFO_OVERTIME_THRESHOLD_MICROS 3000
#else
 # define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500
#endif

uint16_t perf_info_loop_count;
uint32_t perf_info_max_time;
uint16_t perf_info_long_running;

// perf_info_reset - reset all records of loop time to zero
void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_long_running = 0;
}

// perf_info_check_loop_time - check latest loop time vs min, max and overtime threshold
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

// perf_info_get_long_running_percentage - get number of long running loops as a percentage of the total number of loops
uint16_t perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
uint32_t perf_info_get_max_time()
{
    return perf_info_max_time;
}

// perf_info_get_num_long_running - get number of long running loops
uint16_t perf_info_get_num_long_running()
{
    return perf_info_long_running;
}
