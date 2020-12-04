/*
  timestamp jitter correction class
 */

#pragma once

class JitterCorrection {
public:
    // constructor
    JitterCorrection(uint16_t max_lag_ms=500, uint16_t convergence_loops=100);

    // correct an offboard timestamp to a jitter-free local
    // timestamp. See JitterCorrection.cpp for details
    uint64_t correct_offboard_timestamp_usec(uint64_t offboard_usec, uint64_t local_usec);

    // correct an offboard timestamp to a jitter-free local
    // timestamp. See JitterCorrection.cpp for details
    uint32_t correct_offboard_timestamp_msec(uint32_t offboard_ms, uint32_t local_ms);
    
private:
    const uint16_t max_lag_ms;
    const uint16_t convergence_loops;
    int64_t link_offset_usec;
    int64_t min_sample_us;
    bool initialised;
    uint16_t min_sample_counter;
};
