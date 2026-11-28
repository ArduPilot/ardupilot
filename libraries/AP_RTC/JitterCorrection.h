/*
  timestamp jitter correction class
 */

#pragma once

class JitterCorrection {
public:
    // constructor
    JitterCorrection(uint16_t max_lag_ms=500, uint16_t convergence_loops=100);

    // correct an offboard timestamp to a jitter-free local
    // timestamp. See JitterCorrection.cpp for details. local_exact is
    // true when local_usec is when the sample was taken, not when it arrived
    uint64_t correct_offboard_timestamp_usec(uint64_t offboard_usec, uint64_t local_usec, bool local_exact=false);

    // correct an offboard timestamp to a jitter-free local
    // timestamp. See JitterCorrection.cpp for details
    uint32_t correct_offboard_timestamp_msec(uint32_t offboard_ms, uint32_t local_ms);

    int64_t get_link_offset_usec(void) const { return link_offset_usec; }

    // true if the last corrected timestamp used an offset set by a local_exact sample
    bool last_estimate_exact(void) const { return estimate_exact; }

private:
    const uint16_t max_lag_ms;
    const uint16_t convergence_loops;
    int64_t link_offset_usec;
    int64_t min_sample_us;
    bool initialised;
    bool link_offset_exact;
    bool min_sample_exact;
    bool estimate_exact;
    uint16_t min_sample_counter;
};
