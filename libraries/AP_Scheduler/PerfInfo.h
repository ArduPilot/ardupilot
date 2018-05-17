#pragma once

#include <stdint.h>

namespace AP {

class PerfInfo {
public:
    PerfInfo() {}

    /* Do not allow copies */
    PerfInfo(const PerfInfo &other) = delete;
    PerfInfo &operator=(const PerfInfo&) = delete;

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
    void set_loop_rate(uint16_t rate_hz);

    void update_logging();

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

};

};
