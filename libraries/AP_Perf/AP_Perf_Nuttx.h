#pragma once
#include "AP_Perf_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
class AP_Perf_Nuttx : public AP_Perf_Backend {
    virtual perf_counter_t add(perf_counter_type type, const char *name) override;
    virtual void begin(perf_counter_t pc) override;
    virtual void end(perf_counter_t pc) override;
    virtual void count(perf_counter_t pc) override;
};
#endif