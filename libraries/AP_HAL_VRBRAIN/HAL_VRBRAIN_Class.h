#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "AP_HAL_VRBRAIN.h"
#include "AP_HAL_VRBRAIN_Namespace.h"
#include <systemlib/visibility.h>
#include <systemlib/perf_counter.h>

class HAL_VRBRAIN : public AP_HAL::HAL {
public:
    HAL_VRBRAIN();
    void run(int argc, char* const argv[], Callbacks* callbacks) const override;
};

void hal_vrbrain_set_priority(uint8_t priority);

#endif // CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
