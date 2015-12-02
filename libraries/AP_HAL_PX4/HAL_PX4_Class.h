#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_HAL_PX4.h"
#include "AP_HAL_PX4_Namespace.h"
#include <systemlib/visibility.h>
#include <systemlib/perf_counter.h>

class HAL_PX4 : public AP_HAL::HAL {
public:
    HAL_PX4();
    void run(int argc, char* const argv[], Callbacks* callbacks) const override;
};

void hal_px4_set_priority(uint8_t priority);

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
