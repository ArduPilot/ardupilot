#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "AP_HAL_VRBRAIN.h"
#include <AP_HAL/POSIXSemaphores.h>

class VRBRAIN::Semaphore : public AP_HAL::POSIXSemaphore {
public:
    bool take(uint32_t timeout_ms) override;
};
#endif // CONFIG_HAL_BOARD
