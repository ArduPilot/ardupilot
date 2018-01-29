#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_HAL_PX4.h"
#include <AP_HAL/POSIXSemaphores.h>

class PX4::Semaphore : public AP_HAL::POSIXSemaphore {
public:
    bool take(uint32_t timeout_ms) override;
};
#endif // CONFIG_HAL_BOARD
