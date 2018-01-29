#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL/POSIXSemaphores.h"

class HALSITL::Semaphore : public AP_HAL::POSIXSemaphore {
public:
private:
};
#endif  // CONFIG_HAL_BOARD
