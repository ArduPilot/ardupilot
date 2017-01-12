#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

class HAL_URUS : public AP_HAL::HAL {
public:
    HAL_URUS();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
