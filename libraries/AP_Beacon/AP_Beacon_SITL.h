#pragma once

#include "AP_Beacon_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <SITL/SITL.h>

class AP_Beacon_SITL : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_SITL(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

private:
    SITL::SITL *sitl;    
    uint8_t next_beacon;
    uint32_t last_update_ms;
};

#endif // CONFIG_HAL_BOARD
