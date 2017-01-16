#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AC_Fence/AC_PolyFence_loader.h>

class AP_Proximity_SITL : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_SITL(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const;
    float distance_min() const;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const;

private:
    SITL::SITL *sitl;
    Vector2l *fence;
    AP_Int8 *fence_count;
    AP_Float *fence_alt_max;
    uint32_t last_load_ms;
    AC_PolyFence_loader fence_loader;
    Location current_loc;

    // latest sector updated
    uint8_t last_sector;

    void load_fence(void);

    // get distance in meters to fence in a particular direction in degrees (0 is forward, angles increase in the clockwise direction)
    bool get_distance_to_fence(float angle_deg, float &distance) const;

};
#endif // CONFIG_HAL_BOARD
