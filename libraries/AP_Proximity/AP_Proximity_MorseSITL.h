#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>

class AP_Proximity_MorseSITL : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:
    SITL::SITL *sitl = AP::sitl();
    float distance_maximum;
};
#endif // CONFIG_HAL_BOARD
