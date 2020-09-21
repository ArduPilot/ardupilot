#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#include <AP_HAL/utility/Socket.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>

class AP_Proximity_TeraRangerTowerEvoGazeboSITL : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_TeraRangerTowerEvoGazeboSITL(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:
    SocketAPM sock;

    uint32_t _last_timestamp;
};
#endif // CONFIG_HAL_BOARD
