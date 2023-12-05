#pragma once

#ifndef AP_Follow_Location_H
#define AP_Follow_Location_H

#include "GCS_Mavlink.h"
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>

class AP_Follow_Location{
    friend class Copter; // for access to _chan in parameter declarations
public:
    AP_Follow_Location();
    bool get_location();
    bool check_location();
    void _init();
    void update_destination();
    void update_velocity();
private:
    Location NewLoc;
    Location StartLoc;
    Vector3f vel;
    int count;
    int previous_count;
    int range;
    bool DoesItWork = true;
    float kp;
    float x1;
    float y1;
    float z1;
    double wp_len;
};
#endif // AP_Follow_Location_H