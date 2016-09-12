/*
 * IRLock.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: MLandes
 */

#include "IRLock.h"

// default constructor
IRLock::IRLock() :
    _last_update_ms(0),
    _num_targets(0)
{
    // clear target info
    memset(_target_info, 0, sizeof(_target_info));

    // will be adjusted when init is called
    _flags.healthy = false;
}

IRLock::~IRLock() {}

// retrieve body frame x and y angles (in radians) to target
// returns true if data is available
bool IRLock::get_angle_to_target_rad(float &x_angle_rad, float &y_angle_rad) const
{
    // return false if we have no target
    if (_num_targets == 0) {
        return false;
    }

    // use data from first (largest) object
    x_angle_rad = atanf(_target_info[0].pos_x);
    y_angle_rad = atanf(_target_info[0].pos_y);
    return true;
}

// retrieve body frame unit vector in direction of target
// returns true if data is available
bool IRLock::get_unit_vector_body(Vector3f& ret) const
{
    // return false if we have no target
    if (_num_targets == 0) {
        return false;
    }

    // use data from first (largest) object
    ret.x = -_target_info[0].pos_y;
    ret.y = _target_info[0].pos_x;
    ret.z = 1.0f;
    ret /= ret.length();
    return true;
}