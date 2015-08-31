/*
 * IRLock.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: MLandes
 */

#include "IRLock.h"

// default constructor
IRLock::IRLock() :
    _last_update(0),
    _num_targets(0)
{
    // clear target info
    memset(_target_info, 0, sizeof(_target_info));

    // will be adjusted when init is called
    _flags.healthy = false;
}

IRLock::~IRLock() {}

// get_angle_to_target - retrieve body frame x and y angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
bool IRLock::get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const
{
    // return false if we have no target
    if (_num_targets == 0) {
        return false;
    }

    // use data from first object
    x_angle_rad = _target_info[0].angle_x;
    y_angle_rad = _target_info[0].angle_y;
    return true;
}
