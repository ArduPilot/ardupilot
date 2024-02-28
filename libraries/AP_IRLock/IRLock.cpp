/*
 * IRLock.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: MLandes
 */

#include "IRLock.h"

// retrieve body frame unit vector in direction of target
// returns true if data is available
bool IRLock::get_unit_vector_body(Vector3f& ret) const
{
    // return false if we have no target
    if (!_flags.healthy) {
        return false;
    }

    // use data from first (largest) object
    ret.x = -_target_info.pos_y;
    ret.y = _target_info.pos_x;
    ret.z = _target_info.pos_z;
    ret /= ret.length();
    return true;
}
