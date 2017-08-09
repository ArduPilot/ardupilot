/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "RangeFinder.h"

class AP_RangeFinder_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state,
                           MAV_DISTANCE_SENSOR _sensor_type);

    // we declare a virtual destructor so that RangeFinder drivers can
    // override with a custom destructor if need be
    virtual ~AP_RangeFinder_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    MAV_DISTANCE_SENSOR get_sensor_type() const {
        return sensor_type;
    }

    virtual void handle_msg(mavlink_message_t *msg) { return; }

    void update_pre_arm_check();

protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(RangeFinder::RangeFinder_Status status);

    RangeFinder::RangeFinder_State &state;
    MAV_DISTANCE_SENSOR sensor_type;

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;    
};
