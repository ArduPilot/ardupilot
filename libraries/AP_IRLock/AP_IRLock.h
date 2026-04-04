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

/*
 * AP_IRLock.h - IRLock Base Class for Ardupilot
 *
 *  Created on: Nov 10, 2014
 *      Author: MLandes
 */
#pragma once

#include "AP_IRLock_config.h"

#if AP_IRLOCK_ENABLED

#include <AP_Math/AP_Math.h>

class AP_IRLock
{
public:
    // init - initialize sensor library
    // library won't be useable unless this is first called
    virtual void init(int8_t bus) = 0;

    // true if irlock sensor is online and healthy
    bool healthy() const { return _flags.healthy; }

    // timestamp of most recent data read from the sensor
    uint32_t last_update_ms() const { return _last_update_ms; }

    // returns the number of blocks in the current frame
    size_t num_targets() const { return _flags.healthy?1:0; }

    // retrieve latest sensor data - returns true if new data is available
    virtual bool update() = 0;

    // retrieve body frame unit vector in direction of target
    // returns true if data is available
    bool get_unit_vector_body(Vector3f& ret) const;
    

protected:
    struct AP_IRLock_Flags {
        bool healthy; // true if sensor is healthy
    } _flags;

    // internals
    uint32_t _last_update_ms;

    // irlock_target_info is a duplicate of the PX4Firmware irlock_s structure
    typedef struct {
        uint32_t timestamp;   // milliseconds since system start
        float pos_x;          // x-axis distance from center of image to center of target in units of tan(theta)
        float pos_y;          // y-axis distance from center of image to center of target in units of tan(theta)
        float pos_z;
    } irlock_target_info;

    irlock_target_info _target_info;
};

#endif  // AP_IRLOCK_ENABLED
