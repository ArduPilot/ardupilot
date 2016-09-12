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
 * IRLock.h - IRLock Base Class for Ardupilot
 *
 *  Created on: Nov 10, 2014
 *      Author: MLandes
 */
#pragma once

#include <AP_AHRS/AP_AHRS.h>

#define IRLOCK_MAX_TARGETS 5        // max number of targets that can be detected by IR-LOCK sensor (should match PX4Firmware's irlock driver's IRLOCK_OBJECTS_MAX)

class IRLock
{
public:
    IRLock();
    virtual ~IRLock();

    // init - initialize sensor library
    // library won't be useable unless this is first called
    virtual void init() = 0;

    // true if irlock sensor is online and healthy
    bool healthy() const { return _flags.healthy; }

    // timestamp of most recent data read from the sensor
    uint32_t last_update_ms() const { return _last_update_ms; }

    // returns the number of blocks in the current frame
    size_t num_targets() const { return _num_targets; }

    // retrieve latest sensor data - returns true if new data is available
    virtual bool update() = 0;

    // retrieve body frame x and y angles (in radians) to target
    // returns true if data is available
    bool get_angle_to_target_rad(float &x_angle_rad, float &y_angle_rad) const;
    
    // retrieve body frame unit vector in direction of target
    // returns true if data is available
    bool get_unit_vector_body(Vector3f& ret) const;
    

protected:
    struct AP_IRLock_Flags {
        uint8_t healthy : 1; // true if sensor is healthy
    } _flags;

    // internals
    uint32_t _last_update_ms;
    uint16_t _num_targets;

    // irlock_target_info is a duplicate of the PX4Firmware irlock_s structure
    typedef struct {
        uint64_t timestamp;   // microseconds since system start
        float pos_x;          // x-axis distance from center of image to center of target in units of tan(theta)
        float pos_y;          // y-axis distance from center of image to center of target in units of tan(theta)
        float size_x;         // size of target along x-axis in units of tan(theta)
        float size_y;         // size of target along y-axis in units of tan(theta)
    } irlock_target_info;

    irlock_target_info _target_info[IRLOCK_MAX_TARGETS];
};
