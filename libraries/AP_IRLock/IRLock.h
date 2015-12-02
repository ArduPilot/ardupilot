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

#ifndef __IRLOCK_H__
#define __IRLOCK_H__

#include <AP_AHRS/AP_AHRS.h>

#define IRLOCK_MAX_TARGETS 5        // max number of targets that can be detected by IR-LOCK sensor (should match PX4Firmware's irlock driver's IRLOCK_OBJECTS_MAX)
#define IRLOCK_TIMEOUT_MS  100      // target info times out after 0.1 seconds

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
    uint32_t last_update() const { return _last_update; }

    // returns the number of blocks in the current frame
    size_t num_targets() const { return _num_targets; }

    // retrieve latest sensor data - returns true if new data is available
    virtual bool update() = 0;

    // get_angle_to_target - retrieve body frame x and y angles (in radians) to target
    //  returns true if angles are available, false if not (i.e. no target)
    bool get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const;

protected:
    struct AP_IRLock_Flags {
        uint8_t healthy : 1; // true if sensor is healthy
    } _flags;

    // internals
    uint32_t _last_update;
    uint16_t _num_targets;

    // irlock_target_info is a duplicate of the PX4Firmware irlock_s structure
    typedef struct {
        uint64_t timestamp;     // time target was seen in microseconds since system start
        uint16_t target_num;    // target number prioritised by size (largest is 0)
        float angle_x;          // x-axis angle in radians from center of image to center of target
        float angle_y;          // y-axis angle in radians from center of image to center of target
        float size_x;           // size in radians of target along x-axis
        float size_y;           // size in radians of target along y-axis
    } irlock_target_info;

    irlock_target_info _target_info[IRLOCK_MAX_TARGETS];
};


#endif /* __IRLOCK_H__ */
