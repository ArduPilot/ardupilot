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

#include "AP_Beacon.h"

#if AP_BEACON_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

class AP_Beacon_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_Beacon_Backend(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    virtual bool healthy() = 0;

    // update
    virtual void update() = 0;

    // set vehicle position
    // pos should be in meters in NED frame from the beacon's local origin
    // accuracy_estimate is also in meters
    void set_vehicle_position(const Vector3f& pos, float accuracy_estimate);

    // set individual beacon distance from vehicle in meters in NED frame
    void set_beacon_distance(uint8_t beacon_instance, float distance);

    // set beacon's position
    // pos should be in meters in NED from the beacon's local origin
    void set_beacon_position(uint8_t beacon_instance, const Vector3f& pos);

    float get_beacon_origin_lat(void) const { return _frontend.origin_lat; }
    float get_beacon_origin_lon(void) const { return _frontend.origin_lon; }
    float get_beacon_origin_alt(void) const { return _frontend.origin_alt; }

protected:

    // references
    AP_Beacon &_frontend;

    // yaw correction
    int16_t orient_yaw_deg; // cached version of orient_yaw parameter
    float orient_cos_yaw = 0.0f;
    float orient_sin_yaw = 1.0f;

    // yaw correction methods
    Vector3f correct_for_orient_yaw(const Vector3f &vector);

    AP_HAL::UARTDriver *uart;
};

#endif  // AP_BEACON_ENABLED
