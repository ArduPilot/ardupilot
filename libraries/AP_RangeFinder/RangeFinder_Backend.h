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
	AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // we declare a virtual destructor so that RangeFinder drivers can
    // override with a custom destructor if need be
    virtual ~AP_RangeFinder_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    virtual void handle_msg(mavlink_message_t *msg) { return; }

    void update_pre_arm_check();

    enum Rotation orientation() const { return (Rotation)params.orientation.get(); }
    uint16_t distance_cm() const { return state.distance_cm; }
    uint16_t voltage_mv() const { return state.voltage_mv; }
    int16_t max_distance_cm() const { return params.max_distance_cm; }
    int16_t min_distance_cm() const { return params.min_distance_cm; }
    int16_t ground_clearance_cm() const { return params.ground_clearance_cm; }
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type() const;
    RangeFinder::RangeFinder_Status status() const;
    RangeFinder::RangeFinder_Type type() const { return (RangeFinder::RangeFinder_Type)params.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    // returns count of consecutive good readings
    uint8_t range_valid_count() const { return state.range_valid_count; }

    // return a 3D vector defining the position offset of the sensor
    // in metres relative to the body frame origin
    const Vector3f &get_pos_offset() const { return params.pos_offset; }

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(RangeFinder::RangeFinder_Status status);

    RangeFinder::RangeFinder_State &state;
    AP_RangeFinder_Params &params;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const = 0;
};
