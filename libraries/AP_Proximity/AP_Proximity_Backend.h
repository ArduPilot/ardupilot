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

#include "AP_Proximity.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_Common/AP_Common.h>

#define PROXIMITY_GND_DETECT_THRESHOLD 1.0f // set ground detection threshold to be 1 meters
#define PROXIMITY_ALT_DETECT_TIMEOUT_MS 500 // alt readings should arrive within this much time

class AP_Proximity_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_Proximity_Params &_params);

    // we declare a virtual destructor so that Proximity drivers can
    // override with a custom destructor if need be
    virtual ~AP_Proximity_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    // get maximum and minimum distances (in meters) of sensor
    virtual float distance_max() const = 0;
    virtual float distance_min() const = 0;

    // get distance upwards in meters. returns true on success
    virtual bool get_upward_distance(float &distance) const { return false; }

    // handle mavlink messages
    virtual void handle_msg(const mavlink_message_t &msg) {}

    // store rangefinder values
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm);

protected:

    // set status and update valid_count
    void set_status(AP_Proximity::Status status);

    // correct an angle (in degrees) based on the orientation and yaw correction parameters
    float correct_angle_for_orientation(float angle_degrees) const;

    // check if a reading should be ignored because it falls into an ignore area (check_for_ign_area should be sent as false if this check is not needed)
    // pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle)
    // yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
    // Also checks if obstacle is near land or out of range
    // angles should be in degrees and in the range of 0 to 360, distance should be in meteres
    bool ignore_reading(float pitch, float yaw, float distance_m, bool check_for_ign_area = true) const;
    bool ignore_reading(float yaw, float distance_m, bool check_for_ign_area = true) const { return ignore_reading(0.0f, yaw, distance_m, check_for_ign_area); }

    // get alt from rangefinder in meters. This reading is corrected for vehicle tilt
    bool get_rangefinder_alt(float &alt_m) const;

    // Check if Obstacle defined by body-frame yaw and pitch is near ground
    bool check_obstacle_near_ground(float pitch, float yaw, float distance) const;

    // database helpers. All angles are in degrees
    static bool database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned);
    // Note: "angle" refers to yaw (in body frame) towards the obstacle
    static void database_push(float angle, float distance);
    static void database_push(float angle, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned) {
        database_push(angle, 0.0f, distance, timestamp_ms, current_pos, body_to_ned);
    };
    static void database_push(float angle, float pitch, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned);

    // used for ground detection
    uint32_t _last_downward_update_ms;
    bool     _rangefinder_use;
    bool     _rangefinder_healthy;
    float    _rangefinder_alt;

    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;   // reference to this instances state
    AP_Proximity_Params &params;            // parameters for this backend
};

#endif // HAL_PROXIMITY_ENABLED
