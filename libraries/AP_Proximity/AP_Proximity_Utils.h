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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_PROXIMITY_ENABLED
#define HAL_PROXIMITY_ENABLED (!HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_PROXIMITY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>

#define PROXIMITY_GND_DETECT_THRESHOLD 1.0f // set ground detection threshold to be 1 meters
#define PROXIMITY_ALT_DETECT_TIMEOUT_MS 500 // alt readings should arrive within this much time

class AP_Proximity_Utils
{
public:

    // store rangefinder values
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm);

    // check if a reading should be ignored because it falls into an ignore area (check_for_ign_area should be sent as false if this check is not needed)
    // pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle)
    // yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
    // Also checks if obstacle is near land or out of range
    // angles should be in degrees and in the range of 0 to 360, distance should be in meteres
    bool ignore_reading(float pitch, float yaw, float distance_m, bool check_for_ign_area, float max_range_m, float min_range_m) const;

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

protected:

    // used for ground detection
    uint32_t _last_downward_update_ms;
    bool     _rangefinder_use;
    bool     _rangefinder_healthy;
    float    _rangefinder_alt;

};

#endif // HAL_PROXIMITY_ENABLED