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
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include "AP_Proximity_Boundary_3D.h"

#define PROXIMITY_GND_DETECT_THRESHOLD 1.0f // set ground detection threshold to be 1 meters
#define PROXIMITY_ALT_DETECT_TIMEOUT_MS 500 // alt readings should arrive within this much time
#define PROXIMITY_BOUNDARY_3D_TIMEOUT_MS 750 // we should check the 3D boundary faces after every this many ms

class AP_Proximity_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // we declare a virtual destructor so that Proximity drivers can
    // override with a custom destructor if need be
    virtual ~AP_Proximity_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    // timeout faces that have not received data recently and update filter frequencies
    void boundary_3D_checks();

    // get maximum and minimum distances (in meters) of sensor
    virtual float distance_max() const = 0;
    virtual float distance_min() const = 0;

    // get distance upwards in meters. returns true on success
    virtual bool get_upward_distance(float &distance) const { return false; }

    // handle mavlink DISTANCE_SENSOR messages
    virtual void handle_msg(const mavlink_message_t &msg) {}

    // get total number of obstacles, used in GPS based Simple Avoidance
    uint8_t get_obstacle_count() { return boundary.get_obstacle_count(); }
    
    // get vector to obstacle based on obstacle_num passed, used in GPS based Simple Avoidance
    bool get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const { return boundary.get_obstacle(obstacle_num, vec_to_obstacle); }
    
    // returns shortest distance to "obstacle_num" obstacle, from a line segment formed between "seg_start" and "seg_end"
    // used in GPS based Simple Avoidance
    bool closest_point_from_segment_to_obstacle(const uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const { return boundary.closest_point_from_segment_to_obstacle(obstacle_num , seg_start, seg_end, closest_point); }

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const { return boundary.get_closest_object(angle_deg, distance); }

    // get number of objects, angle and distance - used for non-GPS avoidance
    uint8_t get_horizontal_object_count() const {return boundary.get_horizontal_object_count(); }
    bool get_horizontal_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const { return boundary.get_horizontal_object_angle_and_distance(object_number, angle_deg, distance); }

    // get distances in 8 directions. used for sending distances to ground station
    bool get_horizontal_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const;

    // get raw and filtered distances in 8 directions per layer. used for logging
    bool get_active_layer_distances(uint8_t layer, AP_Proximity::Proximity_Distance_Array &prx_dist_array, AP_Proximity::Proximity_Distance_Array &prx_filt_dist_array) const;

    // get number of layers
    uint8_t get_num_layers() const { return boundary.get_num_layers(); }

    // store rangefinder values
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm);

protected:

    // set status and update valid_count
    void set_status(AP_Proximity::Status status);

    // correct an angle (in degrees) based on the orientation and yaw correction parameters
    float correct_angle_for_orientation(float angle_degrees) const;
    
    // check if a reading should be ignored because it falls into an ignore area
    // angles should be in degrees and in the range of 0 to 360
    bool ignore_reading(uint16_t angle_deg, float distance_m) const;

    // get alt from rangefinder in meters. This reading is corrected for vehicle tilt
    bool get_rangefinder_alt(float &alt_m) const;

    // Check if Obstacle defined by body-frame yaw and pitch is near ground
    bool check_obstacle_near_ground(float yaw, float pitch, float distance) const;
    bool check_obstacle_near_ground(float yaw, float distance) const { return check_obstacle_near_ground(yaw, 0.0f, distance); };
    // Check if Obstacle defined by Vector3f is near ground. The vector is assumed to be body frame FRD
    bool check_obstacle_near_ground(const Vector3f &obstacle) const;

    // database helpers. All angles are in degrees
    static bool database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned);
    // Note: "angle" refers to yaw (in body frame) towards the obstacle
    static void database_push(float angle, float distance);
    static void database_push(float angle, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned) {
        database_push(angle, 0.0f, distance, timestamp_ms, current_pos, body_to_ned);
    };
    static void database_push(float angle, float pitch, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned);

    uint32_t _last_timeout_check_ms;  // time when boundary was checked for non-updated valid faces

    // used for ground detection
    uint32_t _last_downward_update_ms;
    bool     _rangefinder_use;
    bool     _rangefinder_healthy;
    float    _rangefinder_alt;

    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;   // reference to this instances state

    // Methods to manipulate 3D boundary in this class
    AP_Proximity_Boundary_3D boundary;
};

#endif // HAL_PROXIMITY_ENABLED
