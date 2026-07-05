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
 *  AP_Formation - GPS+UWB Sensor Fusion for Formation Flight Control
 *
 *  FlightDock autonomous aerial docking research project
 *  Implements complementary filter for GPS and UWB ranging
 *  Runs at 400Hz in ArduPilot fast loop
 *
 *  Target application: QuadPlane following DHC-2 Beaver aircraft
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

class AP_Formation {
public:
    // Constructor
    AP_Formation();

    // Do not allow copying
    CLASS_NO_COPY(AP_Formation);

    // Main control loop - called at 400Hz (dt = 0.0025s)
    void update(float dt);

    // Input setters - receive data from external sources
    void set_target_position(const Vector3f &pos_ned);
    void set_target_velocity(const Vector3f &vel_ned);
    void set_uwb_range(float range_m);
    void set_own_position(const Vector3f &pos_ned);

    // Output getter - commanded velocity for flight controller
    Vector3f get_velocity_command() const { return _vel_cmd; }

    // Control status
    bool is_active() const { return _active; }
    void set_active(bool active) { _active = active; }

    // Reset controller state
    void reset();

    // Parameter table declaration
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Parameters - tunable via GCS (FORM_* prefix)
    AP_Float _formation_offset;     // Trail distance behind leader (m)
    AP_Float _formation_lateral;    // Lateral offset from leader (m)
    AP_Float _formation_vertical;   // Vertical offset from leader (m)
    AP_Float _predict_time;         // Prediction lookahead time (s)
    AP_Float _smooth_alpha;         // Smoothing filter weight (0-1)
    AP_Float _approach_speed;       // Speed when far from formation (m/s)
    AP_Float _maintain_speed;       // Speed when in formation (m/s)
    AP_Float _speed_threshold;      // Distance threshold for speed switch (m)

    // State variables - updated each cycle
    Vector3f _target_pos;           // Lead aircraft position NED (m)
    Vector3f _target_vel;           // Lead aircraft velocity NED (m/s)
    Vector3f _own_pos;              // Own position NED (m)
    float _uwb_range;               // UWB range measurement (m)
    float _smooth_range;            // Filtered range for control (m)
    Vector3f _vel_cmd;              // Current velocity command output (m/s)
    uint32_t _last_update_ms;       // Timestamp of last update
    bool _active;                   // Formation control active flag

    // Previous state for filtering
    bool _smooth_range_initialized; // Track first iteration
    Vector3f _predicted_target_pos; // Cached predicted position

    // Helper methods - internal calculations

    // Calculate adaptive GPS/UWB weighting based on range
    // Returns alpha (0-1): 0=GPS only, 1=UWB only
    float calculate_adaptive_alpha(float range_m) const;

    // Calculate desired position in formation
    // Applies offset behind/beside leader with prediction
    Vector3f calculate_formation_position() const;

    // Calculate GPS-derived range to target
    float calculate_gps_range() const;

    // Apply exponential smoothing filter to range
    void update_smooth_range(float raw_range);

    // Calculate commanded velocity toward formation position
    Vector3f calculate_velocity_command(float dt) const;

    // Binary speed control based on range
    float calculate_target_speed(float range_m) const;
};
