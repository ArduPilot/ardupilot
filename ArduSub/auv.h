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

#include <AP_Param/AP_Param.h>

class AUV_Control {
public:
    AUV_Control();

    enum TailType {
        TAIL_CRUCIFORM = 0, // Conventional cruciform (+): 2 horizontal elevators, 2 vertical rudders
        TAIL_X         = 1  // X-Tail (x): 4 fins at 45 degrees
    };

    // Initialize default channel mappings and angle ranges
    void init();

    // Compute and dispatch control surface demands to SRV_Channels
    void output_control_surfaces();

    // Compute demanded pitch angle (in centidegrees) from depth tracking error
    bool calculate_depth_pitch_cd(float target_z_cm, float curr_z_cm, float &pitch_cd);

    // Cruise throttle normalized [0.0, 1.0]
    float get_cruise_throttle() const { return constrain_float(_cruise_throttle.get() * 0.01f, 0.0f, 1.0f); }

    // Minimum control speed in m/s
    float get_speed_min() const { return _speed_min.get(); }

    // Check if AUV control allocation is active
    bool is_enabled() const;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8     _enable;
    AP_Int8     _tail_type;
    AP_Float    _fin_max_deg;
    AP_Float    _speed_min;
    AP_Float    _speed_nom;
    AP_Float    _speed_scale_min;
    AP_Float    _speed_scale_max;
    AP_Float    _pitch_max_deg;
    AP_Float    _pitch_gain;
    AP_Float    _cruise_throttle;
    AP_Int8     _roll_diff;

    // Helper to calculate dynamic pressure speed scaler
    float get_speed_scaler() const;
};
