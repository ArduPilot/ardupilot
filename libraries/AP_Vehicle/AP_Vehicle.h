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

/*
  this header holds a parameter structure for each vehicle type for
  parameters needed by multiple libraries
 */

#include <AP_Param/AP_Param.h>

class AP_Vehicle {

public:
    /*
      common parameters for fixed wing aircraft
     */
    struct FixedWing {
        AP_Int8 throttle_min;
        AP_Int8 throttle_max;	
        AP_Int8 throttle_slewrate;
        AP_Int8 throttle_cruise;
        AP_Int8 takeoff_throttle_max;
        AP_Int16 airspeed_min;
        AP_Int16 airspeed_max;
        AP_Int32 airspeed_cruise_cm;
        AP_Int32 min_gndspeed_cm;
        AP_Int8  crash_detection_enable;
        AP_Int16 roll_limit_cd;
        AP_Int16 pitch_limit_max_cd;
        AP_Int16 pitch_limit_min_cd;        
        AP_Int8  autotune_level;
        AP_Int8  stall_prevention;
        AP_Int16 loiter_radius;

        struct Rangefinder_State {
            bool in_range:1;
            bool have_initial_reading:1;
            bool in_use:1;
            float initial_range;
            float correction;
            float initial_correction;
            float last_stable_correction;
            uint32_t last_correction_time_ms;
            uint8_t in_range_count;
            float height_estimate;
            float last_distance;
        };


        // stages of flight
        enum FlightStage {
            FLIGHT_TAKEOFF       = 1,
            FLIGHT_VTOL          = 2,
            FLIGHT_NORMAL        = 3,
            FLIGHT_LAND          = 4,
            FLIGHT_ABORT_LAND    = 7
        };
    };

    /*
      common parameters for multicopters
     */
    struct MultiCopter {
        AP_Int16 angle_max;
    };
};


#include "AP_Vehicle_Type.h"
