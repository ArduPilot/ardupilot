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
#ifndef AP_VEHICLE_H
#define AP_VEHICLE_H
/*
  this header holds a parameter structure for each vehicle type for
  parameters needed by multiple libraries
 */

#include <AP_Param.h>

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
        AP_Int16 airspeed_min;
        AP_Int16 airspeed_max;
        AP_Int16 pitch_limit_max_cd;
        AP_Int16 pitch_limit_min_cd;        
        AP_Int8  autotune_level;
        AP_Int16 land_pitch_cd;
        AP_Int8  stall_prevention;
    };

    /*
      common parameters for multicopters
     */
    struct MultiCopter {
        AP_Int16 angle_max;
    };
};

/*
  define common vehicle build types. Note that the APM_BUILD_DIRECTORY
  define is only available with makefile based build, not with
  arduino.
  Also note that code needs to support other APM_BUILD_DIRECTORY
  values for example sketches
 */
#define APM_BUILD_APMrover2      1
#define APM_BUILD_ArduCopter     2
#define APM_BUILD_ArduPlane      3
#define APM_BUILD_AntennaTracker 4
#define APM_BUILD_UNKNOWN        5

/*
  using this macro catches cases where we try to check vehicle type on
  build systems that don't support it
 */
#ifdef APM_BUILD_DIRECTORY
#define APM_BUILD_TYPE(type) ((type) == APM_BUILD_DIRECTORY)
#else
#define APM_BUILD_TYPE(type) ((type) == APM_BUILD_UNKNOWN)
#endif

#endif // AP_VEHICLE_H
