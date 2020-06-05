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

#include "ModeReason.h" // reasons can't be defined in this header due to circular loops

#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>     // board configuration library
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Button/AP_Button.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>                    // Notify library
#include <AP_Param/AP_Param.h>
#include <AP_Relay/AP_Relay.h>                      // APM relay
#include <AP_RSSI/AP_RSSI.h>                        // RSSI Library
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Hott_Telem/AP_Hott_Telem.h>

class AP_Vehicle : public AP_HAL::HAL::Callbacks {

public:

    AP_Vehicle() {
        if (_singleton) {
            AP_HAL::panic("Too many Vehicles");
        }
        _singleton = this;
    }

    /* Do not allow copies */
    AP_Vehicle(const AP_Vehicle &other) = delete;
    AP_Vehicle &operator=(const AP_Vehicle&) = delete;

    static AP_Vehicle *get_singleton();

    bool virtual set_mode(const uint8_t new_mode, const ModeReason reason) = 0;

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

protected:

    // board specific config
    AP_BoardConfig BoardConfig;

#if HAL_WITH_UAVCAN
    // board specific config for CAN bus
    AP_BoardConfig_CAN BoardConfig_CAN;
#endif

    // sensor drivers
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    AP_Button button;
    RangeFinder rangefinder;

    AP_RSSI rssi;

    AP_SerialManager serial_manager;

    AP_Relay relay;

    AP_ServoRelayEvents ServoRelayEvents;

    // notification object for LEDs, buzzers etc (parameter set to
    // false disables external leds)
    AP_Notify notify;

#if HAL_HOTT_TELEM_ENABLED
    AP_Hott_Telem hott_telem;
#endif

    // called from each vehicle
    void vehicle_setup(void);

private:

    static AP_Vehicle *_singleton;

};

namespace AP {
    AP_Vehicle *vehicle();
};

extern const AP_HAL::HAL& hal;

#include "AP_Vehicle_Type.h"
