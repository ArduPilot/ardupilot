/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "NotifyDevice.h"

#ifndef OREOLED_ENABLED
 # define OREOLED_ENABLED   0   // set to 1 to enable OreoLEDs
#endif

// Device parameters values
#define RGB_LED_OFF     0
#define RGB_LED_LOW     1
#define RGB_LED_MEDIUM  2
#define RGB_LED_HIGH    3

class AP_Notify
{
    friend class RGBLed;    // RGBLed needs access to notify parameters
public:
    // Constructor
    AP_Notify();

    /// notify_flags_type - bitmask of notification flags
    struct notify_flags_type {
        uint32_t initialising       : 1;    // 1 if initialising and copter should not be moved
        uint32_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint32_t gps_num_sats       : 6;    // number of sats
        uint32_t armed              : 1;    // 0 = disarmed, 1 = armed
        uint32_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
        uint32_t pre_arm_gps_check  : 1;    // 0 = failing pre-arm GPS checks, 1 = passed
        uint32_t save_trim          : 1;    // 1 if gathering trim data
        uint32_t esc_calibration    : 1;    // 1 if calibrating escs
        uint32_t failsafe_radio     : 1;    // 1 if radio failsafe
        uint32_t failsafe_battery   : 1;    // 1 if battery failsafe
        uint32_t parachute_release  : 1;    // 1 if parachute is being released
        uint32_t ekf_bad            : 1;    // 1 if ekf is reporting problems
        uint32_t autopilot_mode     : 1;    // 1 if vehicle is in an autopilot flight mode (only used by OreoLEDs)
        uint32_t firmware_update    : 1;    // 1 just before vehicle firmware is updated
        uint32_t compass_cal_running: 1;    // 1 if a compass calibration is running

        // additional flags
        uint32_t external_leds      : 1;    // 1 if external LEDs are enabled (normally only used for copter)
        uint32_t vehicle_lost       : 1;    // 1 when lost copter tone is requested (normally only used for copter)
    };

    /// notify_events_type - bitmask of active events.
    //      Notify library is responsible for setting back to zero after notification has been completed
    struct notify_events_type {
        uint16_t arming_failed          : 1;    // 1 if copter failed to arm after user input
        uint16_t user_mode_change       : 1;    // 1 if user has initiated a flight mode change
        uint16_t user_mode_change_failed: 1;    // 1 when user initiated flight mode change fails
        uint16_t failsafe_mode_change   : 1;    // 1 when failsafe has triggered a flight mode change
        uint16_t autotune_complete      : 1;    // 1 when autotune has successfully completed
        uint16_t autotune_failed        : 1;    // 1 when autotune has failed
        uint16_t autotune_next_axis     : 1;    // 1 when autotune has completed one axis and is moving onto the next
        uint16_t mission_complete       : 1;    // 1 when the mission has completed successfully
        uint16_t waypoint_complete      : 1;    // 1 as vehicle completes a waypoint
        uint16_t initiated_compass_cal  : 1;    // 1 when user input to begin compass cal was accepted
        uint16_t compass_cal_saved      : 1;    // 1 when compass calibration was just saved
        uint16_t compass_cal_failed     : 1;    // 1 when compass calibration has just failed
        uint16_t compass_cal_canceled   : 1;    // 1 when compass calibration was just canceled
    };

    // the notify flags are static to allow direct class access
    // without declaring the object
    static struct notify_flags_type flags;
    static struct notify_events_type events;

    // initialisation
    void init(bool enable_external_leds);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

    // handle a LED_CONTROL message
    static void handle_led_control(mavlink_message_t* msg);

    static const struct AP_Param::GroupInfo var_info[];

private:
    static NotifyDevice* _devices[];

    AP_Int8 _rgb_led_brightness;
};
