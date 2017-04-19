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


#ifndef AP_NOTIFY_OREOLED
#define AP_NOTIFY_OREOLED 0
#endif

#ifndef AP_NOTIFY_SOLO_TONES
#define AP_NOTIFY_SOLO_TONES 0
#endif

// Device parameters values
#define RGB_LED_OFF     0
#define RGB_LED_LOW     1
#define RGB_LED_MEDIUM  2
#define RGB_LED_HIGH    3
#define BUZZER_ON       1
#define BUZZER_OFF      0

#define NOTIFY_TEXT_BUFFER_SIZE 51

//Type of on-board display
#define DISPLAY_OFF     0
#define DISPLAY_SSD1306 1
#define DISPLAY_SH1106  2

class AP_Notify
{
    friend class RGBLed;            // RGBLed needs access to notify parameters
    friend class Display;           // Display needs access to notify parameters
public:
    // Constructor
    AP_Notify();

    /// notify_flags_type - bitmask of notification flags
    struct notify_flags_and_values_type {
        uint32_t initialising       : 1;    // 1 if initialising and copter should not be moved
        uint32_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint32_t gps_num_sats       : 6;    // number of sats
        uint32_t flight_mode        : 8;    // flight mode
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
        uint32_t leak_detected      : 1;    // 1 if leak detected
        float    battery_voltage       ;    // battery voltage

        // additional flags
        uint32_t external_leds      : 1;    // 1 if external LEDs are enabled (normally only used for copter)
        uint32_t vehicle_lost       : 1;    // 1 when lost copter tone is requested (normally only used for copter)
        uint32_t waiting_for_throw  : 1;    // 1 when copter is in THROW mode and waiting to detect the user hand launch
        uint32_t powering_off       : 1;    // 1 when the vehicle is powering off
    };

    /// notify_events_type - bitmask of active events.
    //      Notify library is responsible for setting back to zero after notification has been completed
    struct notify_events_type {
        uint32_t arming_failed          : 1;    // 1 if copter failed to arm after user input
        uint32_t user_mode_change       : 1;    // 1 if user has initiated a flight mode change
        uint32_t user_mode_change_failed: 1;    // 1 when user initiated flight mode change fails
        uint32_t failsafe_mode_change   : 1;    // 1 when failsafe has triggered a flight mode change
        uint32_t autotune_complete      : 1;    // 1 when autotune has successfully completed
        uint32_t autotune_failed        : 1;    // 1 when autotune has failed
        uint32_t autotune_next_axis     : 1;    // 1 when autotune has completed one axis and is moving onto the next
        uint32_t mission_complete       : 1;    // 1 when the mission has completed successfully
        uint32_t waypoint_complete      : 1;    // 1 as vehicle completes a waypoint
        uint32_t initiated_compass_cal  : 1;    // 1 when user input to begin compass cal was accepted
        uint32_t compass_cal_saved      : 1;    // 1 when compass calibration was just saved
        uint32_t compass_cal_failed     : 1;    // 1 when compass calibration has just failed
        uint32_t compass_cal_canceled   : 1;    // 1 when compass calibration was just canceled
        uint32_t tune_started           : 1;    // tuning a parameter has started
        uint32_t tune_next              : 3;    // tuning switched to next parameter
        uint32_t tune_save              : 1;    // tuning saved parameters
        uint32_t tune_error             : 1;    // tuning controller error
    };

    // The notify flags and values are static to allow direct class access
    // without declaring the object.
    static struct notify_flags_and_values_type flags;
    static struct notify_events_type events;

    // initialisation
    void init(bool enable_external_leds);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

    // handle a LED_CONTROL message
    static void handle_led_control(mavlink_message_t* msg);

    // handle a PLAY_TUNE message
    static void handle_play_tune(mavlink_message_t* msg);

    bool buzzer_enabled() const { return _buzzer_enable; }

    // set flight mode string
    void set_flight_mode_str(const char *str);
    const char* get_flight_mode_str() const { return _flight_mode_str; }

    // send text to display
    void send_text(const char *str);
    const char* get_text() const { return _send_text; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 _rgb_led_brightness;
    AP_Int8 _rgb_led_override;
    AP_Int8 _buzzer_enable;
    AP_Int8 _display_type;

    char _send_text[NOTIFY_TEXT_BUFFER_SIZE];
    uint32_t _send_text_updated_millis; // last time text changed
    char _flight_mode_str[5];

    static NotifyDevice* _devices[];
};
