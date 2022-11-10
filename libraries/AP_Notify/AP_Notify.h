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
#include "AP_Notify_config.h"

#include "NotifyDevice.h"

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
#define DISPLAY_SITL 10

class AP_Notify
{
    friend class RGBLed;            // RGBLed needs access to notify parameters
    friend class Display;           // Display needs access to notify parameters
public:
    AP_Notify();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Notify);

    // get singleton instance
    static AP_Notify *get_singleton(void) {
        return _singleton;
    }

    // Oreo LED Themes
    enum Oreo_LED_Theme {
        OreoLED_Disabled        = 0,    // Disabled the OLED driver entirely
        OreoLED_Aircraft        = 1,    // Standard aviation themed lighting
        OreoLED_Automobile      = 2,    // Automobile themed lighting (white front, red back)
    };

    enum Notify_LED_Type {
        Notify_LED_None                     = 0,        // not enabled
        Notify_LED_Board                    = (1 << 0), // Built in board LED's
#if AP_NOTIFY_TOSHIBALED_ENABLED
        Notify_LED_ToshibaLED_I2C_Internal  = (1 << 1), // Internal ToshibaLED_I2C
        Notify_LED_ToshibaLED_I2C_External  = (1 << 2), // External ToshibaLED_I2C
#endif
#if AP_NOTIFY_PCA9685_ENABLED
        Notify_LED_PCA9685LED_I2C_External  = (1 << 3), // External PCA9685_I2C
#endif
        Notify_LED_OreoLED                  = (1 << 4), // Oreo
        Notify_LED_UAVCAN                   = (1 << 5), // UAVCAN RGB LED
#if AP_NOTIFY_NCP5623_ENABLED
        Notify_LED_NCP5623_I2C_External     = (1 << 6), // External NCP5623
        Notify_LED_NCP5623_I2C_Internal     = (1 << 7), // Internal NCP5623
#endif
        Notify_LED_NeoPixel                 = (1 << 8), // NeoPixel 5050 AdaFruit 1655 SK6812  Worldsemi WS2812B
        Notify_LED_ProfiLED                 = (1 << 9), // ProfiLED
        Notify_LED_Scripting                = (1 << 10),// Colour accessor for scripting
        Notify_LED_DShot                    = (1 << 11),// Use dshot commands to set ESC LEDs
        Notify_LED_ProfiLED_SPI             = (1 << 12), // ProfiLED
        Notify_LED_MAX
    };

    enum Notify_Buzz_Type {
        Notify_Buzz_None                    = 0,
        Notify_Buzz_Builtin                 = (1 << 0), // Built in default Alarm Out
        Notify_Buzz_DShot                   = (1 << 1), // DShot Alarm
        Notify_Buzz_UAVCAN                  = (1 << 2), // UAVCAN Alarm
    };

    /// notify_flags_type - bitmask of notification flags
    struct notify_flags_and_values_type {
        bool initialising;        // true if initialising and the vehicle should not be moved
        uint8_t gps_status;       // see the GPS_0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint8_t gps_num_sats;     // number of sats
        uint8_t flight_mode;      // flight mode
        bool armed;               // 0 = disarmed, 1 = armed
        bool flying;              // 0 = not flying, 1 = flying/driving/diving/tracking
        bool pre_arm_check;       // true if passing pre arm checks
        bool pre_arm_gps_check;   // true if passing pre arm gps checks
        bool save_trim;           // true if gathering trim data
        bool esc_calibration;     // true if calibrating escs
        bool failsafe_radio;      // true if radio failsafe
        bool failsafe_battery;    // true if battery failsafe
        bool failsafe_gcs;        // true if GCS failsafe
        bool failsafe_ekf;        // true if ekf failsafe
        bool parachute_release;   // true if parachute is being released
        bool ekf_bad;             // true if ekf is reporting problems
        bool autopilot_mode;      // true if vehicle is in an autopilot flight mode (only used by OreoLEDs)
        bool firmware_update;     // true just before vehicle firmware is updated
        bool compass_cal_running; // true if a compass calibration is running
        bool leak_detected;       // true if leak detected
        bool gps_fusion;          // true if the GPS is in use by EKF, usable for flight
        bool gps_glitching;       // true f the GPS is believed to be glitching is affecting navigation accuracy
        bool have_pos_abs;        // true if absolute position is available
        bool vehicle_lost;        // true when lost copter tone is requested (normally only used for copter)
        bool waiting_for_throw;   // true when copter is in THROW mode and waiting to detect the user hand launch
        bool powering_off;        // true when the vehicle is powering off
        bool video_recording;     // true when the vehicle is recording video
        bool temp_cal_running;    // true if a temperature calibration is running
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
        uint32_t initiated_temp_cal     : 1;    // 1 when temperature calibration starts
        uint32_t temp_cal_saved         : 1;    // 1 when temperature calibration was just saved
        uint32_t temp_cal_failed        : 1;    // 1 when temperature calibration has just failed
    };

    // The notify flags and values are static to allow direct class access
    // without declaring the object.
    static struct notify_flags_and_values_type flags;
    static struct notify_events_type events;

    // initialisation
    void init(void);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

#if AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED
    // handle a LED_CONTROL message
    static void handle_led_control(const mavlink_message_t &msg);
#endif

    // handle RGB from Scripting or AP_Periph
    static void handle_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz = 0);

    // handle RGB from Scripting
    static void handle_rgb_id(uint8_t r, uint8_t g, uint8_t b, uint8_t id);

#if AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED
    // handle a PLAY_TUNE message
    static void handle_play_tune(const mavlink_message_t &msg);
#endif

    // play a tune string
    static void play_tune(const char *tune);

    bool buzzer_enabled() const { return _buzzer_type != 0; }

    uint8_t get_buzzer_types() const { return _buzzer_type; }

    // set flight mode string
    void set_flight_mode_str(const char *str);
    const char* get_flight_mode_str() const { return _flight_mode_str; }

    // send text to display
    void send_text(const char *str);
    const char* get_text() const { return _send_text; }
    uint32_t get_text_updated_millis() const {return _send_text_updated_millis; }

    static const struct AP_Param::GroupInfo var_info[];
    uint8_t get_buzz_pin() const  { return _buzzer_pin; }
    uint8_t get_buzz_level() const  { return _buzzer_level; }
    uint8_t get_buzz_volume() const  { return _buzzer_volume; }
    uint8_t get_led_len() const { return _led_len; }
    int8_t get_rgb_led_brightness_percent() const;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HAL_Semaphore sf_window_mutex;
#endif

private:

    static AP_Notify *_singleton;

    void add_backend_helper(NotifyDevice *backend);

    // add all backends
    void add_backends(void);

    // parameters
    AP_Int8 _rgb_led_brightness;
    AP_Int8 _rgb_led_override;
    AP_Int8 _buzzer_type;
    AP_Int8 _display_type;
    AP_Int8 _oreo_theme;
    AP_Int8 _buzzer_pin;
    AP_Int32 _led_type;
    AP_Int8 _buzzer_level;
    AP_Int8 _buzzer_volume;
    AP_Int8 _led_len;

    char _send_text[NOTIFY_TEXT_BUFFER_SIZE];
    uint32_t _send_text_updated_millis; // last time text changed
    char _flight_mode_str[5];

    static NotifyDevice* _devices[];
    static uint8_t _num_devices;
};

namespace AP {
    AP_Notify &notify();
};
