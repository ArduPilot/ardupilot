/*
   OreoLED I2C driver

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
#include "NotifyDevice.h"

#ifndef HAL_OREO_LED_ENABLED
#define HAL_OREO_LED_ENABLED 0
#endif

#define OREOLED_NUM_LEDS        4       // maximum number of individual LEDs connected to the oreo led cpu
#define OREOLED_INSTANCE_ALL    0xff    // instance number to indicate all LEDs (used for set_rgb and set_macro)
#define OREOLED_BRIGHT          0xff    // maximum brightness when flying (disconnected from usb)

#define CUSTOM_HEADER_LENGTH    4       // number of bytes in the custom LED buffer that are used to identify the command

class OreoLED_I2C : public NotifyDevice {
public:
    // constuctor
    OreoLED_I2C(uint8_t bus, uint8_t theme);

    // init - initialised the device
    bool init(void) override;

    // update - updates device according to timed_updated.  Should be
    // called at 50Hz
    void update() override;

#if AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED
    // handle a LED_CONTROL message, by default device ignore message
    void handle_led_control(const mavlink_message_t &msg) override;
#endif

private:
    enum oreoled_pattern {
        OREOLED_PATTERN_OFF = 0,
        OREOLED_PATTERN_SINE = 1,
        OREOLED_PATTERN_SOLID = 2,
        OREOLED_PATTERN_SIREN = 3,
        OREOLED_PATTERN_STROBE = 4,
        OREOLED_PATTERN_FADEIN = 5,
        OREOLED_PATTERN_FADEOUT = 6,
        OREOLED_PATTERN_PARAMUPDATE = 7,
        OREOLED_PATTERN_ENUM_COUNT
    };

    /* enum of available macros	defined by hardware */
    enum oreoled_macro {
        OREOLED_PARAM_MACRO_RESET = 0,
        OREOLED_PARAM_MACRO_COLOUR_CYCLE = 1,
        OREOLED_PARAM_MACRO_BREATH = 2,
        OREOLED_PARAM_MACRO_STROBE = 3,
        OREOLED_PARAM_MACRO_FADEIN = 4,
        OREOLED_PARAM_MACRO_FADEOUT = 5,
        OREOLED_PARAM_MACRO_RED = 6,
        OREOLED_PARAM_MACRO_GREEN = 7,
        OREOLED_PARAM_MACRO_BLUE = 8,
        OREOLED_PARAM_MACRO_YELLOW = 9,
        OREOLED_PARAM_MACRO_WHITE = 10,
        OREOLED_PARAM_MACRO_AUTOMOBILE = 11,
        OREOLED_PARAM_MACRO_AVIATION = 12,
        OREOLED_PARAM_MACRO_ENUM_COUNT
    };

    /* enum passed to OREOLED_SET_MODE defined by hardware */
    enum oreoled_param {
        OREOLED_PARAM_BIAS_RED = 0,
        OREOLED_PARAM_BIAS_GREEN = 1,
        OREOLED_PARAM_BIAS_BLUE = 2,
        OREOLED_PARAM_AMPLITUDE_RED = 3,
        OREOLED_PARAM_AMPLITUDE_GREEN = 4,
        OREOLED_PARAM_AMPLITUDE_BLUE = 5,
        OREOLED_PARAM_PERIOD = 6,
        OREOLED_PARAM_REPEAT = 7,
        OREOLED_PARAM_PHASEOFFSET = 8,
        OREOLED_PARAM_MACRO = 9,
        OREOLED_PARAM_RESET = 10,
        OREOLED_PARAM_APP_CHECKSUM = 11,
        OREOLED_PARAM_ENUM_COUNT
    };

    // update_timer - called by scheduler and updates driver with commands
    void update_timer(void);

    // set_rgb - set color as a combination of red, green and blue values for one or all LEDs, pattern defaults to solid color
    void set_rgb(uint8_t instance, uint8_t red, uint8_t green, uint8_t blue);

    // set_rgb - set color as a combination of red, green and blue values for one or all LEDs, using the specified pattern
    void set_rgb(uint8_t instance, enum oreoled_pattern pattern, uint8_t red, uint8_t green, uint8_t blue);

    // set_rgb - set color as a combination of red, green and blue values for one or all LEDs, using the specified pattern and other parameters
    void set_rgb(uint8_t instance, oreoled_pattern pattern, uint8_t red, uint8_t green, uint8_t blue,
                 uint8_t amplitude_red, uint8_t amplitude_green, uint8_t amplitude_blue,
                 uint16_t period, uint16_t phase_offset);

    // set_macro - set macro for one or all LEDs
    void set_macro(uint8_t instance, enum oreoled_macro macro);

    // send_sync - force a syncronisation of the all LED's
    void send_sync();

    // functions to set LEDs to specific patterns.  These functions return true if no further updates should be made to LEDs this iteration
    bool slow_counter(void);
    bool mode_firmware_update(void);
    bool mode_init(void);
    bool mode_failsafe_radio(void);
    bool mode_failsafe_gcs(void);
    bool set_standard_colors(void);
    bool mode_failsafe_batt(void);
    bool mode_auto_flight(void);
    bool mode_pilot_flight(void);

    // Clear the desired state
    void clear_state(void);

    // oreo led modes (pattern, macro or rgb)
    enum oreoled_mode {
        OREOLED_MODE_NONE=0,
        OREOLED_MODE_MACRO,
        OREOLED_MODE_RGB,
        OREOLED_MODE_RGB_EXTENDED,
    };

    // Oreo LED modes
    enum Oreo_LED_Theme {
        OreoLED_Disabled        = 0,
        OreoLED_Aircraft        = 1,
        OreoLED_Automobile      = 2,
    };

    // oreo_state structure holds possible state of an led
    struct oreo_state {
        enum oreoled_mode mode;
        enum oreoled_pattern pattern;
        enum oreoled_macro macro;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint8_t amplitude_red;
        uint8_t amplitude_green;
        uint8_t amplitude_blue;
        uint16_t period;
        int8_t repeat;
        uint16_t phase_offset;

        oreo_state();

        void clear_state();

        void set_macro(oreoled_macro new_macro);

        void set_rgb(enum oreoled_pattern new_pattern, uint8_t new_red, uint8_t new_green, uint8_t new_blue);

        void set_rgb(enum oreoled_pattern new_pattern, uint8_t new_red, uint8_t new_green,
                     uint8_t new_blue, uint8_t new_amplitude_red, uint8_t new_amplitude_green, uint8_t new_amplitude_blue,
                     uint16_t new_period, uint16_t new_phase_offset);

        bool operator==(const oreo_state &os) const;
    };

    typedef struct {
        uint8_t led_num;
        uint8_t num_bytes;
        uint8_t buff[32];
    } oreoled_cmd_t;


    // send a I2C command
    bool command_send(oreoled_cmd_t &cmd);
    void boot_leds(void);

    // private members
    uint8_t _bus;
    HAL_Semaphore _sem;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool    _send_required;                         // true when we need to send an update to at least one led
    oreo_state _state_desired[OREOLED_NUM_LEDS];    // desired state
    oreo_state _state_sent[OREOLED_NUM_LEDS];       // last state sent to led
    uint8_t _pattern_override;                      // holds last processed pattern override, 0 if we are not overriding a pattern
    uint8_t _oreo_theme;                            // theme (1=AirCraft, 2=Ground Vehicle)
    uint8_t _rear_color_r = 255;                    // the rear LED red value
    uint8_t _rear_color_g = 255;                    // the rear LED green value
    uint8_t _rear_color_b = 255;                    // the rear LED blue value

    uint8_t _slow_count;
    uint8_t _boot_count;
    uint32_t _last_boot_ms;
    uint32_t _last_sync_ms;
};
