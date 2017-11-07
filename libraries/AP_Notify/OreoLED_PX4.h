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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_HAL/AP_HAL.h>
#include "NotifyDevice.h"
#include <drivers/drv_oreoled.h>

#define OREOLED_NUM_LEDS        4       // maximum number of individual LEDs connected to the oreo led cpu
#define OREOLED_INSTANCE_ALL    0xff    // instance number to indicate all LEDs (used for set_rgb and set_macro)
#define OREOLED_BRIGHT          0xff    // maximum brightness when flying (disconnected from usb)

#define CUSTOM_HEADER_LENGTH    4       // number of bytes in the custom LED buffer that are used to identify the command

class OreoLED_PX4 : public NotifyDevice
{
public:
    // constuctor
    OreoLED_PX4(uint8_t theme);

    // init - initialised the device
    bool init(void);

    // update - updates device according to timed_updated.  Should be
    // called at 50Hz
    void update();

    // healthy - return true if at least one LED is responding
    bool healthy() const { return _overall_health; }

    // handle a LED_CONTROL message, by default device ignore message
    void handle_led_control(mavlink_message_t *msg);

private:
    // update_timer - called by scheduler and updates PX4 driver with commands
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
    void sync_counter(void);
    bool mode_firmware_update(void);
    bool mode_init(void);
    bool mode_failsafe_radio(void);
    bool set_standard_colors(void);
    bool mode_failsafe_batt(void);
    bool mode_auto_flight(void);
    bool mode_pilot_flight(void);

    // Clear the desired state
    void clear_state(void);

    // oreo led modes (pattern, macro or rgb)
    enum oreoled_mode {
        OREOLED_MODE_NONE,
        OREOLED_MODE_MACRO,
        OREOLED_MODE_RGB,
        OREOLED_MODE_RGB_EXTENDED,
        OREOLED_MODE_SYNC
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

        void send_sync();

        void set_macro(oreoled_macro new_macro);

        void set_rgb(enum oreoled_pattern new_pattern, uint8_t new_red, uint8_t new_green, uint8_t new_blue);

        void set_rgb(enum oreoled_pattern new_pattern, uint8_t new_red, uint8_t new_green,
                uint8_t new_blue, uint8_t new_amplitude_red, uint8_t new_amplitude_green, uint8_t new_amplitude_blue,
                uint16_t new_period, uint16_t new_phase_offset);

        bool operator==(const oreo_state &os);
    };

    // private members
    bool    _overall_health;                        // overall health
    int     _oreoled_fd;                            // file descriptor
    bool    _send_required;                         // true when we need to send an update to at least one led
    volatile bool _state_desired_semaphore;         // true when we are updating the state desired values to ensure they are not sent prematurely
    oreo_state _state_desired[OREOLED_NUM_LEDS];    // desired state
    oreo_state _state_sent[OREOLED_NUM_LEDS];       // last state sent to led
    uint8_t _pattern_override;                      // holds last processed pattern override, 0 if we are not overriding a pattern
    uint8_t _oreo_theme;                            // theme (1=AirCraft, 2=Ground Vehicle)
    uint8_t _rear_color_r = 255;                    // the rear LED red value
    uint8_t _rear_color_g = 255;                    // the rear LED green value
    uint8_t _rear_color_b = 255;                    // the rear LED blue value
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
