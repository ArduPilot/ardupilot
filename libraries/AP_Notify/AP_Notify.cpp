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

#include "AP_Notify.h"

#include "AP_BoardLED.h"
#include "PixRacerLED.h"
#include "Buzzer.h"
#include "Display.h"
#include "ExternalLED.h"
#include "NavioLED_I2C.h"
#include "OreoLED_PX4.h"
#include "RCOutputRGBLed.h"
#include "ToneAlarm_Linux.h"
#include "ToneAlarm_PX4.h"
#include "ToneAlarm_PX4_Solo.h"
#include "ToshibaLED.h"
#include "ToshibaLED_I2C.h"
#include "VRBoard_LED.h"
#include "DiscreteRGBLed.h"
#include "DiscoLED.h"
#include <stdio.h>


// table of user settable parameters
const AP_Param::GroupInfo AP_Notify::var_info[] = {

    // @Param: LED_BRIGHT
    // @DisplayName: LED Brightness
    // @Description: Select the RGB LED brightness level. When USB is connected brightness will never be higher than low regardless of the setting.
    // @Values: 0:Off,1:Low,2:Medium,3:High
    // @User: Advanced
    AP_GROUPINFO("LED_BRIGHT", 0, AP_Notify, _rgb_led_brightness, RGB_LED_HIGH),

    // @Param: BUZZ_ENABLE
    // @DisplayName: Buzzer enable
    // @Description: Enable or disable the buzzer. Only for Linux and PX4 based boards.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("BUZZ_ENABLE", 1, AP_Notify, _buzzer_enable, BUZZER_ON),

    // @Param: LED_OVERRIDE
    // @DisplayName: Setup for MAVLink LED override
    // @Description: This sets up the board RGB LED for override by MAVLink. Normal notify LED control is disabled
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("LED_OVERRIDE", 2, AP_Notify, _rgb_led_override, 0),

    // @Param: DISPLAY_TYPE
    // @DisplayName: Type of on-board I2C display
    // @Description: This sets up the type of on-board I2C display. Disabled by default.
    // @Values: 0:Disable,1:ssd1306,2:sh1106
    // @User: Advanced
    AP_GROUPINFO("DISPLAY_TYPE", 3, AP_Notify, _display_type, 0),

    AP_GROUPEND
};

// Default constructor
AP_Notify::AP_Notify()
{
	AP_Param::setup_object_defaults(this, var_info);
}

// static flags, to allow for direct class update from device drivers
struct AP_Notify::notify_flags_and_values_type AP_Notify::flags;
struct AP_Notify::notify_events_type AP_Notify::events;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_V4
    PixRacerLED boardled;
#else
    AP_BoardLED boardled;
#endif
    ToshibaLED_I2C toshibaled;
    Display display;

#if AP_NOTIFY_SOLO_TONES == 1
    ToneAlarm_PX4_Solo tonealarm;
#else
    ToneAlarm_PX4 tonealarm;
#endif

#if AP_NOTIFY_OREOLED == 1
    OreoLED_PX4 oreoled;
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &tonealarm, &oreoled, &display};
#else
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &tonealarm, &display};
#endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ToneAlarm_PX4 tonealarm;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_VRBRAIN_V45
    AP_BoardLED boardled;
#else
    VRBoard_LED boardled;
#endif
    ToshibaLED_I2C toshibaled;
    ExternalLED externalled;
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &externalled, &tonealarm};

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
        NavioLED_I2C navioled;
        ToshibaLED_I2C toshibaled;
        NotifyDevice *AP_Notify::_devices[] = {&navioled, &toshibaled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
        DiscreteRGBLed navioled(4, 27, 6, false);
        ToshibaLED_I2C toshibaled;
        NotifyDevice *AP_Notify::_devices[] = {&navioled, &toshibaled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        Buzzer buzzer;
        Display display;
        NotifyDevice *AP_Notify::_devices[] = {&display, &buzzer};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
        AP_BoardLED boardled;
        NotifyDevice *AP_Notify::_devices[] = {&boardled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
        ToshibaLED_I2C toshibaled;
        ToneAlarm_Linux tonealarm;
        NotifyDevice *AP_Notify::_devices[] = {&toshibaled, &tonealarm};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
        RCOutputRGBLedOff led(15, 13, 14, 255);
        NotifyDevice *AP_Notify::_devices[] = { &led };
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
        AP_BoardLED boardled;
        NotifyDevice *AP_Notify::_devices[] = {&boardled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
        AP_BoardLED boardled;
        RCOutputRGBLed bhled(HAL_RCOUT_RGBLED_RED, HAL_RCOUT_RGBLED_GREEN, HAL_RCOUT_RGBLED_BLUE);
        NotifyDevice *AP_Notify::_devices[] = {&boardled, &bhled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
        DiscoLED discoled;
        ToneAlarm_Linux tonealarm;
        NotifyDevice *AP_Notify::_devices[] = {&discoled, &tonealarm};
    #else
        AP_BoardLED boardled;
        ToshibaLED_I2C toshibaled;
        ToneAlarm_Linux tonealarm;
        NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &tonealarm};
    #endif
#else
    AP_BoardLED boardled;
    ToshibaLED_I2C toshibaled;
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled};
#endif

#define CONFIG_NOTIFY_DEVICES_COUNT (ARRAY_SIZE(AP_Notify::_devices))

// initialisation
void AP_Notify::init(bool enable_external_leds)
{
    // clear all flags and events
    memset(&AP_Notify::flags, 0, sizeof(AP_Notify::flags));
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));

    // clear flight mode string and text buffer
    memset(_flight_mode_str, 0, sizeof(_flight_mode_str));
    memset(_send_text, 0, sizeof(_send_text));
    _send_text_updated_millis = 0;

    AP_Notify::flags.external_leds = enable_external_leds;

    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        _devices[i]->pNotify = this;
        _devices[i]->init();
    }
}

// main update function, called at 50Hz
void AP_Notify::update(void)
{
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        _devices[i]->update();
    }

    //reset the events
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));
}

// handle a LED_CONTROL message
void AP_Notify::handle_led_control(mavlink_message_t *msg)
{
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        _devices[i]->handle_led_control(msg);
    }
}

// handle a PLAY_TUNE message
void AP_Notify::handle_play_tune(mavlink_message_t *msg)
{
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        _devices[i]->handle_play_tune(msg);
    }
}

// set flight mode string
void AP_Notify::set_flight_mode_str(const char *str)
{
    strncpy(_flight_mode_str, str, 4);
    _flight_mode_str[sizeof(_flight_mode_str)-1] = 0;
}

void AP_Notify::send_text(const char *str)
{
    strncpy(_send_text, str, sizeof(_send_text));
    _send_text[sizeof(_send_text)-1] = 0;
    _send_text_updated_millis = AP_HAL::millis();
}
