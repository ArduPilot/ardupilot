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
#include "PCA9685LED_I2C.h"
#include "NCP5623.h"
#include "OreoLED_I2C.h"
#include "RCOutputRGBLed.h"
#include "ToneAlarm.h"
#include "ToshibaLED_I2C.h"
#include "VRBoard_LED.h"
#include "DiscreteRGBLed.h"
#include "DiscoLED.h"
#include "Led_Sysfs.h"
#include "UAVCAN_RGB_LED.h"
#include <stdio.h>
#include "AP_BoardLED2.h"

extern const AP_HAL::HAL& hal;

AP_Notify *AP_Notify::_instance;

#define CONFIG_NOTIFY_DEVICES_MAX 6

#define TOSHIBA_LED_I2C_BUS_INTERNAL    0
#define TOSHIBA_LED_I2C_BUS_EXTERNAL    1

// all I2C_LEDS
#define I2C_LEDS (Notify_LED_ToshibaLED_I2C_Internal | Notify_LED_ToshibaLED_I2C_External | \
                  Notify_LED_NCP5623_I2C_Internal | Notify_LED_NCP5623_I2C_External)

#ifndef BUILD_DEFAULT_LED_TYPE
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
  #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | I2C_LEDS)

// Linux boards    
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | I2C_LEDS |\
                                    Notify_LED_PCA9685LED_I2C_External)

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | I2C_LEDS)

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | I2C_LEDS |\
                                    Notify_LED_UAVCAN)
  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board)

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_ToshibaLED_I2C_External)

  #else // other linux
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | I2C_LEDS)
  #endif

// F4Light
#elif CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT
  #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | Notify_LED_ToshibaLED_I2C_External)

// All other builds
#else
    #define BUILD_DEFAULT_LED_TYPE (Notify_LED_Board | I2C_LEDS)

#endif // board selection

#endif // BUILD_DEFAULT_LED_TYPE

#ifndef BUZZER_ENABLE_DEFAULT
#define BUZZER_ENABLE_DEFAULT 1
#endif

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
    // @Description: Enable or disable the buzzer.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("BUZZ_ENABLE", 1, AP_Notify, _buzzer_enable, BUZZER_ENABLE_DEFAULT),

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

#if !HAL_MINIMIZE_FEATURES
    // @Param: OREO_THEME
    // @DisplayName: OreoLED Theme
    // @Description: Enable/Disable Solo Oreo LED driver, 0 to disable, 1 for Aircraft theme, 2 for Rover theme
    // @Values: 0:Disabled,1:Aircraft,2:Rover
    // @User: Advanced
    AP_GROUPINFO("OREO_THEME", 4, AP_Notify, _oreo_theme, 0),
#endif

#if !defined(HAL_BUZZER_PIN)
    // @Param: BUZZ_PIN
    // @DisplayName: Buzzer pin
    // @Description: Enables to connect active buzzer to arbitrary pin. Requires 3-pin buzzer or additional MOSFET!
    // @Values: 0:Disabled
    // @User: Advanced
    AP_GROUPINFO("BUZZ_PIN", 5, AP_Notify, _buzzer_pin, 0),
#endif

    // @Param: LED_TYPES
    // @DisplayName: LED Driver Types
    // @Description: Controls what types of LEDs will be enabled
    // @Bitmask: 0:Build in LED, 1:Internal ToshibaLED, 2:External ToshibaLED, 3:External PCA9685, 4:Oreo LED, 5:UAVCAN, 6:NCP5623 External, 7:NCP5623 Internal
    // @User: Advanced
    AP_GROUPINFO("LED_TYPES", 6, AP_Notify, _led_type, BUILD_DEFAULT_LED_TYPE),

    AP_GROUPEND
};

// Default constructor
AP_Notify::AP_Notify()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_instance != nullptr) {
        AP_HAL::panic("AP_Notify must be singleton");
    }
    _instance = this;
}

// static flags, to allow for direct class update from device drivers
struct AP_Notify::notify_flags_and_values_type AP_Notify::flags;
struct AP_Notify::notify_events_type AP_Notify::events;

NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_MAX];
uint8_t AP_Notify::_num_devices;

void AP_Notify::add_backend_helper(NotifyDevice *backend)
{
    _devices[_num_devices] = backend;
    _devices[_num_devices]->pNotify = this;
    if(!_devices[_num_devices]->init()) {
        delete _devices[_num_devices];
        _devices[_num_devices] = nullptr;
    } else {
      _num_devices++;
    }
}

#define ADD_BACKEND(backend) do { add_backend_helper(backend); if (_num_devices >= CONFIG_NOTIFY_DEVICES_MAX) return;} while(0)

// add notify backends to _devices array
void AP_Notify::add_backends(void)
{
    if (_num_devices != 0) {
        return;
    }

    for (uint32_t i = 1; i < Notify_LED_MAX; i = i << 1) {
        switch(_led_type & i) {
            case Notify_LED_None:
                break;
            case Notify_LED_Board:
                // select the most appropriate built in LED driver type
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
                ADD_BACKEND(new Led_Sysfs("rgb_led0", "rgb_led2", "rgb_led1"));
  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
                ADD_BACKEND(new RCOutputRGBLedInverted(12, 13, 14));
  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
                ADD_BACKEND(new RCOutputRGBLed(HAL_RCOUT_RGBLED_RED, HAL_RCOUT_RGBLED_GREEN, HAL_RCOUT_RGBLED_BLUE));
  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
                ADD_BACKEND(new DiscoLED());
  #endif
#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V52 || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_VRUBRAIN_V51
                ADD_BACKEND(new ExternalLED()); // despite the name this is a built in set of onboard LED's
#endif // CONFIG_HAL_BOARD_SUBTYPE == various CHIBIOS-VRBRAINs

#if defined(HAL_HAVE_PIXRACER_LED)
                ADD_BACKEND(new PixRacerLED());
#elif (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN) && defined(HAL_GPIO_C_LED_PIN))
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V52 || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_VRUBRAIN_V51
                ADD_BACKEND(new VRBoard_LED());
  #else
                ADD_BACKEND(new AP_BoardLED());
  #endif
#elif (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN))
                ADD_BACKEND(new AP_BoardLED2());
#endif
                break;
            case Notify_LED_ToshibaLED_I2C_Internal:
                ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
                break;
            case Notify_LED_ToshibaLED_I2C_External:
                ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
                break;
#if !HAL_MINIMIZE_FEATURES
            case Notify_LED_NCP5623_I2C_External:
                FOREACH_I2C_EXTERNAL(b) {
                    ADD_BACKEND(new NCP5623(b));
                }
                break;
            case Notify_LED_NCP5623_I2C_Internal:
                ADD_BACKEND(new NCP5623(TOSHIBA_LED_I2C_BUS_INTERNAL));
                break;
#endif
            case Notify_LED_PCA9685LED_I2C_External:
                ADD_BACKEND(new PCA9685LED_I2C());
                break;
            case Notify_LED_OreoLED:
#if !HAL_MINIMIZE_FEATURES
                if (_oreo_theme) {
                    ADD_BACKEND(new OreoLED_I2C(0, _oreo_theme));
                }
#endif
                break;
            case Notify_LED_UAVCAN:
#if HAL_WITH_UAVCAN
                ADD_BACKEND(new UAVCAN_RGB_LED(0));
#endif // HAL_WITH_UAVCAN
                break;

        }
    }


    // Always try and add a display backend
    ADD_BACKEND(new Display());

// ChibiOS noise makers
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#ifdef HAL_BUZZER_PIN
    ADD_BACKEND(new Buzzer());
#endif
#ifdef HAL_PWM_ALARM
    ADD_BACKEND(new AP_ToneAlarm());
#endif

// Linux noise makers
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    // No noise makers, keep this though to ensure that the final else is safe

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
        CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
    ADD_BACKEND(new Buzzer());

  #else // other linux
    ADD_BACKEND(new AP_ToneAlarm());
  #endif

// F4Light noise makers
#elif CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT
    ADD_BACKEND(new Buzzer());
#endif // Noise makers

}

// initialisation
void AP_Notify::init(void)
{
    // clear all flags and events
    memset(&AP_Notify::flags, 0, sizeof(AP_Notify::flags));
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));

    // add all the backends
    add_backends();
}

// main update function, called at 50Hz
void AP_Notify::update(void)
{
    for (uint8_t i = 0; i < _num_devices; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->update();
        }
    }

    //reset the events
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));
}

// handle a LED_CONTROL message
void AP_Notify::handle_led_control(mavlink_message_t *msg)
{
    for (uint8_t i = 0; i < _num_devices; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->handle_led_control(msg);
        }
    }
}

// handle a PLAY_TUNE message
void AP_Notify::handle_play_tune(mavlink_message_t *msg)
{
    for (uint8_t i = 0; i < _num_devices; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->handle_play_tune(msg);
        }
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
