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
#include "ToneAlarm_ChibiOS.h"
#include "ToneAlarm_PX4.h"
#include "ToshibaLED.h"
#include "ToshibaLED_I2C.h"
#include "VRBoard_LED.h"
#include "DiscreteRGBLed.h"
#include "DiscoLED.h"
#include "Led_Sysfs.h"
#include "UAVCAN_RGB_LED.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Notify *AP_Notify::_instance;

#define CONFIG_NOTIFY_DEVICES_MAX 6

#define TOSHIBA_LED_I2C_BUS_INTERNAL    0
#define TOSHIBA_LED_I2C_BUS_EXTERNAL    1

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

    // @Param: OREO_THEME
    // @DisplayName: OreoLED Theme
    // @Description: Enable/Disable Solo Oreo LED driver, 0 to disable, 1 for Aircraft theme, 2 for Rover theme
    // @Values: 0:Disabled,1:Aircraft,2:Rover
    // @User: Advanced
    AP_GROUPINFO("OREO_THEME", 4, AP_Notify, _oreo_theme, 0),

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

#define ADD_BACKEND(backend) do { _devices[_num_devices++] = backend; if (_num_devices >= CONFIG_NOTIFY_DEVICES_MAX) return;} while(0)

// add notify backends to _devices array
void AP_Notify::add_backends(void)
{
    if (_num_devices != 0) {
        return;
    }

// Notify devices for PX4 boards
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_V3 // Has enough memory for Oreo LEDs
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new ToneAlarm_PX4());
    ADD_BACKEND(new Display());

    // Oreo LED enable/disable by NTF_OREO_THEME parameter
    if (_oreo_theme) {
        ADD_BACKEND(new OreoLED_PX4(_oreo_theme));
    }

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_V4 // Has its own LED board
    ADD_BACKEND(new PixRacerLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new ToneAlarm_PX4());
    ADD_BACKEND(new Display());

  #else   // All other px4 boards use standard devices.
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new ToneAlarm_PX4());
    ADD_BACKEND(new Display());
  #endif

// Notify devices for ChibiOS boards
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    ADD_BACKEND(new ToneAlarm_ChibiOS());
    ADD_BACKEND(new PixRacerLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new Display());

// Notify devices for VRBRAIN boards
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN  
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_VRBRAIN_V45 // Uses px4 LED board
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new ToneAlarm_PX4());
    ADD_BACKEND(new ExternalLED());
  #else
    ADD_BACKEND(new VRBoard_LED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new ToneAlarm_PX4());
    ADD_BACKEND(new ExternalLED());
  #endif

// Notify devices for linux boards    
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new NavioLED_I2C());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
    ADD_BACKEND(new Led_Sysfs("rgb_led0", "rgb_led2", "rgb_led1"));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
    ADD_BACKEND(new RCOutputRGBLedInverted(12, 13, 14));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new UAVCAN_RGB_LED(0));

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new Buzzer());
    ADD_BACKEND(new Display());

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new Display());

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new Buzzer());
    ADD_BACKEND(new Display());

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
    ADD_BACKEND(new RCOutputRGBLedOff(15, 13, 14, 255));

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
    ADD_BACKEND(new AP_BoardLED());

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new RCOutputRGBLed(HAL_RCOUT_RGBLED_RED, HAL_RCOUT_RGBLED_GREEN, HAL_RCOUT_RGBLED_BLUE));

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
    ADD_BACKEND(new DiscoLED());
    ADD_BACKEND(new ToneAlarm_Linux());

  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));

  #else // other linux
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new ToneAlarm_Linux());
  #endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
# ifdef HAL_HAVE_PIXRACER_LED
    ADD_BACKEND(new PixRacerLED());
# else
    ADD_BACKEND(new AP_BoardLED());
# endif
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new Display());
#elif CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new Display());
    ADD_BACKEND(new Buzzer());
//    ADD_BACKEND(new AP_BoardLED2()); // needs AP_BoardLED2 in master
#else
    ADD_BACKEND(new AP_BoardLED());
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_EXTERNAL));
    ADD_BACKEND(new ToshibaLED_I2C(TOSHIBA_LED_I2C_BUS_INTERNAL));
    ADD_BACKEND(new Display());
#endif
}

// initialisation
void AP_Notify::init(bool enable_external_leds)
{
    // add all the backends
    add_backends();

    // clear all flags and events
    memset(&AP_Notify::flags, 0, sizeof(AP_Notify::flags));
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));

    // clear flight mode string and text buffer
    memset(_flight_mode_str, 0, sizeof(_flight_mode_str));
    memset(_send_text, 0, sizeof(_send_text));
    _send_text_updated_millis = 0;

    AP_Notify::flags.external_leds = enable_external_leds;

    for (uint8_t i = 0; i < _num_devices; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->pNotify = this;
            _devices[i]->init();
        }
    }
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
