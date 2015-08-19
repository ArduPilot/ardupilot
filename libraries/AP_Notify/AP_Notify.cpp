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

#include "AP_Notify.h"

// static flags, to allow for direct class update from device drivers
struct AP_Notify::notify_flags_type AP_Notify::flags;
struct AP_Notify::notify_events_type AP_Notify::events;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_BoardLED boardled;
    ToshibaLED_PX4 toshibaled;
    ToneAlarm_PX4 tonealarm;
#if OREOLED_ENABLED
    OreoLED_PX4 oreoled;
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &tonealarm, &oreoled};
#else
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &tonealarm};
#endif
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2 
    AP_BoardLED boardled;
    ExternalLED externalled;
    Buzzer buzzer;
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &externalled, &buzzer};
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    Buzzer buzzer;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_VRBRAIN_V45
    AP_BoardLED boardled;
#else
    VRBoard_LED boardled;
#endif
    ToshibaLED_I2C toshibaled;
    ExternalLED externalled;
    NotifyDevice *AP_Notify::_devices[] = {&boardled, &toshibaled, &externalled, &buzzer};
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
        AP_BoardLED boardled;
        NavioLED_I2C navioled;
        ToshibaLED_I2C toshibaled;
        NotifyDevice *AP_Notify::_devices[] = {&boardled, &navioled, &toshibaled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        ToshibaLED_I2C toshibaled;
        NotifyDevice *AP_Notify::_devices[] = {&toshibaled};
    #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
        ToshibaLED_I2C toshibaled;
        ToneAlarm_Linux tonealarm;
        NotifyDevice *AP_Notify::_devices[] = {&toshibaled, &tonealarm};
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

    AP_Notify::flags.external_leds = enable_external_leds;

    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
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
