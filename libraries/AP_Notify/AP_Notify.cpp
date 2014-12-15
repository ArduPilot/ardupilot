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

#include <AP_Notify.h>

// static flags, to allow for direct class update from device drivers
struct AP_Notify::notify_flags_type AP_Notify::flags;
struct AP_Notify::notify_events_type AP_Notify::events;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_BoardLED boardled;
    ToshibaLED_PX4 toshibaled;
    ToneAlarm_PX4 tonealarm;
    NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_COUNT] = {&boardled, &toshibaled, &tonealarm};
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2 
    AP_BoardLED boardled;
    ExternalLED externalled;
    Buzzer buzzer;
    NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_COUNT] = {&boardled, &externalled, &buzzer};
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    Buzzer buzzer;
    AP_BoardLED boardled;
    ToshibaLED_I2C toshibaled;
    ExternalLED externalled;
    NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_COUNT] = {&boardled, &toshibaled, &externalled, &buzzer};
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
        AP_BoardLED boardled;
        NavioLED_I2C navioled;
        NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_COUNT] = {&boardled, &navioled};
    #else
        AP_BoardLED boardled;
        ToshibaLED_I2C toshibaled;
        ToneAlarm_Linux tonealarm;
        NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_COUNT] = {&boardled, &toshibaled, &tonealarm};
    #endif
#else
    AP_BoardLED boardled;
    ToshibaLED_I2C toshibaled;
    NotifyDevice *AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_COUNT] = {&boardled, &toshibaled};
#endif

// initialisation
void AP_Notify::init(bool enable_external_leds)
{
    AP_Notify::flags.external_leds = enable_external_leds;

    for (int i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        _devices[i]->init();
    }
}

// main update function, called at 50Hz
void AP_Notify::update(void)
{
    for (int i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        _devices[i]->update();
    }

    //reset the events
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));
}
