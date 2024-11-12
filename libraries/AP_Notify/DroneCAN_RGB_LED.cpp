/*
 * Copyright (C) 2017 Emlid Ltd. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include "DroneCAN_RGB_LED.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

#include <AP_CANManager/AP_CANManager.h>

#define LED_OFF 0
#define LED_FULL_BRIGHT 255
#define LED_MEDIUM ((LED_FULL_BRIGHT / 5) * 4)
#define LED_DIM ((LED_FULL_BRIGHT / 5) * 2)

DroneCAN_RGB_LED::DroneCAN_RGB_LED()
    : DroneCAN_RGB_LED(LED_OFF,
                     LED_FULL_BRIGHT, LED_MEDIUM, LED_DIM)
{
}

DroneCAN_RGB_LED::DroneCAN_RGB_LED(uint8_t led_off,
                               uint8_t led_full, uint8_t led_medium,
                               uint8_t led_dim)
    : RGBLed(led_off, led_full, led_medium, led_dim)
{
}

bool DroneCAN_RGB_LED::init()
{
    // LEDs can turn up later
    return true;
}


bool DroneCAN_RGB_LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    uavcan_equipment_indication_LightsCommand msg {};
    msg.commands.len = 1;
    msg.commands.data[0].light_id =0;
    msg.commands.data[0].color.red = red >> 3;
    msg.commands.data[0].color.green = green >> 2;
    msg.commands.data[0].color.blue = blue >> 3;

    // broadcast the message on all ifaces
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    bool ok = false;
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto *dronecan = AP_DroneCAN::get_dronecan(i);
        if (dronecan != nullptr) {
            ok |= dronecan->rgb_led.broadcast(msg);
        }
    }
    return ok;
}

#endif // HAL_ENABLE_DRONECAN_DRIVERS

