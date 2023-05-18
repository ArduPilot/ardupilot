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

#if HAL_CANMANAGER_ENABLED
#include "UAVCAN_RGB_LED.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#include <AP_CANManager/AP_CANManager.h>

#define LED_OFF 0
#define LED_FULL_BRIGHT 255
#define LED_MEDIUM ((LED_FULL_BRIGHT / 5) * 4)
#define LED_DIM ((LED_FULL_BRIGHT / 5) * 2)

UAVCAN_RGB_LED::UAVCAN_RGB_LED(uint8_t led_index)
    : UAVCAN_RGB_LED(led_index, LED_OFF,
                     LED_FULL_BRIGHT, LED_MEDIUM, LED_DIM)
{
}

UAVCAN_RGB_LED::UAVCAN_RGB_LED(uint8_t led_index, uint8_t led_off,
                               uint8_t led_full, uint8_t led_medium,
                               uint8_t led_dim)
    : RGBLed(led_off, led_full, led_medium, led_dim)
    , _led_index(led_index)
{
}

bool UAVCAN_RGB_LED::init()
{
    const uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_UAVCAN *uavcan = AP_UAVCAN::get_uavcan(i);
        if (uavcan != nullptr) {
            return true;
        }
    }
    // no UAVCAN drivers
    return false;
}


bool UAVCAN_RGB_LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    bool success = false;
    uint8_t can_num_drivers = AP::can().get_num_drivers();

    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_UAVCAN *uavcan = AP_UAVCAN::get_uavcan(i);
        if (uavcan != nullptr) {
            success = uavcan->led_write(_led_index, red, green, blue) || success;
        }
    }
    return success;
}
#endif
