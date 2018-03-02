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

#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_WITH_UAVCAN
#include "UAVCAN_RGB_LED.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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

bool UAVCAN_RGB_LED::hw_init()
{
    return true;
}


bool UAVCAN_RGB_LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    bool success = false;
    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    success = success || uavcan->led_write(_led_index, red, green, blue);
                }
            }
        }
    }
    return success;
}
#endif
