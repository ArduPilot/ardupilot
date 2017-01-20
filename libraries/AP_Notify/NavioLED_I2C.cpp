/*
  NavioLED I2C driver
*/
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
#include "NavioLED_I2C.h"

#include <AP_HAL/AP_HAL.h>

#define PCA9685_ADDRESS 0x40
#define PCA9685_PWM 0x6

extern const AP_HAL::HAL& hal;

bool NavioLED_I2C::hw_init()
{
    _dev = hal.i2c_mgr->get_device(1, PCA9685_ADDRESS);

    if (!_dev) {
        return false;
    }

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&NavioLED_I2C::_timer, void));

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool NavioLED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb.r = red;
    rgb.g = green;
    rgb.b = blue;
    _need_update = true;
    return true;
}

void NavioLED_I2C::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;
    
    uint16_t red_adjusted = rgb.r * 0x10;
    uint16_t green_adjusted = rgb.g * 0x10;
    uint16_t blue_adjusted = rgb.b * 0x10;

    uint8_t blue_channel_lsb = blue_adjusted & 0xFF;
    uint8_t blue_channel_msb = blue_adjusted >> 8;

    uint8_t green_channel_lsb = green_adjusted & 0xFF;
    uint8_t green_channel_msb = green_adjusted >> 8;

    uint8_t red_channel_lsb = red_adjusted & 0xFF;
    uint8_t red_channel_msb = red_adjusted >> 8;


    uint8_t transaction[] = {PCA9685_PWM, 0x00, 0x00, blue_channel_lsb, blue_channel_msb,
			     0x00, 0x00, green_channel_lsb, green_channel_msb,
			     0x00, 0x00, red_channel_lsb, red_channel_msb};

    _dev->transfer(transaction, sizeof(transaction), nullptr, 0);
}
