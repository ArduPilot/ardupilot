/*
  ToshibaLED I2C driver
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

/* LED driver for TCA62724FMG */

#include "ToshibaLED_I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define TOSHIBA_LED_I2C_ADDR 0x55    // default I2C bus address
#define TOSHIBA_LED_I2C_BUS  1

#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register

bool ToshibaLED_I2C::hw_init()
{
    _dev = std::move(hal.i2c_mgr->get_device(TOSHIBA_LED_I2C_BUS, TOSHIBA_LED_I2C_ADDR));

    // take i2c bus sempahore
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // enable the led
    bool ret = _dev->write_register(TOSHIBA_LED_ENABLE, 0x03);

    // update the red, green and blue values to zero
    uint8_t val[4] = { TOSHIBA_LED_PWM0, _led_off, _led_off, _led_off };
    ret &= _dev->transfer(val, sizeof(val), nullptr, 0);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool ToshibaLED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!_dev || !_dev->get_semaphore()->take(5)) {
        return false;
    }

    /* 4-bit for each color */
    uint8_t val[4] = { TOSHIBA_LED_PWM0, (uint8_t)(blue >> 4),
                       (uint8_t)(green / 16), (uint8_t)(red / 16) };
    bool ret = _dev->transfer(val, sizeof(val), nullptr, 0);

    _dev->get_semaphore()->give();

    return ret;
}
