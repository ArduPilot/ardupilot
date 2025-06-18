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

#if AP_NOTIFY_TOSHIBALED_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define TOSHIBA_LED_BRIGHT  0xFF    // full brightness
#define TOSHIBA_LED_MEDIUM  0x80    // medium brightness
#define TOSHIBA_LED_DIM     0x11    // dim
#define TOSHIBA_LED_OFF     0x00    // off

#define TOSHIBA_LED_I2C_ADDR 0x55    // default I2C bus address

#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register

ToshibaLED_I2C::ToshibaLED_I2C(uint8_t bus)
    : RGBLed(TOSHIBA_LED_OFF, TOSHIBA_LED_BRIGHT, TOSHIBA_LED_MEDIUM, TOSHIBA_LED_DIM)
    , _bus(bus)
{
}

bool ToshibaLED_I2C::init(void)
{
    // first look for led on external bus
    _dev = hal.i2c_mgr->get_device_ptr(_bus, TOSHIBA_LED_I2C_ADDR);
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // enable the led
    bool ret = _dev->write_register(TOSHIBA_LED_ENABLE, 0x03);
    if (!ret) {
        return false;
    }

    // update the red, green and blue values to zero
    uint8_t val[4] = { TOSHIBA_LED_PWM0, _led_off, _led_off, _led_off };
    ret = _dev->transfer(val, sizeof(val), nullptr, 0);

    _dev->set_retries(1);

    if (ret) {
        _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&ToshibaLED_I2C::_timer, void));
    }

    return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool ToshibaLED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb = {red, green, blue};
    _need_update = true;
    return true;
}

void ToshibaLED_I2C::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

    /* 4-bit for each color */
    uint8_t val[4] = { TOSHIBA_LED_PWM0, (uint8_t)(rgb.b >> 4),
                       (uint8_t)(rgb.g / 16), (uint8_t)(rgb.r / 16) };

    _dev->transfer(val, sizeof(val), nullptr, 0);
}

#endif  // AP_NOTIFY_TOSHIBALED_ENABLED
