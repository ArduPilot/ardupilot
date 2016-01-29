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
#include "ToshibaLED_I2C.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define TOSHIBA_LED_ADDRESS 0x55    // default I2C bus address
#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register

bool ToshibaLED_I2C::hw_init()
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // disable recording of i2c lockup errors
    hal.i2c->ignore_errors(true);

    // enable the led
    bool ret = (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_ENABLE, 0x03) == 0);

    // update the red, green and blue values to zero
    uint8_t val[3] = { _led_off, _led_off, _led_off };
    ret &= (hal.i2c->writeRegisters(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, 3, val) == 0);

    // re-enable recording of i2c lockup errors
    hal.i2c->ignore_errors(false);

    // give back i2c semaphore
    i2c_sem->give();

    return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool ToshibaLED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        return false;
    }

    // update the red value
    uint8_t val[3] = { (uint8_t)(blue>>4), (uint8_t)(green>>4), (uint8_t)(red>>4) };
    bool success = (hal.i2c->writeRegisters(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, 3, val) == 0);

    // give back i2c semaphore
    i2c_sem->give();
    return success;
}
