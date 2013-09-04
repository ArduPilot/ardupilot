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

#include <AP_HAL.h>
#include "ToshibaLED.h"
#include "ToshibaLED_I2C.h"

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

    // enable the led
    bool ret = (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_ENABLE, 0x03) == 0);

    // update the red, green and blue values to zero
    ret &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, TOSHIBA_LED_OFF) == 0);
    ret &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM1, TOSHIBA_LED_OFF) == 0);
    ret &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM2, TOSHIBA_LED_OFF) == 0);

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

    bool success = true;
    // update the red value
    success &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM2, red>>4) == 0);
    // update the green value
    success &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM1, green>>4) == 0);
    // update the blue value
    success &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, blue>>4) == 0);

    // give back i2c semaphore
    i2c_sem->give();
    return success;
}
