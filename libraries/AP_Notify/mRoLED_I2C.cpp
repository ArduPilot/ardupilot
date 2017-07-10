/*
  mRoLED I2C driver
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

/* LED driver for mRo module */

#include "mRoLED_I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define mRo_LED_I2C_ADDR     0x38 // default I2C bus address of NCP5623
                                  // Note that it has no sub-address nor read-back register
#define CMD_SHUTDOWN            0
#define CMD_MAX_CURRENT      0x38
#define PH2_I2C_BUS_INTERNAL    1 // I2C1 on GPS1 connector, 3.3V
#define PH2_I2C_BUS_EXTERNAL    0 // I2C2 connector or I2C2 on GPS2, 3.3 - 5 V

#define mRoDebug true

bool mRoLED_I2C::hw_init()
{
    // first look for led on external bus
/* cubirootk
    _dev = std::move(hal.i2c_mgr->get_device(PH2_I2C_BUS_INTERNAL, mRo_LED_I2C_ADDR));
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // enable the led
    bool ret = _dev->write_register(CMD_MAX_CURRENT, CMD_MAX_CURRENT);

    // on failure try the internal bus
    if (!ret) {
        // give back external bus semaphore
        _dev->get_semaphore()->give();
        // get internal I2C bus driver
        _dev = std::move(hal.i2c_mgr->get_device(PH2_I2C_BUS_EXTERNAL, mRo_LED_I2C_ADDR));
        if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            return false;
        }
        ret = _dev->write_register(CMD_MAX_CURRENT, CMD_MAX_CURRENT);
    }
*/



#if mRoDebug
    _dev = std::move(hal.i2c_mgr->get_device(0, 0x09)); // for BlinkM
    //    void Copter::gcs_send_text(MAV_SEVERITY_CRITICAL,"mRoLED init");
    //    hal.console->printf("mRoLED init\n");
  #else
    _dev = std::move(hal.i2c_mgr->get_device(0, mRo_LED_I2C_ADDR));
  #endif
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    // update the red, green and blue values to zero
#if mRoDebug
    uint8_t val[4] = { 0x6e, 0x80, 0x80, 0x80 };  // half white
  #else
    uint8_t val[4] = { 0x48, 0x68, 0x88, CMD_MAX_CURRENT }; //half white
  #endif
    bool ret = _dev->transfer(val, sizeof(val), nullptr, 0);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&mRoLED_I2C::_timer, void));
    
    return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool mRoLED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb = {red, green, blue};
    _need_update = true;
    return true;
}

void mRoLED_I2C::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

#if mRoDebug
    uint8_t val[4] = { 0x6e, rgb.r, rgb.g, rgb.b };
  #else
    /* 5-bit for each color */
    uint8_t blue_adjusted =  ((rgb.b & 0xf8) >>3) | 0x40; // PWM1 (upper LED)
    uint8_t green_adjusted = ((rgb.g / 8) & 0x1f) | 0x60; // PWM2 ( mid  LED)
    uint8_t red_adjusted =   ((rgb.r / 8) & 0x1f) | 0x80; // PWM3 (lower LED)
    uint8_t val[4] = { red_adjusted, green_adjusted, blue_adjusted, 0x38 };
  #endif
    _dev->transfer(val, sizeof(val), nullptr, 0);
}
