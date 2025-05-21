/*
  IS31FL3195 I2C driver
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

/* LED driver for IS31FL3195 */

#include "IS31FL3195.h"

#if AP_NOTIFY_IS31FL3195_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define IS31FL3195_LED_BRIGHT  255    // full brightness
#define IS31FL3195_LED_MEDIUM  170    // medium brightness
#define IS31FL3195_LED_DIM     85     // dim
#define IS31FL3195_LED_OFF     0      // off

#define REGISTER_MAGIC_VLAUE 0xc5

enum class Register {
    PRODUCT_ID       = 0x00,  // not really product ID, rather address of this LED
    SHUTDOWN_CONTROL = 0x01,
    OUT1             = 0x10,
    OUT2             = 0x21,
    OUT3             = 0x32,
    COLOR_UPDATE     = 0x50,
    RESET            = 0x5f,
};

IS31FL3195::IS31FL3195(uint8_t bus, uint8_t addr)
    : RGBLed(IS31FL3195_LED_OFF, IS31FL3195_LED_BRIGHT, IS31FL3195_LED_MEDIUM, IS31FL3195_LED_DIM)
    , _bus(bus)
    , _addr(addr)
{
}

bool IS31FL3195::init(void)
{
    _dev = hal.i2c_mgr->get_device_ptr(_bus, _addr);
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // check the PRODUCT_ID register to check we have found the device. The product ID register
    // should contain the 8 bit I2C address
    uint8_t v;
    if (!_dev->read_registers((uint8_t)Register::PRODUCT_ID, &v, 1) || v != _addr<<1U) {
        return false;
    }
    
    // reset the device and probe to see if this device looks like an IS31FL3195:
    if (!_dev->write_register((uint8_t)Register::RESET, REGISTER_MAGIC_VLAUE)) {
        return false;
    }

    // reset delay; unsure if this is really required:
    hal.scheduler->delay_microseconds(100);

    // check SHUTDOWN_CONTROL as an additional ID after reset
    if (!_dev->read_registers((uint8_t)Register::SHUTDOWN_CONTROL, &v, 1) || v != 0xf0) {
        return false;
    }
    
    // come out of shutdown mode
    if (!_dev->write_register((uint8_t)Register::SHUTDOWN_CONTROL, 0xf1)) {
        return false;
    }
    
    _dev->set_retries(1);

    // update at 50Hz
    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&IS31FL3195::_timer, void));

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool IS31FL3195::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb[0] = red;
    rgb[1] = green;
    rgb[2] = blue;
    _need_update = true;
    return true;
}

void IS31FL3195::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

    // write color values
    _dev->write_register(uint8_t(Register::OUT1), rgb[0]);
    _dev->write_register(uint8_t(Register::OUT2), rgb[1]);
    _dev->write_register(uint8_t(Register::OUT3), rgb[2]);
    _dev->write_register(uint8_t(Register::COLOR_UPDATE), REGISTER_MAGIC_VLAUE);
}

#endif  // AP_NOTIFY_IS31FL3195_ENABLED
