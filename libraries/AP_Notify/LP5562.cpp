/*
  LP5562 I2C driver
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

/* LED driver for LP5562 */

#include "LP5562.h"

#if AP_NOTIFY_LP5562_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define LP5562_LED_BRIGHT  255    // full brightness
#define LP5562_LED_MEDIUM  170    // medium brightness
#define LP5562_LED_DIM     85     // dim
#define LP5562_LED_OFF     0      // off

enum class Register {
    ENABLE  = 0x00,
    B_PWM   = 0x02,
    G_PWM   = 0x03,
    R_PWM   = 0x04,
    B_CURRENT= 0x05,
    G_CURRENT= 0x06,
    R_CURRENT= 0x07,
    CONFIG  = 0x08,
    RESET   = 0x0D,
    LED_MAP = 0x70,
};

LP5562::LP5562(uint8_t bus, uint8_t addr)
    : RGBLed(LP5562_LED_OFF, LP5562_LED_BRIGHT, LP5562_LED_MEDIUM, LP5562_LED_DIM)
    , _bus(bus)
    , _addr(addr)
{
}

bool LP5562::init(void)
{
    _dev = hal.i2c_mgr->get_device_ptr(_bus, _addr);
    if (!_dev) {
        return false;
    }

    if (!configure_dev()) {
        delete _dev;
        _dev = nullptr;
        return false;
    }

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&LP5562::_timer, void));

    return true;
}

bool LP5562::configure_dev()
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // reset the device and probe to see if this device looks like an LP5662:
    if (!_dev->write_register((uint8_t)Register::RESET, 0xff)) {
        return false;
    }

    // reset delay; unsure if this is really required:
    hal.scheduler->delay_microseconds(100);

    // check the GBR PWM control registers have their reset values:
    for (uint8_t i=(uint8_t)Register::B_CURRENT; i<=(uint8_t)Register::R_CURRENT; i++) {
        uint8_t value;
        if (!_dev->read_registers(i, &value, 1)) {
            return false;
        }
        if (value != 0xAF) {  // 0xAF is the startup value for these registers per datasheet
            return false;
        }
    }

    //  values here are taken literally from 7.3.2 in the datasheet.
    //  See the simulator for register breakdown.

    // chip enable:
    if (!_dev->write_register((uint8_t)Register::ENABLE, 0b1000000)) {
        return false;
    }

    // start-up-delay:
    hal.scheduler->delay_microseconds(500);

    // use internal clock:
    if (!_dev->write_register((uint8_t)Register::CONFIG, 0b00000001)) {
        return false;
    }

    // set direct PWM control:
    if (!_dev->write_register((uint8_t)Register::LED_MAP, 0b00000000)) {
        return false;
    }

    _dev->set_retries(1);

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool LP5562::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    bgr[0] = blue;
    bgr[1] = green;
    bgr[2] = red;
    _need_update = true;
    return true;
}

void LP5562::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

    for (uint8_t i=0; i< ARRAY_SIZE(bgr); i++) {
        const uint8_t new_colour = bgr[i];
        const uint8_t last_sent = last_sent_bgr[i];
        if (new_colour == last_sent) {
            continue;
        }
        // note that new_colour is already scaled by supplying
        // brightness values in the constructor.

        // take advantage of the linear layout of the registers.  The
        // direct PWM registers start at 0x02 for blue:
        _dev->write_register((uint8_t)0x02 + i, new_colour);

        last_sent_bgr[i] = new_colour;
    }
}

#endif  // AP_NOTIFY_LP5562_ENABLED
