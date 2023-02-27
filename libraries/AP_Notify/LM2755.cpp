/*
  LM2755 I2C driver
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

/* LED driver for LM2755 */

#include "LM2755.h"

#if AP_NOTIFY_LM2755_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define LM2755_LED_BRIGHT  31    // full brightness
#define LM2755_LED_MEDIUM  20    // medium brightness
#define LM2755_LED_DIM     10    // dim
#define LM2755_LED_OFF     0     // off

enum class Register {
    GENERAL_PURPOSE = 0x10,
    D1_HIGH_LEVEL = 0xA9,
    D1_LOW_LEVEL = 0xA8,
    D2_HIGH_LEVEL = 0xB9,
    D2_LOW_LEVEL = 0xB8,
    D3_HIGH_LEVEL = 0xC9,
    D3_LOW_LEVEL = 0xC8,
};

// registers correpsonding to rgb values:
static const Register registers_high[] {
    Register::D1_HIGH_LEVEL,
    Register::D2_HIGH_LEVEL,
    Register::D3_HIGH_LEVEL
};
static const Register registers_low[] {
    Register::D1_LOW_LEVEL,
    Register::D2_LOW_LEVEL,
    Register::D3_LOW_LEVEL
};


LM2755::LM2755(uint8_t bus, uint8_t _addr)
    : RGBLed(LM2755_LED_OFF, LM2755_LED_BRIGHT, LM2755_LED_MEDIUM, LM2755_LED_DIM)
    , _bus(bus)
    , addr(_addr)
{
}

bool LM2755::init(void)
{
    // first look for led on external bus
    _dev = std::move(hal.i2c_mgr->get_device(_bus, addr));
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // set high and low values for each of rgb to 0
    for (uint8_t i=0; i<ARRAY_SIZE(registers_low); i++) {
        _dev->write_register((uint8_t)registers_low[i], 0);
        _dev->write_register((uint8_t)registers_high[i], 0);
    }

    // enable the dimming feature:
    bool ret = _dev->write_register(
        (uint8_t)Register::GENERAL_PURPOSE,
        1U << 3 |  // d1 dimming
        1U << 4 |  // d2 dimming
        1U << 5    // d3 dimming
        );
    if (!ret) {
        return false;
    }

    _dev->set_retries(1);

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&LM2755::_timer, void));

    return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool LM2755::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb[0] = red;
    rgb[1] = green;
    rgb[2] = blue;
    _need_update = true;
    return true;
}

void LM2755::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

    for (uint8_t i=0; i< ARRAY_SIZE(rgb); i++) {
        const uint8_t new_colour = rgb[i];
        const uint8_t last_sent = last_sent_rgb[i];
        if (new_colour == last_sent) {
            continue;
        }
        // note that new_colour is alread scaled by supplying
        // brightness values in the constructor.

        // don't get the endpoints out-of-order:
        if (new_colour < last_sent) {
            _dev->write_register((uint8_t)registers_low[i], new_colour);
            _dev->write_register((uint8_t)registers_high[i], new_colour);
        } else {
            _dev->write_register((uint8_t)registers_high[i], new_colour);
            _dev->write_register((uint8_t)registers_low[i], new_colour);
        }
        last_sent_rgb[i] = new_colour;
    }
}

#endif  // AP_NOTIFY_LM2755_ENABLED
