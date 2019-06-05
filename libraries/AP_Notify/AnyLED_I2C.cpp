/*
   AnyLED I2C driver
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
#include "AnyLED_I2C.h"
#include <AP_HAL/AP_HAL.h>
#include <utility>


extern const AP_HAL::HAL& hal;

#define ANY_LED_BRIGHT      0xFF    // full brightness
#define ANY_LED_MEDIUM      0xFF    // medium brightness
#define ANY_LED_DIM         0xFF    // dim
#define ANY_LED_OFF         0x00    // off

#define ANY_LED_I2C_ADDR    0x90    // default I2C bus address



AnyLED_I2C::AnyLED_I2C(uint8_t bus)
    : RGBLed(ANY_LED_OFF, ANY_LED_BRIGHT, ANY_LED_MEDIUM, ANY_LED_DIM)
    , _bus(bus)
{
}

bool AnyLED_I2C::hw_init(void)
{
    // first look for led on external bus
    _dev = std::move(hal.i2c_mgr->get_device(_bus, ANY_LED_I2C_ADDR));
    if (!_dev) {
        return false;
    }

    _dev->get_semaphore()->take_blocking();
    _dev->set_retries(10);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AnyLED_I2C::_timer, void));

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool AnyLED_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb = {red, green, blue};
    _need_update = true;
    return true;
}

void AnyLED_I2C::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

    uint8_t val[3] = {rgb.r, rgb.g, rgb.b};
    #if DEBUG_ANYLED_I2C
        printf("r: %d, g: %d, b: %d\n", rgb.r, rgb.g, rgb.b);
    #endif

    bool ret = _dev->transfer(val, sizeof(val), nullptr, 0);
    
    if(!ret){
        #if DEBUG_ANYLED_I2C
            printf("anyled i2c transfer failed!!!\n");
        #endif
        return;
    }
}
