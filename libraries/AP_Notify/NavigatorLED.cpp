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

// This driver is for the neopixel on the Blue Robotics Navigator board
// designed for the Raspberry Pi 4. The neopixel data in is connected to
// the Raspberry Pi's MOSI pin. The clock, chip select, and MISO pins are
// not used. The data is sent to the neopixel in 24 'SPI bytes', where each
// spi byte is formatted to appear as a single bit of data to the neopixel.

#include "AP_Notify_config.h"

#if AP_NOTIFY_NAVIGATOR_LED_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Notify/AP_Notify.h"
#include "NavigatorLED.h"

#define NEOPIXEL_LED_LOW    0x18
#define NEOPIXEL_LED_MEDIUM 0x40
#define NEOPIXEL_LED_HIGH   0xFF
#define NEOPIXEL_LED_OFF    0x00

#define LED_T0 0b11000000
#define LED_T1 0b11110000

extern const AP_HAL::HAL& hal;

NavigatorLED::NavigatorLED() :
    RGBLed(NEOPIXEL_LED_OFF, NEOPIXEL_LED_HIGH, NEOPIXEL_LED_MEDIUM, NEOPIXEL_LED_LOW)
{
}

bool NavigatorLED::init()
{
    _dev = hal.spi->get_device("led");
    if (!_dev) {
        return false;
    }
    return true;
}

bool NavigatorLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!_dev) {
        return false;
    }

    // format our spi bytes to transmit the desired color data
    _setup_data(red, green, blue);

    // take i2c bus sempahore
    WITH_SEMAPHORE(_dev->get_semaphore());

    // send the new color data
    return _dev->transfer(_data, sizeof(_data), nullptr, 0);
}

// Get our bytes ready to send the desired color data
// Datasheet: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
// 24bit msg as 3 byte GRB (not RGB) where first bit is G7, and last bit is B0
// (first) G7|G6|G5|G4|G3|G2|G1|G0|R7|R6|R5|R4|R3|R2|R1|R0|B7|B6|B5|B4|B3|B2|B1|B0 (last)
void NavigatorLED::_setup_data(uint8_t red, uint8_t green, uint8_t blue)
{
    for (uint8_t i = 0; i < 8; i++) {
        _data[i] = (green & (1<<(7-i))) ? LED_T1 : LED_T0;
    }
    for (uint8_t i = 0; i < 8; i++) {
        _data[8 + i] = (red & (1<<(7-i))) ? LED_T1 : LED_T0;
    }
    for (uint8_t i = 0; i < 8; i++) {
        _data[16 + i] = (blue & (1<<(7-i))) ? LED_T1 : LED_T0;
    }
}

#endif  // AP_NOTIFY_NAVIGATOR_LED_ENABLED
