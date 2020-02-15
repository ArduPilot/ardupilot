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
#include "Display_SH1106_I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

// constructor
Display_SH1106_I2C::Display_SH1106_I2C(AP_HAL::OwnPtr<AP_HAL::Device> dev) :
    _dev(std::move(dev))
{
}

Display_SH1106_I2C::~Display_SH1106_I2C()
{
}

Display_SH1106_I2C *Display_SH1106_I2C::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    Display_SH1106_I2C *driver = new Display_SH1106_I2C(std::move(dev));
    if (!driver || !driver->hw_init()) {
        delete driver;
        return nullptr;
    }
    return driver;
}


bool Display_SH1106_I2C::hw_init()
{
    struct PACKED {
        uint8_t reg;
        uint8_t seq[26];
    } init_seq = { 0x0,  {
            0xAE,         // Display OFF
            0xA1,         // Segment re-map
            0xC8,         // COM Output Scan Direction
            0xA8, 0x3F,   // MUX Ratio
            0xD5, 0x50,   // Display Clock Divide Ratio and Oscillator Frequency: (== +0%)
            0xD3, 0x00,   // Display Offset
            0xDB, 0x40,   // VCOMH Deselect Level
            0x81, 0xCF,   // Contrast Control
            0xAD, 0x8B,   // DC-DC Control Mode: 1b (== internal DC-DC enabled) (AKA: Enable charge pump regulator)
            0x40,         // Display Start Line
            0xDA, 0x12,   // +++ COM Pins hardware configuration
            0xD9, 0xF1,   // +++ Pre-charge Period
            0xA4,         // +++ Entire Display ON (ignoring RAM): 0b (== OFF)
            0xA6,         // +++ Normal/Inverse Display: 0b (== Normal)
            0xAF,         // Display ON
            0xB0,         // Page Address
            0x02, 0x10    // Column Address
    } };

    memset(_displaybuffer, 0, SH1106_COLUMNS * SH1106_ROWS_PER_PAGE);

    // take i2c bus semaphore
    if (!_dev) {
        return false;
    }
    _dev->get_semaphore()->take_blocking();

    // init display
    bool success = _dev->transfer((uint8_t *)&init_seq, sizeof(init_seq), nullptr, 0);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    if (success) {
        _need_hw_update = true;
        _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&Display_SH1106_I2C::_timer, void));
    }

    return success;
}

void Display_SH1106_I2C::hw_update()
{
    _need_hw_update = true;
}

void Display_SH1106_I2C::_timer()
{
    if (!_need_hw_update) {
        return;
    }
    _need_hw_update = false;

    struct PACKED {
        uint8_t reg;
        uint8_t column_0_3;
        uint8_t column_4_7;
        uint8_t page;
    } command = { 0x0, 0x2, 0x10, 0xB0 };

    struct PACKED {
        uint8_t reg;
        uint8_t db[SH1106_COLUMNS/2];
    } display_buffer = { 0x40, {} };

    // write buffer to display
    for (uint8_t i = 0; i < (SH1106_ROWS / SH1106_ROWS_PER_PAGE); i++) {
        command.page = 0xB0 | (i & 0x0F);
        _dev->transfer((uint8_t *)&command, sizeof(command), nullptr, 0);

        memcpy(&display_buffer.db[0], &_displaybuffer[i * SH1106_COLUMNS], SH1106_COLUMNS/2);
        _dev->transfer((uint8_t *)&display_buffer, SH1106_COLUMNS/2 + 1, nullptr, 0);
        memcpy(&display_buffer.db[0], &_displaybuffer[i * SH1106_COLUMNS + SH1106_COLUMNS/2 ], SH1106_COLUMNS/2);
        _dev->transfer((uint8_t *)&display_buffer, SH1106_COLUMNS/2 + 1, nullptr, 0);
    }
}

void Display_SH1106_I2C::set_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SH1106_COLUMNS) || (y >= SH1106_ROWS)) {
        return;
    }
    // set pixel in buffer
    _displaybuffer[x + (y / 8 * SH1106_COLUMNS)] |= 1 << (y % 8);
}

void Display_SH1106_I2C::clear_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SH1106_COLUMNS) || (y >= SH1106_ROWS)) {
        return;
    }
    // clear pixel in buffer
    _displaybuffer[x + (y / 8 * SH1106_COLUMNS)] &= ~(1 << (y % 8));
}

void Display_SH1106_I2C::clear_screen()
{
    memset(_displaybuffer, 0, SH1106_COLUMNS * SH1106_ROWS_PER_PAGE);
}
