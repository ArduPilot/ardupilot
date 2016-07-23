/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
#include "Display_SSD1306_I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#define SSD1306_I2C_BUS 2
#define SSD1306_I2C_ADDR 0x3C    // default I2C bus address

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();


bool Display_SSD1306_I2C::hw_init()
{
    struct {
        uint8_t reg;
        uint8_t seq[31];
    } init_seq = { 0x0,  {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3,
                          0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                          0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF,
                          0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
                          0xAF, 0x21, 0, 127, 0x22, 0, 7 } };

    _dev = std::move(hal.i2c_mgr->get_device(SSD1306_I2C_BUS, SSD1306_I2C_ADDR));

    // take i2c bus sempahore
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // init display
    _dev->transfer((uint8_t *)&init_seq, sizeof(init_seq), nullptr, 0);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    // clear display
    hw_update();

    return true;
}

bool Display_SSD1306_I2C::hw_update()
{
    struct PACKED {
        uint8_t reg;
        uint8_t cmd[6];
    } command = { 0x0, {0x21, 0, 127, 0x22, 0, 7} };

    struct PACKED {
        uint8_t reg;
        uint8_t db[SSD1306_ROWS];
    } display_buffer = { 0x40, {} };

    if (!_dev || !_dev->get_semaphore()->take(5)) {
        return false;
    }

    // write buffer to display
    for (uint8_t i = 0; i < (SSD1306_COLUMNS / SSD1306_COLUMNS_PER_PAGE); i++) {
        command.cmd[4] = i;

        _dev->transfer((uint8_t *)&command, sizeof(command), nullptr, 0);

        memcpy(&display_buffer.db[0], &_displaybuffer[i * SSD1306_ROWS], SSD1306_ROWS);
        _dev->transfer((uint8_t *)&display_buffer, sizeof(display_buffer), nullptr, 0);
    }

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    return true;
}

bool Display_SSD1306_I2C::set_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SSD1306_ROWS) || (y >= SSD1306_COLUMNS)) {
        return false;
    }
    // set pixel in buffer
    _displaybuffer[x + (y / 8 * SSD1306_ROWS)] |= 1 << (y % 8);

    return true;
}

bool Display_SSD1306_I2C::clear_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SSD1306_ROWS) || (y >= SSD1306_COLUMNS)) {
        return false;
    }
    // clear pixel in buffer
    _displaybuffer[x + (y / 8 * SSD1306_ROWS)] &= ~(1 << (y % 8));

    return true;
}
