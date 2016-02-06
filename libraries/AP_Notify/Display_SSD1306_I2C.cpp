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

#include <AP_HAL/AP_HAL.h>

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

bool Display_SSD1306_I2C::hw_init()
{
    // display init sequence
    uint8_t init_seq[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3,
                          0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                          0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF,
                          0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
                          0xAF, 0x21, 0, 127, 0x22, 0, 7
                         };

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // disable recording of i2c lockup errors
    hal.i2c->ignore_errors(true);

    // init display
    hal.i2c->writeRegisters(SSD1306_I2C_ADDRESS, 0x0, sizeof(init_seq), init_seq);

    // re-enable recording of i2c lockup errors
    hal.i2c->ignore_errors(false);

    // give back i2c semaphore
    i2c_sem->give();

    // clear display
    hw_update();

    return true;
}

bool Display_SSD1306_I2C::hw_update()
{
    uint8_t command[] = {0x21, 0, 127, 0x22, 0, 7};

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        return false;
    }

    // write buffer to display
    for (uint8_t i = 0; i < (SSD1306_COLUMNS / SSD1306_COLUMNS_PER_PAGE); i++) {
        command[4] = i;
        hal.i2c->writeRegisters(SSD1306_I2C_ADDRESS, 0x0, sizeof(command), command);
        hal.i2c->writeRegisters(SSD1306_I2C_ADDRESS, 0x40, SSD1306_ROWS, &_displaybuffer[i * SSD1306_ROWS]);
    }

    // give back i2c semaphore
    i2c_sem->give();

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
