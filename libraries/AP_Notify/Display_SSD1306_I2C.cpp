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

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// constructor
Display_SSD1306_I2C::Display_SSD1306_I2C(AP_HAL::OwnPtr<AP_HAL::Device> dev) :
    _dev(std::move(dev))
{
    _displaybuffer_sem = hal.util->new_semaphore();
}

Display_SSD1306_I2C::~Display_SSD1306_I2C()
{
    // note that a callback is registered below.  here we delete the
    // semaphore, in that callback we use it.  That means - don't
    // delete this Display backend if you've ever registered that
    // callback!  This delete is only here to not leak memory during
    // the detection phase.
    delete _displaybuffer_sem;
}


Display_SSD1306_I2C *Display_SSD1306_I2C::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    Display_SSD1306_I2C *driver = new Display_SSD1306_I2C(std::move(dev));
    if (!driver || !driver->hw_init()) {
        delete driver;
        return nullptr;
    }
    return driver;
}

bool Display_SSD1306_I2C::hw_init()
{
    struct PACKED {
        uint8_t reg;
        uint8_t seq[31];
    } init_seq = { 0x0,  {
                          // LEGEND:
                          //    *** is out of sequence for init steps recommended in datasheet
                          //    +++ not listed in sequence for init steps recommended in datasheet

            0xAE,         // Display OFF
            0xD5, 0x80,   // *** Set Display Clock Divide Ratio and Oscillator Frequency
                          //    Clock Divide Ratio: 0b (== 1)
                          //    Oscillator Frequency: 1000b (== +0%)
            0xA8, 0x3F,   // MUX Ratio: 111111b (== 64MUX)
            0xD3, 0x00,   // Display Offset: 0b (== 0)
            0x40,         // Display Start Line: 0b (== 0)
            0x8D, 0x14,   // *** Enable charge pump regulator: 1b (== Enable)
            0x20, 0x00,   // *** Memory Addressing Mode: 00b (== Horizontal Addressing Mode)
            0xA1,         // Segment re-map: 1b (== column address 127 is mapped to SEG0)
            0xC8,         // COM Output Scan Direction: 1b (== remapped mode. Scan from COM[N-1] to COM0)
            0xDA, 0x12,   // COM Pins hardware configuration: 01b (POR)
                          //    (== Alternative COM pin configuration + Disable COM Left/Right remap)
            0x81, 0xCF,   // Contrast Control: 0xCF (== 207 decimal, range 0..255)
            0xD9, 0xF1,   // +++ Pre-charge Period: 0xF1 (== 1 DCLK P1 + 15 DCLK P2)
            0xDB, 0x40,   // +++ VCOMH Deselect Level: 100b (INVALID?!) (== ?!)
            0xA4,         // Entire Display ON (ignoring RAM): (== OFF)
            0xA6,         // Normal/Inverse Display: 0b (== Normal)
            0xAF,         // Display ON: 1b (== ON)
            0x21, 0, 127, // +++ Column Address: (== start:0, end:127)
            0x22, 0, 7    // +++ Page Address: (== start:0, end:7)
    } };

    memset(_displaybuffer, 0, SSD1306_COLUMNS * SSD1306_ROWS_PER_PAGE);

    // take i2c bus semaphore
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // init display
    bool success = _dev->transfer((uint8_t *)&init_seq, sizeof(init_seq), nullptr, 0);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    if (success) {
        _need_hw_update = true;
        _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&Display_SSD1306_I2C::_timer, void));
    }

    return success;
}

void Display_SSD1306_I2C::hw_update()
{
    _need_hw_update = true;
}

void Display_SSD1306_I2C::_timer()
{
    if (!_need_hw_update) {
        return;
    }
    _need_hw_update = false;

    struct PACKED {
        uint8_t reg;
        uint8_t cmd[6];
    } command = { 0x0, {0x21, 0, 127, 0x22, 0, 7} };

    struct PACKED {
        uint8_t reg;
        uint8_t db[SSD1306_COLUMNS/2];
    } display_buffer = { 0x40, {} };

    // write buffer to display
    for (uint8_t i = 0; i < (SSD1306_ROWS / SSD1306_ROWS_PER_PAGE); i++) {
        command.cmd[4] = i;
        _dev->transfer((uint8_t *)&command, sizeof(command), nullptr, 0);

        if (_displaybuffer_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            memcpy(&display_buffer.db[0], &_displaybuffer[i * SSD1306_COLUMNS], SSD1306_COLUMNS/2);
            _displaybuffer_sem->give();
            _dev->transfer((uint8_t *)&display_buffer, SSD1306_COLUMNS/2 + 1, nullptr, 0);
        }

        if (_displaybuffer_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            memcpy(&display_buffer.db[0], &_displaybuffer[i * SSD1306_COLUMNS + SSD1306_COLUMNS/2 ], SSD1306_COLUMNS/2);
            _displaybuffer_sem->give();
            _dev->transfer((uint8_t *)&display_buffer, SSD1306_COLUMNS/2 + 1, nullptr, 0);
        }
    }
}

void Display_SSD1306_I2C::set_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SSD1306_COLUMNS) || (y >= SSD1306_ROWS)) {
        return;
    }
    if (!_displaybuffer_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }
    // set pixel in buffer
    _displaybuffer[x + (y / 8 * SSD1306_COLUMNS)] |= 1 << (y % 8);
    _displaybuffer_sem->give();
}

void Display_SSD1306_I2C::clear_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SSD1306_COLUMNS) || (y >= SSD1306_ROWS)) {
        return;
    }
    if (!_displaybuffer_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }
    // clear pixel in buffer
    _displaybuffer[x + (y / 8 * SSD1306_COLUMNS)] &= ~(1 << (y % 8));
    _displaybuffer_sem->give();
}

void Display_SSD1306_I2C::clear_screen()
{
    if (!_displaybuffer_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }
     memset(_displaybuffer, 0, SSD1306_COLUMNS * SSD1306_ROWS_PER_PAGE);
     _displaybuffer_sem->give();
}
