/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <AP_HAL/HAL.h>
#include <AP_OSD/AP_OSD.h>


class AP_OSD_Backend {

public:
    //constructor
    AP_OSD_Backend(AP_OSD &osd): _osd(osd) {};

    //destructor
    virtual ~AP_OSD_Backend(void) {}

    //draw given text to framebuffer
    virtual void write(uint8_t x, uint8_t y, const char* text, uint8_t char_attr = 0) = 0;

    //draw formatted text to framebuffer
    virtual void write(uint8_t x, uint8_t y, bool blink, const char *fmt, ...);

    //initilize framebuffer and underlying hardware
    virtual bool init() = 0;

    //update screen
    virtual void flush() = 0;

    //clear screen
    virtual void clear() = 0;

    enum character_attribute_t {
        BLINK = (1 << 4),
        INVERT = (1 << 3),
    };

protected:
    AP_OSD& _osd;

    // get font choice
    uint8_t get_font_num(void) const { return (uint8_t)_osd.font_num.get(); }

};


