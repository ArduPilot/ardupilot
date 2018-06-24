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

#include <AP_OSD/AP_OSD_Backend.h>

class AP_OSD_MAX7456 : public AP_OSD_Backend {

public:

    static AP_OSD_Backend *probe(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    //draw given text to framebuffer
    void write(uint8_t x, uint8_t y, const char* text, uint8_t char_attr) override;

    //initilize display port and underlying hardware
    bool init() override;

    //flush framebuffer to screen
    void flush() override;

    //clear framebuffer
    void clear() override;

private:

    //constructor
    AP_OSD_MAX7456(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    void buffer_add_cmd(uint8_t reg, uint8_t arg);

    bool update_font();

    void check_reinit();

    void reinit();

    void transfer_frame();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t  video_signal_reg;
    bool initialized;

    static const uint16_t video_buffer_chars_ntsc = 390;
    static const uint16_t video_buffer_chars_pal = 480;
    static const uint8_t max_updated_chars = 64;
    static const uint16_t spi_buffer_size = ((max_updated_chars + 1) * 8);
    
    uint8_t frame[video_buffer_chars_pal];

    //frame already transfered to max
    //used to optimize number of characters updated
    uint8_t shadow_frame[video_buffer_chars_pal];

    uint8_t attr[video_buffer_chars_pal];

    uint8_t shadow_attr[video_buffer_chars_pal];

    uint8_t buffer[spi_buffer_size];
    int buffer_offset;

    uint32_t last_signal_check;
    uint32_t video_detect_time;

    uint16_t max_screen_size;
};
