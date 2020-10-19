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
#include <AP_Common/Bitmask.h>

#if HAL_WITH_OSD_BITMAP
class AP_OSD_MAX7456 : public AP_OSD_Backend
{

public:

    static AP_OSD_Backend *probe(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    //draw given text to framebuffer
    void write(uint8_t x, uint8_t y, const char* text) override;

    //initilize display port and underlying hardware
    bool init() override;

    //flush framebuffer to screen
    void flush() override;

    //clear framebuffer
    void clear() override;

    // return a correction factor used to display angles correctly
   float get_aspect_ratio_correction() const override;

private:

    //constructor
    AP_OSD_MAX7456(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    void buffer_add_cmd(uint8_t reg, uint8_t arg);

    bool update_font();

    bool check_font_char(uint8_t chr, const uint8_t* font_data);

    bool update_font_char(uint8_t chr, const uint8_t* font_data);

    void check_reinit();

    void reinit();

    void transfer_frame();

    bool is_dirty(uint8_t x, uint8_t y);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t  video_signal_reg;
    bool initialized;
    uint8_t last_font;
    int8_t last_v_offset;
    int8_t last_h_offset;

    static const uint8_t video_lines_ntsc = 13;
    static const uint8_t video_lines_pal = 16;
    static const uint8_t video_columns = 30;
    static const uint16_t spi_buffer_size = 512;

    uint8_t frame[video_lines_pal][video_columns];

    //frame already transfered to max
    //used to optimize number of characters updated
    uint8_t shadow_frame[video_lines_pal][video_columns];

    uint8_t buffer[spi_buffer_size];
    int buffer_offset;

    uint32_t last_signal_check;
    uint32_t video_detect_time;

    uint16_t video_lines;
};
#endif // HAL_WITH_OSD_BITMAP
