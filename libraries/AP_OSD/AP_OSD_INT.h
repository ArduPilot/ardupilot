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


class AP_OSD_INT : public AP_OSD_Backend {

public:

    static AP_OSD_Backend *probe(AP_OSD &osd);

    void write(uint8_t x, uint8_t y, const char* text) override;

    bool init() override;

    void flush() override;

    void clear() override;

    static const int video_y = 288;
    static const int video_x = 384;

    static const int text_lines = 16;
    static const int text_columns = 30;
    static const int char_width = 12;
    static const int char_height = 18;
    
    static const int line_disp = 0;

    typedef uint32_t osd_frame_t[video_y][video_x/32];
    osd_frame_t frame;
    osd_frame_t frame_mask;
    osd_frame_t frame_levl;
private:
    void draw_char(int ch_x, int ch_y, char c);
    void draw_point(int x, int y);
    AP_OSD_INT(AP_OSD &osd);
    bool render_buffer();
    bool initialized;
    static const uint32_t font_data[256 * 18];
};

void osd_setup(AP_OSD_INT *);

