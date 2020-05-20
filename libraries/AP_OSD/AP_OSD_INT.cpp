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
#include <AP_OSD/AP_OSD_INT.h>

#include <AP_HAL/Util.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Scheduler.h>

#include <utility>

AP_OSD_INT::AP_OSD_INT(AP_OSD &osd):
    AP_OSD_Backend(osd)
{
}

bool AP_OSD_INT::init()
{
    osd_setup(this);
    initialized = true;
    return true;
}

AP_OSD_Backend *AP_OSD_INT::probe(AP_OSD &osd)
{
    AP_OSD_INT *backend = new AP_OSD_INT(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

void AP_OSD_INT::flush() {
    if (initialized) {
        for (int iy = 0; iy < AP_OSD_INT::video_y; iy++) {
            for (int ix = 0; ix < AP_OSD_INT::video_x / 32; ix++) {
                frame_levl[iy][ix] = frame[iy][ix];
                uint32_t mask = frame[iy][ix] | (frame[iy][ix] << 1)
                        | (frame[iy][ix] >> 1);
                if (iy - 1 >= 0) {
                    mask |= frame[iy - 1][ix];
                }
                if (iy + 1 < AP_OSD_INT::video_y) {
                    mask |= frame[iy + 1][ix];
                }
                if (ix - 1 >= 0) {
                    mask |= ((frame[iy][ix - 1] & 1) << 31);
                }
                if (ix + 1 < AP_OSD_INT::video_x) {
                    mask |= (frame[iy][ix + 1] >> 31);
                }
                frame_mask[iy][ix] = mask;
            }
        }
    }
}

void AP_OSD_INT::clear()
{
    AP_OSD_Backend::clear();
    memset(frame, 0, sizeof(frame));
}

void AP_OSD_INT::write(uint8_t x, uint8_t y, const char* text)
{
    if (y >= text_lines || text == nullptr) {
        return;
    }
    while ((x < text_columns) && (*text != 0)) {
        draw_char(x, y, *text);
        ++text;
        ++x;
    }
}

void AP_OSD_INT::draw_char(int ch_x, int ch_y, char c)
{
    for (int iy = 0; iy < char_height; iy++) {
        for (int ix = 0; ix < char_width; ix++) {
            if ((font_data[((uint8_t)c) * char_height + iy] >> (31 - ix)) & 1) {
                draw_point(ch_x * char_width + ix, ch_y * char_height + iy);
            }
        }
    }
}

void AP_OSD_INT::draw_point(int x, int y)
{
    x += line_disp;
    uint32_t word_offset = x / 32;
    uint32_t bit_offset = x % 32;
    frame[y][word_offset] |= (1 << (31 - bit_offset));
}
