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

#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_HAL/Util.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define SYM_DIG_OFS_1 0x90
#define SYM_DIG_OFS_2 0xA0

void AP_OSD_Backend::write(uint8_t x, uint8_t y, bool blink, const char *fmt, ...)
{
#if OSD_ENABLED
    if (blink && (blink_phase < 2)) {
        return;
    }
    char buff[32+1]; // +1 for snprintf null-termination
    va_list ap;
    va_start(ap, fmt);
    int res = hal.util->vsnprintf(buff, sizeof(buff), fmt, ap);
    res = MIN(res, int(sizeof(buff)));
    if (res > 0 && check_option(AP_OSD::OPTION_DECIMAL_PACK)) {
        // automatically use packed decimal characters
        // based on fiam idea implemented in inav osd
        char *p = strchr(&buff[1],'.');
        if (p && isdigit(p[1]) && isdigit(p[-1])) {
            p[-1] += SYM_DIG_OFS_1;
            p[1] += SYM_DIG_OFS_2;
            memmove(p, p+1, strlen(p+1)+1);
            res--;
        }
    }
    if (res < int(sizeof(buff))-1) {
        write(x, y, buff);
    }
    va_end(ap);
#endif
}

/*
  load a font from sdcard or ROMFS
 */
FileData *AP_OSD_Backend::load_font_data(uint8_t font_num)
{
    FileData *fd;

    // first try from microSD
    char fontname[] = "font0.bin";
    fontname[4] = font_num + '0';

    fd = AP::FS().load_file(fontname);
    if (fd == nullptr) {
        char fontname_romfs[] = "@ROMFS/font0.bin";
        fontname_romfs[7+4] = font_num + '0';
        fd = AP::FS().load_file(fontname_romfs);
    }
    if (fd == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "OSD: Failed to load font %u", font_num);
        if (font_num != 0) {
            // fallback to font0.bin. This allows us to reduce the
            // number of fonts we include in flash without breaking
            // user setups
            return load_font_data(0);
        }
    }
    return fd;
}
