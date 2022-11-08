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
#include <GCS_MAVLink/GCS.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;
constexpr uint8_t AP_OSD_Backend::symbols[AP_OSD_NUM_SYMBOLS];

#define SYM_DIG_OFS_1 0x90
#define SYM_DIG_OFS_2 0xA0


uint8_t AP_OSD_Backend::convert_to_decimal_packed_characters(char* buff, uint8_t size) {
    if (size == 0) {
        return 0;
    }
#if OSD_ENABLED
    // use packed decimal characters based on fiam idea implemented in inav osd
    // search the decimal separator with a bound and always terminate the string
    char* p = (char*)memchr(&buff[1],'.',size-1);
    if (p && isdigit(p[1]) && isdigit(p[-1])) {
        // remove the decimal separator and replace the digit before and after
        p[-1] += SYM_DIG_OFS_1;
        p[1] += SYM_DIG_OFS_2;
        // shift anything after p[1] 1 character to the left
        const char* move_start = p+1;
        const uint8_t move_size = size-(move_start-buff);
        memmove(p, move_start, move_size);
        p[move_size] = 0x00; // terminate
        return size-1;
    }
#else
    // we guarantee string is terminated
    buff[size-1] = 0x00;
#endif
    return size;
}

uint8_t AP_OSD_Backend::format_string_for_osd(char* buff, uint8_t size, bool decimal_packed, const char *fmt, va_list ap)
{
    if (size == 0) {
        return 0;
    }
#if OSD_ENABLED
    // note: vsnprintf() always terminates the string
    int res = hal.util->vsnprintf(buff, size, fmt, ap);
    res = MIN(res, size);
    if (res > 0 && decimal_packed) {
        // note: convert_to_decimal_packed_characters() always terminates the string
        res = convert_to_decimal_packed_characters(buff, res);
    }
    return res;
#else
    // we guarantee string is terminated
    buff[0] = 0x00;
    // and notify the caller we actually failed
    return 0;
#endif
}

void AP_OSD_Backend::write(uint8_t x, uint8_t y, bool blink, const char *fmt, ...)
{
#if OSD_ENABLED
    if (blink && (blink_phase < 2)) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    char buff[32+1]; // +1 for null-termination
    // note: format_string_for_osd() always terminates the string
    IGNORE_RETURN(format_string_for_osd(buff, sizeof(buff), check_option(AP_OSD::OPTION_DECIMAL_PACK), fmt, ap));
    va_end(ap);
    // buff is null terminated, this call should be safe without further checks
    write(x, y, buff);
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

void AP_OSD_Backend::init_symbol_set(uint8_t *lookup_table, const uint8_t size)
{
    memcpy(lookup_table, symbols, size);
}
