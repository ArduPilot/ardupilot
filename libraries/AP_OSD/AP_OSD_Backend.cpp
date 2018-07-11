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
    if (blink && (blink_phase < 2)) {
        return;
    }
    char buff[32];
    va_list ap;
    va_start(ap, fmt);
    int res = hal.util->vsnprintf(buff, sizeof(buff), fmt, ap);
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
    if (res < int(sizeof(buff))) {
        write(x, y, buff);
    }
    va_end(ap);
}
