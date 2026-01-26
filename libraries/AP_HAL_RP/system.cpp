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
 */

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP_Class.h"
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/time.h"

namespace AP_HAL
{

void panic(const char *errormsg, ...)
{
#if 0
    va_list ap;

    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);
#endif
    while (1) {}
}

uint32_t micros()
{
    return micros64();
}

uint32_t millis()
{
    return millis64();
}

uint64_t micros64()
{
    return time_us_64();
}

uint64_t millis64()
{
    return (uint64_t)(time_us_64() / 1000ULL);
}

} // namespace AP_HAL
