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
#include <AP_HAL_ESP32/HAL_ESP32_Class.h>
#include "SdCard.h"

#include <stdint.h>
#include "esp_timer.h"

namespace AP_HAL
{

void panic(const char *errormsg, ...)
{
    va_list ap;

    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);

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
    return esp_timer_get_time();
}

uint64_t millis64()
{
    return micros64()/1000;
}

} // namespace AP_HAL

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    static const HAL_ESP32 hal;
    return hal;
}
