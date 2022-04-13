/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  show system information
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include "AP_Periph.h"

extern const AP_HAL::HAL &hal;

void AP_Periph_FW::dump_sysinfo(void)
{
    const int8_t info = g.sysinfo.get();
    if (info <= 0) {
        return;
    }
    ExpandingString s {};
    switch (info) {
    case 1:
        hal.util->timer_info(s);
        break;
    case 2:
        hal.util->mem_info(s);
        break;
#if HAL_UART_STATS_ENABLED
    case 3:
        hal.util->uart_info(s);
        break;
#endif
    case 4:
        hal.util->dma_info(s);
        break;
    case 5:
        hal.util->thread_info(s);
        break;
    }
    if (s.get_length() > 0) {
        char *ptr = nullptr;
        for (char *t = strtok_r(s.get_writeable_string(), "\n\r", &ptr);
             t;
             t = strtok_r(nullptr, "\n\r", &ptr)) {
            EXPECT_DELAY_MS(5);
            can_printf("%s", t);
            can_update();
        }
    }
    g.sysinfo.set_and_save(0);
}
