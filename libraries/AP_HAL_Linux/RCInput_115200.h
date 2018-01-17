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

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE

#include "RCInput.h"

namespace Linux {

class RCInput_115200 : public RCInput
{
public:
    RCInput_115200(const char *device) :
       device_path(device) {}
    void init() override;
    void _timer_tick(void) override;
    void set_device_path(const char *path);

private:
    const char *device_path;
    int32_t fd = -1;

    enum Decoders {
        DECODER_DSM=0,
        DECODER_ST24,
        DECODER_SUMD,
        DECODER_SRXL,
        DECODER_SYNC
    };
    enum Decoders decoder = DECODER_SYNC;

    uint8_t dsm_count;
    uint8_t st24_count;
    uint32_t last_input_ms;
};

}

#endif // CONFIG_HAL_BOARD_SUBTYPE
