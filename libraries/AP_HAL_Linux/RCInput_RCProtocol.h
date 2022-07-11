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
  RC input system that uses libraries/AP_RCProtocol with UART based inputs
  with either SBUS protocol or 115200 based protocols (or both)
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_VNAV

namespace Linux {

class RCInput_RCProtocol : public RCInput {
public:
    RCInput_RCProtocol(const char *dev_sbus, const char *dev_115200);
    void init() override;
    void _timer_tick(void) override;

private:
    int open_sbus(const char *path);
    int open_115200(const char *path);

    const char *dev_inverted;
    const char *dev_115200;

    int fd_inverted;
    int fd_115200;
    uint32_t last_frame_ms;
    bool inverted_is_115200;
};
};

#endif
