/*
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_AEROFC_V1

#include "RCOutput_Tap.h"

#include <termios.h>
#include <string.h>

namespace ap {

bool RCOutput_Tap::_uart_set_speed(int speed)
{
    if (_uart_fd < 0) {
        return false;
    }

    struct termios uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    tcgetattr(_uart_fd, &uart_config);

    // set baud rate
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        return false;
    }

    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
        return false;
    }

    return true;
}

}

#endif
