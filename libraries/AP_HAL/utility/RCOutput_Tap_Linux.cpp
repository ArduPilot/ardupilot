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

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput_Tap.h"

#include <asm/termbits.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace ap {

bool RCOutput_Tap::_uart_set_speed(int speed)
{
    struct termios2 tc;

    if (_uart_fd < 0) {
        return false;
    }

    memset(&tc, 0, sizeof(tc));
    if (ioctl(_uart_fd, TCGETS2, &tc) == -1) {
        return false;
    }

    /* speed is configured by c_[io]speed */
    tc.c_cflag &= ~CBAUD;
    tc.c_cflag |= BOTHER;
    tc.c_ispeed = speed;
    tc.c_ospeed = speed;

    if (ioctl(_uart_fd, TCSETS2, &tc) == -1) {
        return false;
    }

    if (ioctl(_uart_fd, TCFLSH, TCIOFLUSH) == -1) {
        return false;
    }

    return true;
}

}

#endif
