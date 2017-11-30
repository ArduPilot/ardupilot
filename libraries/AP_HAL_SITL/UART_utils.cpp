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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(__CYGWIN__)

#include "UARTDriver.h"

#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

bool HALSITL::UARTDriver::set_speed(int speed)
{
    struct termios2 tc;

    if (_fd < 0) {
        return false;
    }

    memset(&tc, 0, sizeof(tc));
    if (ioctl(_fd, TCGETS2, &tc) == -1) {
        return false;
    }

    /* speed is configured by c_[io]speed */
    tc.c_cflag &= ~CBAUD;
    tc.c_cflag |= BOTHER;
    tc.c_ispeed = speed;
    tc.c_ospeed = speed;

    if (ioctl(_fd, TCSETS2, &tc) == -1) {
        return false;
    }

    if (ioctl(_fd, TCFLSH, TCIOFLUSH) == -1) {
        return false;
    }

    return true;
}

void HALSITL::UARTDriver::configure_parity(uint8_t v) {
    if (_fd < 0) {
        return;
    }
    struct termios2 tc;

    memset(&tc, 0, sizeof(tc));
    if (ioctl(_fd, TCGETS2, &tc) == -1) {
        return;
    }
    if (v != 0) {
        // enable parity
        tc.c_cflag |= PARENB;
        if (v == 1) {
            tc.c_cflag |= PARODD;
        } else {
            tc.c_cflag &= ~PARODD;
        }
    }
    else {
        // disable parity
        tc.c_cflag &= ~PARENB;
    }
    if (ioctl(_fd, TCSETS2, &tc) == -1) {
        return;
    }

    if (ioctl(_fd, TCFLSH, TCIOFLUSH) == -1) {
        return;
    }
}

void HALSITL::UARTDriver::set_stop_bits(int n) {
    if (_fd < 0) {
        return;
    }
    struct termios2 tc;

    memset(&tc, 0, sizeof(tc));
    if (ioctl(_fd, TCGETS2, &tc) == -1) {
        return;
    }
    if (n > 1) tc.c_cflag |= CSTOPB;
    else tc.c_cflag &= ~CSTOPB;

    if (ioctl(_fd, TCSETS2, &tc) == -1) {
        return;
    }

    if (ioctl(_fd, TCFLSH, TCIOFLUSH) == -1) {
        return;
    }
}

#endif
