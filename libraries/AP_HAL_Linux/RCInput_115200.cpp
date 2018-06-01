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
  this is a driver for R/C input protocols that use a 115200 baud
  non-inverted 8-bit protocol. That includes:
    - DSM
    - SUMD
    - ST24
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
#include "RCInput_115200.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <termios.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_115200::init()
{
    fd = open(device_path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd != -1) {
        struct termios options;

        tcgetattr(fd, &options);

        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
        options.c_cflag |= CS8;

        options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
        options.c_iflag &= ~(IXON|IXOFF|IXANY);
        options.c_oflag &= ~OPOST;

        if (tcsetattr(fd, TCSANOW, &options) != 0) {
            AP_HAL::panic("RCInput_UART: error configuring device: %s",
                          strerror(errno));
        }

        tcflush(fd, TCIOFLUSH);
    }
}

void RCInput_115200::set_device_path(const char *path)
{
    device_path = path;
}

void RCInput_115200::_timer_tick(void)
{
    if (fd == -1) {
        return;
    }

    // read up to 256 bytes at a time
    uint8_t bytes[256];
    int nread;
    
    fd_set fds;
    struct timeval tv;
    
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    // check if any bytes are available
    if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
        return;
    }

    // cope with there being a large number of pending bytes at
    // the start and discard them
    do {
        nread = ::read(fd, bytes, sizeof(bytes));
    } while (nread == sizeof(bytes));

    if (nread <= 0) {
        return;
    }
    bool got_frame = false;
    
    if (decoder == DECODER_SYNC ||
        decoder == DECODER_SRXL) {
        // try srxl first as it has a 16 bit CRC
        if (add_srxl_input(bytes, nread)) {
            // lock immediately
            decoder = DECODER_SRXL;
            got_frame = true;
        }
    }

    if (decoder == DECODER_SYNC ||
        decoder == DECODER_SUMD) {
        // SUMD also has a 16 bit CRC
        if (add_sumd_input(bytes, nread)) {
            // lock immediately
            decoder = DECODER_SUMD;
            got_frame = true;
        }
    }

    if (decoder == DECODER_SYNC ||
        decoder == DECODER_DSM) {
        // process as DSM
        if (add_dsm_input(bytes, nread)) {
            dsm_count++;
            if (dsm_count == 10) {
                // we're confident
                decoder = DECODER_DSM;
            }
            got_frame = true;
        }
    }

    if (decoder == DECODER_SYNC ||
        decoder == DECODER_ST24) {
        // process as st24
        if (add_st24_input(bytes, nread)) {
            st24_count++;
            if (st24_count == 10) {
                // we're confident
                decoder = DECODER_ST24;
            }
            got_frame = true;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (got_frame) {
        last_input_ms = now;
    } else if (now - last_input_ms > 1000 && decoder != DECODER_SYNC) {
        // start search again
        decoder = DECODER_SYNC;
        dsm_count = 0;
        st24_count = 0;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

