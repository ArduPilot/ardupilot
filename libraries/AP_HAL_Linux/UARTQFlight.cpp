/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  This is a UART driver for the QFLIGHT port. Actual UART output
  happens via RPC calls. See the qflight/ subdirectory for details
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include <stdio.h>
#include <unistd.h>

#include "UARTQFlight.h"

#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>
#include <stdio.h>

QFLIGHTDevice::QFLIGHTDevice(const char *_device_path)
{
    device_path = _device_path;
    if (strncmp(device_path, "qflight:", 8) == 0) {
        device_path += 8;
    }
}

QFLIGHTDevice::~QFLIGHTDevice()
{
    close();
}

bool QFLIGHTDevice::close()
{
    if (fd != -1) {
        if (qflight_UART_close(fd) != 0) {
            return false;
        }
    }

    fd = -1;
    return true;
}

bool QFLIGHTDevice::open()
{
    int ret = qflight_UART_open(device_path, &fd);

    if (ret != 0 || fd == -1) {
        printf("Failed to open UART device %s ret=%d fd=%d\n",
               device_path, ret, (int)fd);
        return false;
    }
    printf("opened QFLIGHT UART device %s ret=%d fd=%d\n",
           device_path, ret, (int)fd);
    return true;
}

ssize_t QFLIGHTDevice::read(uint8_t *buf, uint16_t n)
{
    int32_t nread = 0;
    int ret = qflight_UART_read(fd, buf, n, &nread);
    if (ret != 0) {
        return 0;
    }
    return nread;
}

ssize_t QFLIGHTDevice::write(const uint8_t *buf, uint16_t n)
{
    int32_t nwritten = 0;
    int ret = qflight_UART_write(fd, buf, n, &nwritten);
    if (ret != 0) {
        return 0;
    }
    return nwritten;
}

void QFLIGHTDevice::set_blocking(bool blocking)
{
    // no implementation yet
}

void QFLIGHTDevice::set_speed(uint32_t baudrate)
{
    qflight_UART_set_baudrate(fd, baudrate);
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

