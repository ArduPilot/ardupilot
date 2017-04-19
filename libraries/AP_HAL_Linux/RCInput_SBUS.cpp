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
  this is a driver for SBUS input in Linux board using a UART. Note
  that it relies on kernel support for 100kbaud and on a uart inverted
  in hardware
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
#include "RCInput_SBUS.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

#define SBUS_FRAME_SIZE 25

void RCInput_SBUS::init()
{
    fd = open(device_path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd != -1) {
        printf("Opened SBUS input %s fd=%d\n", device_path, (int)fd);
        fflush(stdout);
        struct termios2 tio {};
        
        if (ioctl(fd, TCGETS2, &tio) != 0) {
            close(fd);
            fd = -1;
            return;
        }
        tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR
                         | IGNCR | ICRNL | IXON);
        tio.c_iflag |= (INPCK | IGNPAR);
        tio.c_oflag &= ~OPOST;
        tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
        // use BOTHER to specify speed directly in c_[io]speed member
        tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
        tio.c_ispeed = 100000;
        tio.c_ospeed = 100000;
        // see select() comment below
        tio.c_cc[VMIN] = SBUS_FRAME_SIZE;
        tio.c_cc[VTIME] = 0;
        if (ioctl(fd, TCSETS2, &tio) != 0) {
            close(fd);
            fd = -1;
            return;
        }
    }
}

void RCInput_SBUS::set_device_path(const char *path)
{
    device_path = path;
    printf("Set SBUS device path %s\n", path);
}

#define SBUS_DEBUG_LOG 0
#define SBUS_CAUSE_CORRUPTION 0

void RCInput_SBUS::_timer_tick(void)
{
    if (fd == -1) {
        return;
    }

    // read up to 10 frames at a time
    uint8_t bytes[SBUS_FRAME_SIZE*10];
    int nread;
    
    fd_set fds;
    struct timeval tv;
    
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    // as VMIN is SBUS_FRAME_SIZE the select won't return unless there is
    // at least SBUS_FRAME_SIZE bytes available
    if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
        return;
    }

#if SBUS_DEBUG_LOG
    static int logfd = -1;
    if (logfd == -1) {
        logfd = open("sbus.log", O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC, 0644);
    }
#endif

#if SBUS_CAUSE_CORRUPTION
    // deliberately lose bytes from the port
    static unsigned corruption_counter;
    if (corruption_counter++ % 1000 == 0) {
        uint8_t nn = corruption_counter/1000;
        int n2 = ::read(fd, bytes, nn);
        dprintf(logfd, "throw %u\n", (unsigned)n2);
    }
#endif

    // cope with there being a large number of pending bytes at
    // the start
    do {
        nread = ::read(fd, bytes, sizeof(bytes));
    } while (nread == sizeof(bytes));
    
    if (nread % SBUS_FRAME_SIZE != 0) {
        /*
          SBUS frames are 25 bytes long, and always start with
          0x0f, but there is no other framing information to
          prevent us getting out of sync. All we have are the
          timing guarantees
          
          In this case we have read a partial frame, or lost some
          bytes. A 25 bytes frame at 100000 baud takes 2.5ms. By
          delaying 3.5ms here we will get any remaining bytes for
          this frame. We shouldn't get the start of the next frame
          as frames happen at most every 7ms
          
          This strategy allows us to resync even if we lose
          bytes. It assumes an interframe gap of more than
          3.5ms. If SBUS is run at very high rate (like 300Hz)
          then this won't work
        */
        hal.scheduler->delay_microseconds(3500);
        int n2 = ::read(fd, bytes+nread, sizeof(bytes)-nread);
        if (n2 > 0) {
            nread += n2;
        }
    }
    if (nread % SBUS_FRAME_SIZE != 0) {
        // if we don't have a multuple of 25 then throw away the lot
#if SBUS_DEBUG_LOG
        dprintf(logfd, "discard %u\n", (unsigned)nread);
#endif
        return;
    }
    if (nread <= 0) {
        return;
    }
#if SBUS_DEBUG_LOG
    if (logfd != -1) {
        dprintf(logfd, "%06u %u: ", (unsigned)AP_HAL::millis(), (unsigned)nread);
        for (uint8_t i=0; i<nread; i++) {
            dprintf(logfd, "%02x ", (unsigned)bytes[i]);
        }
        dprintf(logfd, "\n");
    }
#endif
    // only process the last SBUS_FRAME_SIZE bytes. Only the latest
    // frame matters
    add_sbus_input(&bytes[nread-SBUS_FRAME_SIZE], SBUS_FRAME_SIZE);
}
#endif // CONFIG_HAL_BOARD_SUBTYPE

