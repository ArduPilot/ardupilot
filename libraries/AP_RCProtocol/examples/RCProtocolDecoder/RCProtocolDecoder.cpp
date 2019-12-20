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
/*
  decode RC input using SITL on command line
 */

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_RCProtocol/AP_RCProtocol.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

static AP_RCProtocol *rcprot;

// change this to the device being tested.
const char *devicename = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10596TP-if00-port0";
const uint32_t baudrate = 115200;

static int fd;

// setup routine
void setup()
{
    // introduction
    hal.console->printf("ArduPilot RC protocol decoder\n");
    hal.scheduler->delay(100);

    fd = open(devicename, O_RDONLY|O_CLOEXEC);
    if (fd == -1) {
        perror(devicename);
        exit(1);
    }

    struct termios options;

    tcgetattr(fd, &options);
    cfsetspeed(&options, baudrate);
    tcgetattr(fd, &options);

    if (baudrate == 100000) {
        // SBUS: 100000bps, even parity, two stop bits
        options.c_cflag |= (CSTOPB | PARENB);
    } else {
        // DSM: 115200, one stop, no parity
        options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
        options.c_cflag |= CS8;

    }
    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    options.c_iflag &= ~(IXON|IXOFF|IXANY);
    options.c_oflag &= ~OPOST;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr");
        exit(1);
    }
    tcflush(fd, TCIOFLUSH);

    rcprot = new AP_RCProtocol();
    rcprot->init();
}

//Main loop where the action takes place
void loop()
{
    uint8_t buf[32];
    ssize_t ret = read(fd, buf, sizeof(buf));
    if (ret <= 0) {
        return;
    }
    for (uint8_t i=0; i<ret; i++) {
        rcprot->process_byte(buf[i], 115200);
        if (rcprot->new_input()) {
            uint8_t nchan = rcprot->num_channels();
            printf("%u: ", nchan);
            for (uint8_t j=0; j<nchan; j++) {
                uint16_t v = rcprot->read(j);
                printf("%04u ", v);
            }
            printf("\n");
        }
    }
}

#else
// dummy implementation
void setup() {}
void loop() {}

#endif // CONFIG_HAL_BOARD

AP_HAL_MAIN();
