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
 * decode RC input using SITL on command line
 *
 * To use this as an RC protocol decoder for SITL with a real transmitter:
 *
 * 1. Compile using Linux - SITL has timing that is too variable
 * 2. Connect an RX device to an FTDI adapter
 * 3. Set the FTDI serial port to 115k baud, 8N1
 * 4. Set the FTDI serial BM options to 1ms latency (very important)
 * 5. Set the tty using: stty -F <device> raw 115200
 * 6. Run this sketch providing the serial device name as an argument. RC values will be automatically written to the SITL RC port
 */

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_HAL/utility/Socket.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Math/AP_Math.h>
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

class RC_Channel_RC : public RC_Channel
{
};

class RC_Channels_RC : public RC_Channels
{
public:
    RC_Channel *channel(uint8_t chan) override {
        return &obj_channels[chan];
    }

    RC_Channel_RC obj_channels[NUM_RC_CHANNELS];
private:
    int8_t flight_mode_channel_number() const override { return -1; };
};

#define RC_CHANNELS_SUBCLASS RC_Channels_RC
#define RC_CHANNEL_SUBCLASS RC_Channel_RC

#include <RC_Channel/RC_Channels_VarInfo.h>

RC_Channels_RC _rc;
SocketAPM rc_socket{true};

// change this to the device being tested.
const char *devicename = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10596TP-if00-port0";
const uint32_t baudrate = 115200;

static int fd;
static uint16_t chan[16];
static uint8_t nchan = 0;

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

    rcprot = &AP::RC();
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    rcprot->init();
#endif
    // proxy to SITL's rcin port
    rc_socket.connect("0.0.0.0", 5501);
}

//Main loop where the action takes place
void loop()
{
    uint8_t buf[62]; // lowest USB buffer size is 62 user bytes

    ssize_t ret = read(fd, buf, sizeof(buf));

    for (uint8_t i=0; i<ret; i++) {
        rcprot->process_byte(buf[i], 115200);
        if (rcprot->new_input()) {
            nchan = MIN(rcprot->num_channels(), 16);
            rcprot->read(chan, nchan);
            printf("%u: ", nchan);
            for (uint8_t j=0; j<nchan; j++) {
                // normalize data for SITL
                chan[j] = constrain_int16(chan[j], 1100, 1900);
                printf("%04u ", chan[j]);
            }
            printf("\n");

        }
    }
    // SITL expects either 8 or 16 channels, send whether we got data or not
    if (nchan <= 8) {
        rc_socket.send(chan, 8 * 2);
    } else {
        rc_socket.send(chan, 16 * 2);
    }
}

#else
// dummy implementation
void setup() {}
void loop() {}

#endif // CONFIG_HAL_BOARD

AP_HAL::HAL::FunCallbacks callbacks(setup, loop);
extern "C" {
int main(int argc, char* const argv[]);
int main(int argc, char* const argv[]) {
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    if (argc > 1) {
        devicename = argv[1];
    }
#endif
    hal.run(argc, argv, &callbacks);
    return 0;
}
}
