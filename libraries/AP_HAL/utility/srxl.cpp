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
  SRXL protocol decoder, tested against AR7700 SRXL port
  Andrew Tridgell, September 2016
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "srxl.h"

/*
  there are other SRXL varients, but we need some sample data before accepting them
 */
#define SRXL_MAX_CHANNELS 20
#define SRXL_HEAD_MARKER  0xA5

#define SRXL_NUM_BYTES 18

static uint8_t buffer[SRXL_NUM_BYTES];
static uint8_t buflen;
static uint64_t last_data_us;
static uint16_t channels[SRXL_MAX_CHANNELS];
static uint16_t max_channels;

/*
  get SRXL crc16 for a set of bytes
 */
static uint16_t srxl_crc16(const uint8_t *bytes, uint8_t nbytes)
{
    uint16_t crc = 0;
    while (nbytes--) {
        int i;
	crc ^= (uint16_t)(*bytes++) << 8;

	for (i = 0; i < 8; i++) {
		crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
	}
    }
    return crc;
}


/*
  decode a SRXL packet
 */
int srxl_decode(uint64_t timestamp_us, uint8_t byte, uint8_t *num_values, uint16_t *values, uint16_t max_values, bool *failsafe_state)
{
    if (timestamp_us - last_data_us > 5000U || buflen == SRXL_NUM_BYTES) {
        // we've seen a frame gap
        if (buflen != 0) {
            buflen = 0;
        }
    }
    last_data_us = timestamp_us;
    buffer[buflen++] = byte;

    switch (buffer[0]) {
    case SRXL_HEAD_MARKER:
        break;

    default:
        // invalid packet, we don't support this format yet
        return 2;
    }

    uint8_t expected_bytes = SRXL_NUM_BYTES;
    if (buflen < expected_bytes) {
        // more bytes pending
        return 1;
    }

    uint16_t crc = srxl_crc16(buffer, buflen-2);
    uint64_t crc2 = buffer[buflen-1] | (buffer[buflen-2]<<8);
    if (crc != crc2) {
        // bad CRC
        return 4;
    }

    // up to 7 channel values per packet. Each channel value is 16
    // bits, with 11 bits of data and 4 bits of channel number. The
    // top bit indicates a special X-Plus channel
    for (uint8_t i=0; i<7; i++) {
        uint16_t b = buffer[i*2+2] << 8 | buffer[i*2+3];
        uint16_t c = b >> 11; // channel number
        int32_t v = b & 0x7FF;
        if (b & 0x8000) {
            //printf("bad data 0x%04x\n", b);
            // bad data
            return 2;
        } 
        if (c == 12) {
            // special handling for channel 12, this contains the XPlus channels
            // see http://www.deviationtx.com/forum/protocol-development/2088-18-channels-for-dsm2-dsmx?start=40
            //printf("c12: 0x%x %02x %02x\n", (unsigned)(b>>9), (unsigned)buffer[0], (unsigned)buffer[1]);
            v = (b & 0x1FF) << 2;
            c = 10 + ((b >> 9) & 0x7);
            if (buffer[1] & 1) {
                c += 4;
            }
#if 0
            printf("b=0x%04x v=%u c=%u b[1]=0x%02x\n",
                   (unsigned)b, (unsigned)v, (unsigned)c, (unsigned)buffer[1]);
#endif
        } else if (c > 12) {
            // invalid
            v = 0;
        }

        if (c < SRXL_MAX_CHANNELS) {
            v = (((v - 0x400) * 500) / 876) + 1500;
            channels[c] = v;
            if (c >= max_channels) {
                max_channels = c+1;
            }
        }
        
        //printf("%u:%u ", (unsigned)c, (unsigned)v);
    }
    //printf("\n");

    *num_values = max_channels;
    if (*num_values > max_values) {
        *num_values = max_values;
    }
    memcpy(values, channels, (*num_values)*2);

    // check failsafe bit, this goes low when connection to the
    // transmitter is lost
    *failsafe_state = ((buffer[1] & 2) == 0);
    
#if 0
    for (uint8_t i=0; i<buflen; i++) {
        printf("%02x ", (unsigned)buffer[i]);
    }
    printf("\n");
#endif

    buflen = 0;

    // success
    return 0;
}


#ifdef TEST_MAIN_PROGRAM
/*
  test harness for use under Linux with USB serial adapter
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>

static uint64_t micros64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)));
}

int main(int argc, const char *argv[])
{
    int fd = open(argv[1], O_RDONLY);
    if (fd == -1) {
        perror(argv[1]);
        exit(1);
    }

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
        perror("tcsetattr");
        exit(1);
    }
    tcflush(fd, TCIOFLUSH);
    
    while (true) {
        uint8_t b;
        uint8_t num_values = 0;
        uint16_t values[20];
        bool failsafe_state;
        fd_set fds;
        struct timeval tv;
    
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        tv.tv_sec = 1;
        tv.tv_usec = 0;

        // check if any bytes are available
        if (select(fd+1, &fds, NULL, NULL, &tv) != 1) {
            break;
        }
        
        if (read(fd, &b, 1) != 1) {
            break;
        }

        if (srxl_decode(micros64(), b, &num_values, values, sizeof(values)/sizeof(values[0]), &failsafe_state) == 0) {
#if 1
            printf("%u: ", num_values);
            for (uint8_t i=0; i<num_values; i++) {
                printf("%u:%4u ", i+1, values[i]);
            }
            printf("%s\n", failsafe_state?"FAIL":"OK");
#endif
        }
    }
}
#endif


#ifdef TEST_HEX_PROGRAM
/*
  test harness for use under Linux with hex dump in a file
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>

int main(int argc, const char *argv[])
{
    FILE *f = fopen(argv[1], "r");
    if (!f) {
        perror(argv[1]);
        exit(1);
    }

    uint64_t t=0;
    while (true) {
        uint8_t b;
        uint8_t num_values = 0;
        uint16_t values[20];
        bool failsafe_state;

        unsigned u[18];
        if (fscanf(f, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                   &u[0], &u[1], &u[2], &u[3], &u[4], &u[5], &u[6], &u[7], &u[8], &u[9], 
                   &u[10], &u[11], &u[12], &u[13], &u[14], &u[15], &u[16], &u[17]) != 18) {
            break;
        }
        t += 11000;

        for (uint8_t i=0; i<18; i++) {
            b = u[i];

            if (srxl_decode(t, b, &num_values, values, sizeof(values)/sizeof(values[0]), &failsafe_state) == 0) {
#if 1
                printf("%u: ", num_values);
                for (uint8_t i=0; i<num_values; i++) {
                    printf("%u:%4u ", i+1, values[i]);
                }
                printf("%s\n", failsafe_state?"FAIL":"OK");
#endif
            }
        }
    }
}
#endif
