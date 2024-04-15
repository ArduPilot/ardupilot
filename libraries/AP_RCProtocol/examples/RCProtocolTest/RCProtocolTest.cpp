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
  test for RC input protocols
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <stdio.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

void setup();
void loop();

class RC_Channel_Example : public RC_Channel {};

class RC_Channels_Example : public RC_Channels
{
public:
    RC_Channel_Example obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Example *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:
    int8_t flight_mode_channel_number() const override { return 5; }
};

#define RC_CHANNELS_SUBCLASS RC_Channels_Example
#define RC_CHANNEL_SUBCLASS RC_Channel_Example

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Example rchannels;
static AP_SerialManager serial_manager;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_VideoTX vtx; // for set_vtx functions

static AP_RCProtocol *rcprot;

static uint32_t test_count;
static uint32_t test_failures;

static void delay_ms(uint32_t ms)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    hal.scheduler->stop_clock(AP_HAL::micros64()+ms*1000);
#else
    hal.scheduler->delay(ms);
#endif
}

// setup routine
void setup()
{
    // introduction
    hal.console->printf("ArduPilot RC protocol test\n");
    delay_ms(100);
}

static bool check_result(const char *name, bool bytes, const uint16_t *values, uint8_t nvalues)
{
    char label[20];
    snprintf(label, 20, "%s(%s)", name, bytes?"bytes":"pulses");
    const bool have_input = rcprot->new_input();
    if (values == nullptr) {
        if (have_input) {
            printf("%s: got input, none expected\n", label);
            return false;
        } else {
            printf("%s: OK (no input)\n", label);
            return true;
        }
    }
    if (!have_input) {
        printf("%s: No new input\n", label);
        test_failures++;
        return false;
    }
    const char *pname = rcprot->protocol_name();
    if (strncmp(pname, name, strlen(pname)) != 0) {
        printf("%s: wrong protocol detected %s\n", label, rcprot->protocol_name());
        test_failures++;
        return false;
    }
    uint8_t n = rcprot->num_channels();
    if (n != nvalues) {
        printf("%s: wrong number of channels %u should be %u\n", label, n, nvalues);
        test_failures++;
        return false;
    }
    for (uint8_t i=0; i<n; i++) {
        uint16_t v = rcprot->read(i);
        if (values[i] != v) {
            printf("%s: chan %u wrong value %u should be %u\n", label, i+1, v, values[i]);
            test_failures++;
            return false;
        }
    }
    printf("%s OK\n", label);
    return true;
}

/*
  test a byte protocol handler
 */
static bool test_byte_protocol(const char *name, uint32_t baudrate,
                               const uint8_t *bytes, uint8_t nbytes,
                               const uint16_t *values, uint8_t nvalues,
                               uint8_t repeats,
                               uint8_t pause_at)
{
    bool ret = true;
    for (uint8_t repeat=0; repeat<repeats+4; repeat++) {
        for (uint8_t i=0; i<nbytes; i++) {
            if (pause_at > 0 && i > 0 && ((i % pause_at) == 0)) {
                delay_ms(10);
            }
            rcprot->process_byte(bytes[i], baudrate);
        }
        delay_ms(10);
        if (repeat > repeats) {
            ret &= check_result(name, true, values, nvalues);
        }
    }
    return ret;
}

static void send_bit(uint8_t bit, uint32_t baudrate, bool inverted)
{
    static uint16_t bits_0, bits_1;
    if (!inverted) {
        // inverted serial idles low
        bit = !bit;
    }
    if (bit == 0) {
        if (bits_1 > 0) {
            uint32_t w0=(bits_0 * (uint32_t)1000000) / baudrate;
            uint32_t w1=(bits_1 * (uint32_t)1000000) / baudrate;
            //printf("%u %u\n", w0, w1);
            rcprot->process_pulse(w0, w1);
            bits_0 = 1;
            bits_1 = 0;
        } else {
            bits_0++;
        }
    } else {
        bits_1++;
    }
}

/*
  call process_pulse() for a byte of input
 */
static void send_byte(uint8_t b, uint32_t baudrate, bool inverted)
{
    send_bit(0, baudrate, inverted); // start bit
    uint8_t parity = 0;
    for (uint8_t i=0; i<8; i++) {
        uint8_t bit = (b & (1U<<i))?1:0;
        send_bit(bit, baudrate, inverted);
        if (bit) {
            parity = !parity;
        }
    }
    if (baudrate == 100000) {
        // assume SBUS, send parity
        send_bit(parity, baudrate, inverted);
    }
    send_bit(1, baudrate, inverted); // stop bit
    if (baudrate == 100000) {
        send_bit(1, baudrate, inverted); // 2nd stop bit
    }
}

/*
  add a gap in bits
 */
static void send_pause(uint8_t b, uint32_t baudrate, uint32_t pause_us, bool inverted)
{
    uint32_t nbits = pause_us * 1e6 / baudrate;
    for (uint32_t i=0; i<nbits; i++) {
        send_bit(b, baudrate, inverted);
    }
}

/*
  test a byte protocol handler
 */
static bool test_pulse_protocol(const char *name, uint32_t baudrate,
                                const uint8_t *bytes, uint8_t nbytes,
                                const uint16_t *values, uint8_t nvalues,
                                uint8_t repeats, uint8_t pause_at,
                                bool inverted)
{
    bool ret = true;
    for (uint8_t repeat=0; repeat<repeats+4; repeat++) {
        send_pause(1, baudrate, 6000, inverted);
        for (uint8_t i=0; i<nbytes; i++) {
            if (pause_at > 0 && i > 0 && ((i % pause_at) == 0)) {
                send_pause(1, baudrate, 10000, inverted);
            }
            send_byte(bytes[i], baudrate, inverted);
        }
        send_pause(1, baudrate, 6000, inverted);
        if (repeat > repeats) {
            ret &= check_result(name, false, values, nvalues);
        }
    }
    return ret;
}

/*
  test a protocol handler
 */
static bool test_protocol(const char *name, uint32_t baudrate,
                          const uint8_t *bytes, uint8_t nbytes,
                          const uint16_t *values, uint8_t nvalues,
                          uint8_t repeats=1,
                          int8_t pause_at=0,
                          bool inverted=false)
{
    bool ret = true;
    rcprot = new AP_RCProtocol();
    rcprot->init();

    ret &= test_byte_protocol(name, baudrate, bytes, nbytes, values, nvalues, repeats, pause_at);
    delete rcprot;

    rcprot = new AP_RCProtocol();
    rcprot->init();
    ret &= test_pulse_protocol(name, baudrate, bytes, nbytes, values, nvalues, repeats, pause_at, inverted);
    delete rcprot;

    return ret;
}

/*
  test a protocol handler where we only expected byte input to work
 */
static bool test_protocol_bytesonly(const char *name, uint32_t baudrate,
                                    const uint8_t *bytes, uint8_t nbytes,
                                    const uint16_t *values, uint8_t nvalues,
                                    uint8_t repeats=1,
                                    int8_t pause_at=0,
                                    bool inverted=false)
{
    bool ret = true;
    rcprot = new AP_RCProtocol();
    rcprot->init();

    ret &= test_byte_protocol(name, baudrate, bytes, nbytes, values, nvalues, repeats, pause_at);
    delete rcprot;

    rcprot = new AP_RCProtocol();
    rcprot->init();
    ret &= test_pulse_protocol(name, baudrate, bytes, nbytes, nullptr, 0, repeats, pause_at, inverted);
    delete rcprot;

    return ret;
}

/*
  test with random data
 */
static void test_random(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const uint32_t baudrates[] = { 115200, 100000, 416666, 420000 };
    const uint32_t test_bytes = 1000000;
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd == -1) {
        printf("Can't open /dev/urandom\n");
        return;
    }
    uint8_t *buf = (uint8_t *)malloc(test_bytes);
    for (const auto b : baudrates) {
        printf("Testing random with baud %u\n", unsigned(b));
        rcprot = new AP_RCProtocol();
        rcprot->init();
        if (::read(fd, buf, test_bytes) != test_bytes) {
            printf("Failed to read from /dev/urandom\n");
            break;
        }
        for (uint32_t i=0; i<test_bytes; i++) {
            rcprot->process_byte(buf[i], b);
        }
        delete rcprot;
        rcprot = nullptr;
    }
    free(buf);
    close(fd);
#endif
}

//Main loop where the action takes place
#pragma GCC diagnostic error "-Wframe-larger-than=2000"
void loop()
{
    const uint8_t srxl_bytes[] = { 0xa5, 0x03, 0x0c, 0x04, 0x2f, 0x6c, 0x10, 0xb4, 0x26,
                                   0x16, 0x34, 0x01, 0x04, 0x76, 0x1c, 0x40, 0xf5, 0x3b };
    const uint16_t srxl_output[] = { 1567, 1502, 1019, 1536, 1804, 2000, 1500 };

    const uint8_t sbus_bytes[] = {0x0F, 0x4C, 0x1C, 0x5F, 0x32, 0x34, 0x38, 0xDD, 0x89,
                                  0x83, 0x0F, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint16_t sbus_output[] = {1562, 1496, 1000, 1531, 1806, 2006, 1495, 1495, 875,
                                    875, 875, 875, 875, 875, 875, 875};

    const uint8_t dsm_bytes[] = {0x00, 0xab, 0x00, 0xae, 0x08, 0xbf, 0x10, 0xd0, 0x18,
                                 0xe1, 0x20, 0xf2, 0x29, 0x03, 0x31, 0x14, 0x00, 0xab,
                                 0x39, 0x25, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff};
    const uint16_t dsm_output[] = {1010, 1020, 1000, 1030, 1040, 1050, 1060, 1070};

    // DSMX_2048_11MS
    const uint8_t dsm_bytes2[] = {0x00, 0xb2, 0x80, 0x94, 0x3c, 0x02, 0x1b, 0xfe,
                                  0x44, 0x00, 0x4c, 0x00, 0x5c, 0x00, 0xff, 0xff,
                                  0x00, 0xb2, 0x0c, 0x03, 0x2e, 0xaa, 0x14, 0x00,
                                  0x21, 0x56, 0x34, 0x02, 0x54, 0x00, 0xff, 0xff };

    const uint16_t dsm_output2[] = {1501, 1500, 985, 1499, 1099, 1901, 1501, 1501, 1500, 1500, 1500, 1500};

    // DSMX_2048_11MS, from genuine spektrum satellite, 12 channels
    const uint8_t dsm_bytes3[] = {0x00, 0x00, 0x81, 0x56, 0x39, 0x50, 0x1C, 0x06,
                                  0x44, 0x00, 0x4c, 0x00, 0x5c, 0x00, 0xff, 0xff,
                                  0x00, 0x00, 0x0c, 0x06, 0x2b, 0x32, 0x14, 0x06,
                                  0x21, 0x96, 0x31, 0x50, 0x54, 0x00, 0xff, 0xff };

    const uint16_t dsm_output3[] = {1503, 1503, 1099, 1503, 1137, 1379, 1096, 1096, 1500, 1500, 1500, 1500};

    // DSMX_2048_22MS, from genuine spektrum satellite, 12 channels
    const uint8_t dsm_bytes4[] = {0x00, 0x5a, 0x81, 0x7a, 0x39, 0x50, 0x1C, 0x06,
                                  0x44, 0x00, 0x4c, 0x00, 0x5c, 0x00, 0xff, 0xff,
                                  0x00, 0x5a, 0x0c, 0x06, 0x2b, 0x32, 0x14, 0x06,
                                  0x21, 0x96, 0x31, 0x50, 0x54, 0x00, 0xff, 0xff };

    const uint16_t dsm_output4[] = {1503, 1503, 1120, 1503, 1137, 1379, 1096, 1096, 1500, 1500, 1500, 1500};

    const uint8_t dsm_bytes5[] = {0x03, 0xB2, 0x05, 0xFE, 0x17, 0x55, 0x13, 0x55,
                                  0x09, 0xFC, 0x18, 0xAB, 0x00, 0x56, 0x0D, 0xFD};

    const uint16_t dsm_output5[] = {1498, 1496, 999, 1497, 1901, 1901, 1099};

    // DSMX 22ms D6G3 and SPM4648 autobound
    const uint8_t dsmx22ms_bytes[] = {
        0x00, 0xB2, 0x0C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x25, 0xF8, 0x34, 0x00, 0x54, 0x00, 0xFF, 0xFF,
        0x00, 0xB2, 0x81, 0x50, 0x3C, 0x00, 0x1B, 0xFD,
        0x44, 0x00, 0x4C, 0x00, 0x5C, 0x00, 0xFF, 0xFF
    };
    const uint16_t dsmx22ms_output[] = {
        1500, 1500, 1096, 1499, 1796, 1099, 1500, 1500, 1500, 1500, 1500, 1500
    };

    // DSMX 22ms D6G3 and SPM4648 autobound VTX frame Ch1, B1, Pw25, Race
    const uint8_t dsmx22ms_vtx_bytes[] = {
        // two normal frames to satisfy the format guesser
        0x00, 0xB2, 0x0C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x25, 0xF8, 0x34, 0x00, 0x54, 0x00, 0xFF, 0xFF,
        0x00, 0xB2, 0x81, 0x50, 0x3C, 0x00, 0x1B, 0xFD,
        0x44, 0x00, 0x4C, 0x00, 0x5C, 0x00, 0xFF, 0xFF,
        // This is channels 1, 5, 2, 4, 6
        0x00, 0xB2, 0x0C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x25, 0xF8, 0x34, 0x00, 0xE0, 0x00, 0xE0, 0x0A
    };
    const uint16_t dsmx22ms_vtx_output[] = {
        1500, 1500, 1096, 1499, 1796, 1099, 1500, 1500, 1500, 1500, 1500, 1500
    };
    // DSMX 11ms D6G3 and SPM4648 autobound
    const uint8_t dsmx11ms_bytes[] = {
        0x01, 0xB2, 0x0C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x1B, 0xFC, 0x25, 0xF8, 0x44, 0x00, 0x4C, 0x00,
        0x01, 0xB2, 0x8C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x1B, 0xFC, 0x01, 0x50, 0x3C, 0x00, 0x34, 0x00
    };
    const uint16_t dsmx11ms_output[] = {
        1500, 1500, 1096, 1498, 1796, 1099, 1500, 1500, 1500, 1500
    };

    // DSMX 11ms D6G3 and SPM4648 autobound VTX frame Ch1, B1, Pw25, Race
    const uint8_t dsmx11ms_vtx_bytes[] = {
        0x01, 0xB2, 0x0C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x1B, 0xFD, 0x25, 0xF8, 0x44, 0x00, 0x4C, 0x00,
        0x01, 0xB2, 0x8C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x1B, 0xFD, 0x01, 0x50, 0x3C, 0x00, 0x34, 0x00,
        0x00, 0xB2, 0x0C, 0x00, 0x29, 0x56, 0x14, 0x00,
        0x1B, 0xFD, 0x25, 0xF8, 0xE0, 0x00, 0xE0, 0x0A
    };
    const uint16_t dsmx11ms_vtx_output[] = {
        1500, 1500, 1096, 1499, 1796, 1099, 1500, 1500, 1500, 1500
    };

    const uint8_t sumd_bytes[] = {0xA8, 0x01, 0x08, 0x2F, 0x50, 0x31, 0xE8, 0x21, 0xA0,
                                  0x2F, 0x50, 0x22, 0x60, 0x22, 0x60, 0x2E, 0xE0, 0x2E,
                                  0xE0, 0x87, 0xC6};

    const uint8_t sumd_bytes2[] = {0xA8, 0x01, 0x0C, 0x22, 0x60, 0x2F, 0x60, 0x2E, 0xE0, 0x2E, 0xE0, 0x3B,
                                  0x60, 0x3B, 0x60, 0x3B, 0x60, 0x3B, 0x60, 0x3B, 0x60, 0x3B, 0x60, 0x3B, 0x60, 0x3B,
                                  0x60, 0x17, 0x02};

    const uint8_t sumd_bytes3[] = {0xA8, 0x01, 0x10, 0x1F, 0x40, 0x2E, 0xE8, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0,
                                   0x2E, 0xE0, 0x2E, 0xE0, 0x22, 0x60, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E,
                                   0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x20, 0x4F, 0x10};

    const uint16_t sumd_output[] = {1597, 1076, 1514, 1514, 1100, 1100, 1500, 1500};
    const uint16_t sumd_output2[] = {1516, 1500, 1100, 1500, 1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900};
    const uint16_t sumd_output3[] = {1501, 1500, 1000, 1500, 1500, 1500, 1500, 1100, 1500, 1500, 1500, 1500, 0, 0, 0, 1892};

    const uint8_t ibus_bytes[] = {0x20, 0x40, 0xdc, 0x05, 0xdc, 0x05, 0xe8, 0x03, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0x47, 0xf3};
    const uint16_t ibus_output[] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

    const uint8_t fport_bytes[] = {0x7e, 0x19, 0x00, 0xe7, 0x3b, 0xdf, 0x5a, 0xce,
                                   0x07, 0x10, 0x75, 0x49, 0x9c, 0x15, 0xe0, 0x03,
                                   0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f,
                                   0x7c, 0x00, 0x38, 0xfa, 0x7e};

    const uint16_t fport_output[] = {1499, 1499, 1101, 1499, 1035, 1341, 2006, 982, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495};

    const uint8_t fport2_16ch_bytes[] = {0x18, 0xff, 
                                    0xac, 0x00, 0x5f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, // 8ch on 11 bits
                                    0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, // 8ch on 11 bits
                                    0x00, 0x5e, 0x98};
    const uint16_t fport2_16ch_output[] = {982, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495, 1495};

    const uint8_t fport2_24ch_bytes[] = {0x23, 0xff,
                                    0xe0, 0x03, 0xdf, 0x2c, 0xc2, 0xc7, 0x0a, 0xf0, 0xb1, 0x82, 0x15, // 8ch on 11 bits
                                    0xe0, 0x9b, 0x38, 0x2b, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, // 8ch on 11 bits
                                    0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, // 8ch on 11 bits
                                    0x00, 0x5b, 0x02 };

    // we only decode up to 18ch
    const uint16_t fport2_24ch_output[] = {1495, 1495, 986, 1495, 982, 1495, 982, 982, 1495, 2006, 982, 1495, 1495, 1495, 1495, 1495, 1495, 1495};

    const uint8_t crsf_bytes[] = {0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x0F, 0x6E };
    const uint16_t crsf_output[] = {1501, 1500, 989, 1497, 1873, 1136, 2011, 988, 988, 988, 988, 2011, 0, 0, 0, 0, 0, 0};
    // CRSF partial frame followed by full frame
    const uint8_t crsf_bad_bytes1[] = {0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x0F, 0x6E };
    const uint16_t crsf_bad_output1[] = {1501, 1500, 989, 1497, 1873, 1136, 2011, 988, 988, 988, 988, 2011, 0, 0, 0, 0, 0, 0};
    // CRSF full frame with bad CRC followed by full frame
    const uint8_t crsf_bad_bytes2[] = {0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x0F, 0x6F,
                                  0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x0F, 0x6E };
    const uint16_t crsf_bad_output2[] = {1501, 1500, 989, 1497, 1873, 1136, 2011, 988, 988, 988, 988, 2011, 0, 0, 0, 0, 0, 0};

    // CRSF with lots of start markers followed by full frame
    const uint8_t crsf_bad_bytes3[] = {0xC8, 0x14, 0xC8, 0x14, 0xC8, 0x0C, 0xA0,
                                   0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                   0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x0F, 0x6E, };
    const uint16_t crsf_bad_output3[] = {1501, 1500, 989, 1497, 1873, 1136, 2011, 988, 988, 988, 988, 2011, 0, 0, 0, 0, 0, 0};

    // CRSF with a partial frame followed by a full frame
    const uint8_t crsf_bad_bytes4[] = {
                                   0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                   0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
                                   0xC8, 0x14, 0x17, 0x20, 0x03, 0x0C, 0xA0, 0x00, 0xF6, 0xB7, 0x6E, 0x94, 0xFC,
                                   0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x0F, 0x6E };
    const uint16_t crsf_bad_output4[] = {1501, 1500, 989, 1497, 1873, 1136, 2011, 988, 988, 988, 988, 2011, 0, 0, 0, 0, 0, 0};
    

    test_protocol("SRXL", 115200, srxl_bytes, sizeof(srxl_bytes), srxl_output, ARRAY_SIZE(srxl_output), 1);
    test_protocol("SUMD", 115200, sumd_bytes, sizeof(sumd_bytes), sumd_output, ARRAY_SIZE(sumd_output), 1);
    test_protocol("SUMD2", 115200, sumd_bytes2, sizeof(sumd_bytes2), sumd_output2, ARRAY_SIZE(sumd_output2), 1);
    test_protocol("SUMD3", 115200, sumd_bytes3, sizeof(sumd_bytes3), sumd_output3, ARRAY_SIZE(sumd_output3), 1);
    test_protocol("IBUS", 115200, ibus_bytes, sizeof(ibus_bytes), ibus_output, ARRAY_SIZE(ibus_output), 1);

    // SBUS needs 3 repeats to pass the RCProtocol 3 frames test
    test_protocol("SBUS", 100000, sbus_bytes, sizeof(sbus_bytes), sbus_output, ARRAY_SIZE(sbus_output), 3, 0, true);

    // CRSF needs 3 repeats to pass the RCProtocol 3 frames test
    test_protocol_bytesonly("CRSF", 416666, crsf_bytes, sizeof(crsf_bytes), crsf_output, ARRAY_SIZE(crsf_output), 3, 0, true);
    test_protocol_bytesonly("CRSF2", 416666, crsf_bad_bytes1, sizeof(crsf_bad_bytes1), crsf_bad_output1, ARRAY_SIZE(crsf_bad_output1), 3, 0, true);
    test_protocol_bytesonly("CRSF3", 416666, crsf_bad_bytes2, sizeof(crsf_bad_bytes2), crsf_bad_output2, ARRAY_SIZE(crsf_bad_output2), 3, 0, true);
    test_protocol_bytesonly("CRSF4", 416666, crsf_bad_bytes3, sizeof(crsf_bad_bytes3), crsf_bad_output3, ARRAY_SIZE(crsf_bad_output3), 3, 0, true);
    test_protocol_bytesonly("CRSF5", 416666, crsf_bad_bytes4, sizeof(crsf_bad_bytes4), crsf_bad_output4, ARRAY_SIZE(crsf_bad_output4), 3, 0, true);

    // DSM needs 8 repeats, 5 to guess the format, then 3 to pass the RCProtocol 3 frames test
    test_protocol("DSM1", 115200, dsm_bytes,  sizeof(dsm_bytes),  dsm_output,  ARRAY_SIZE(dsm_output), 9);
    test_protocol("DSM2", 115200, dsm_bytes2, sizeof(dsm_bytes2), dsm_output2, ARRAY_SIZE(dsm_output2), 9, 16);
    test_protocol("DSM3", 115200, dsm_bytes3, sizeof(dsm_bytes3), dsm_output3, ARRAY_SIZE(dsm_output3), 9, 16);
    test_protocol("DSM4", 115200, dsm_bytes4, sizeof(dsm_bytes4), dsm_output4, ARRAY_SIZE(dsm_output4), 9, 16);
    test_protocol("DSM5", 115200, dsm_bytes5, sizeof(dsm_bytes5), dsm_output5, ARRAY_SIZE(dsm_output5), 9);
    test_protocol("DSMX22", 115200, dsmx22ms_bytes, sizeof(dsmx22ms_bytes), dsmx22ms_output, ARRAY_SIZE(dsmx22ms_output), 9, 16);
    test_protocol("DSMX22_VTX", 115200, dsmx22ms_vtx_bytes, sizeof(dsmx22ms_vtx_bytes), dsmx22ms_vtx_output, ARRAY_SIZE(dsmx22ms_vtx_output), 9, 16);
    test_protocol("DSMX11", 115200, dsmx11ms_bytes, sizeof(dsmx11ms_bytes), dsmx11ms_output, ARRAY_SIZE(dsmx11ms_output), 9, 16);
    test_protocol("DSMX11_VTX", 115200, dsmx11ms_vtx_bytes, sizeof(dsmx11ms_vtx_bytes), dsmx11ms_vtx_output, ARRAY_SIZE(dsmx11ms_vtx_output), 9, 16);

    test_protocol("FPORT", 115200, fport_bytes, sizeof(fport_bytes), fport_output, ARRAY_SIZE(fport_output), 3, 0, true);
    test_protocol("FPORT2_16CH", 115200, fport2_16ch_bytes, sizeof(fport2_16ch_bytes), fport2_16ch_output, ARRAY_SIZE(fport2_16ch_output), 3, 0, true);
    test_protocol("FPORT2_24CH", 115200, fport2_24ch_bytes, sizeof(fport2_24ch_bytes), fport2_24ch_output, ARRAY_SIZE(fport2_24ch_output), 3, 0, true);

    /*
      now test with random data to ensure we don't have any logic bugs that can cause a crash of the parser
     */
    test_random();

    if (test_count++ == 10) {
        if (test_failures == 0) {
            printf("Test PASSED\n");
            ::exit(0);
        }
        printf("Test FAILED - %u failures\n", unsigned(test_failures));
        ::exit(1);
    }
    printf("Test count %u - %u failures\n", unsigned(test_count), unsigned(test_failures));
}

AP_HAL_MAIN();
