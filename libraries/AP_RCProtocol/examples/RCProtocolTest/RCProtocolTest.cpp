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
#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_RCProtocol *rcprot;

// setup routine
void setup()
{
    // introduction
    hal.console->printf("ArduPilot RC protocol test\n");
    hal.scheduler->delay(100);
}

static bool check_result(const char *name, bool bytes, const uint16_t *values, uint8_t nvalues)
{
    char label[20];
    snprintf(label, 20, "%s(%s)", name, bytes?"bytes":"pulses");
    if (!rcprot->new_input()) {
        printf("%s: No new input\n", label);
        return false;
    }
    if (strcmp(rcprot->protocol_name(), name) != 0) {
        printf("%s: wrong protocol detected %s\n", label, rcprot->protocol_name());
        return false;
    }
    uint8_t n = rcprot->num_channels();
    if (n != nvalues) {
        printf("%s: wrong number of channels %u should be %u\n", label, n, nvalues);
        return false;
    }
    for (uint8_t i=0; i<n; i++) {
        uint16_t v = rcprot->read(i);
        if (values[i] != v) {
            printf("%s: chan %u wrong value %u should be %u\n", label, i+1, v, values[i]);
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
                               uint8_t repeats)
{
    bool ret = true;
    for (uint8_t repeat=0; repeat<repeats+4; repeat++) {
        for (uint8_t i=0; i<nbytes; i++) {
            rcprot->process_byte(bytes[i], baudrate);
        }
        hal.scheduler->delay(10);
        if (repeat > repeats) {
            ret &= check_result(name, true, values, nvalues);
        }
    }
    return ret;
}

static void send_bit(uint8_t bit, uint32_t baudrate)
{
    static uint16_t bits_0, bits_1;
    if (baudrate == 115200) {
        // yes, this is backwards ...
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
static void send_byte(uint8_t b, uint32_t baudrate)
{
    send_bit(0, baudrate); // start bit
    uint8_t parity = 0;
    for (uint8_t i=0; i<8; i++) {
        uint8_t bit = (b & (1U<<i))?1:0;
        send_bit(bit, baudrate);
        if (bit) {
            parity = !parity;
        }
    }
    if (baudrate == 100000) {
        // assume SBUS, send parity
        send_bit(parity, baudrate);
    }
    send_bit(1, baudrate); // stop bit
    if (baudrate == 100000) {
        send_bit(1, baudrate); // 2nd stop bit
    }
}

/*
  test a byte protocol handler
 */
static bool test_pulse_protocol(const char *name, uint32_t baudrate,
                                const uint8_t *bytes, uint8_t nbytes,
                                const uint16_t *values, uint8_t nvalues,
                                uint8_t repeats)
{
    bool ret = true;
    for (uint8_t repeat=0; repeat<repeats+4; repeat++) {
        for (uint16_t i=0; i<8000; i++) {
            send_bit(1, baudrate);
        }
        for (uint8_t i=0; i<nbytes; i++) {
            send_byte(bytes[i], baudrate);
        }
        for (uint16_t i=0; i<8000; i++) {
            send_bit(1, baudrate);
        }
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
                          uint8_t repeats=1)
{
    bool ret = true;

    rcprot = new AP_RCProtocol();
    rcprot->init();
    ret &= test_byte_protocol(name, baudrate, bytes, nbytes, values, nvalues, repeats);
    delete rcprot;

    rcprot = new AP_RCProtocol();
    rcprot->init();
    ret &= test_pulse_protocol(name, baudrate, bytes, nbytes, values, nvalues, repeats);
    delete rcprot;

    return ret;
}

//Main loop where the action takes place
void loop()
{
    const uint8_t srxl_bytes[] = { 0xa5, 0x03, 0x0c, 0x04, 0x2f, 0x6c, 0x10, 0xb4, 0x26,
                                   0x16, 0x34, 0x01, 0x04, 0x76, 0x1c, 0x40, 0xf5, 0x3b };
    const uint16_t srxl_output[] = { 1567, 1502, 1019, 1536, 1804, 2000, 1500 };

    const uint8_t sbus_bytes[] = {0x0F, 0x4C, 0x1C, 0x5F, 0x32, 0x34, 0x38, 0xDD, 0x89,
                                  0x83, 0x0F, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint16_t sbus_output[] = {1562, 1496, 1000, 1530, 1806, 2006, 1494, 1494, 874,
                                    874, 874, 874, 874, 874, 874, 874};

    const uint8_t dsm_bytes[] = {0x00, 0xab, 0x00, 0xae, 0x08, 0xbf, 0x10, 0xd0, 0x18,
                                 0xe1, 0x20, 0xf2, 0x29, 0x03, 0x31, 0x14, 0x00, 0xab,
                                 0x39, 0x25, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff};
    const uint16_t dsm_output[] = {1010, 1020, 1000, 1030, 1040, 1050, 1060, 1070};

    const uint8_t sumd_bytes[] = {0xA8, 0x01, 0x08, 0x2F, 0x50, 0x31, 0xE8, 0x21, 0xA0,
                                  0x2F, 0x50, 0x22, 0x60, 0x22, 0x60, 0x2E, 0xE0, 0x2E,
                                  0xE0, 0x87, 0xC6};
    const uint16_t sumd_output[] = {1597, 1076, 1514, 1514, 1100, 1100, 1500, 1500};

    test_protocol("SRXL", 115200, srxl_bytes, sizeof(srxl_bytes), srxl_output, ARRAY_SIZE(srxl_output), 1);
    test_protocol("SUMD", 115200, sumd_bytes, sizeof(sumd_bytes), sumd_output, ARRAY_SIZE(sumd_output), 1);

    // SBUS needs 3 repeats to pass the RCProtocol 3 frames test
    test_protocol("SBUS", 100000, sbus_bytes, sizeof(sbus_bytes), sbus_output, ARRAY_SIZE(sbus_output), 3);

    // DSM needs 8 repeats, 5 to guess the format, then 3 to pass the RCProtocol 3 frames test
    test_protocol("DSM", 115200, dsm_bytes, sizeof(dsm_bytes), dsm_output, ARRAY_SIZE(dsm_output), 9);
}

AP_HAL_MAIN();
