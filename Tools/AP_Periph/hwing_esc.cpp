/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */
#include "hwing_esc.h"
#include <AP_HAL/utility/sparse-endian.h>

#ifdef HAL_PERIPH_ENABLE_HWESC

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0x9B
#define TELEM_LEN    0x16

// constructor
HWESC_Telem::HWESC_Telem(void)
{
}

void HWESC_Telem::init(AP_HAL::UARTDriver *_uart)
{
    uart = _uart;
    uart->begin(19200);
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

/*
  update ESC telemetry
 */
bool HWESC_Telem::update()
{
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    // we expect at least 50ms idle between frames
    uint32_t now = AP_HAL::millis();
    bool frame_gap = (now - last_read_ms) > 10;

    last_read_ms = now;

    // don't read too much in one loop to prevent too high CPU load
    if (n > 500) {
        n = 500;
    }
    if (len == 0 && !frame_gap) {
        // discard
        while (n--) {
            uart->read();
        }
        return false;
    }

    if (frame_gap) {
        len = 0;
    }

    bool ret = false;

    while (n--) {
        uint8_t b = uart->read();
        //hal.console->printf("t=%u 0x%02x\n", now, b);
        if (len == 0 && b != TELEM_HEADER) {
            continue;
        }
        if (len == 1 && b != TELEM_LEN) {
            continue;
        }
        uint8_t *buf = (uint8_t *)&pkt;
        buf[len++] = b;
        if (len == sizeof(pkt)) {
            ret = parse_packet();
            len = 0;
        }
    }
    return ret;
}

static uint16_t calc_crc(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc += *buf++;
    }
    return crc;
}

/*
  parse packet
 */
bool HWESC_Telem::parse_packet(void)
{
    uint16_t crc = calc_crc((uint8_t *)&pkt, sizeof(pkt)-2);
    if (crc != pkt.crc) {
        return false;
    }

    decoded.counter = be32toh(pkt.counter);
    decoded.throttle_req = be16toh(pkt.throttle_req);
    decoded.throttle = be16toh(pkt.throttle);
    decoded.rpm = be16toh(pkt.rpm);
    decoded.voltage = be16toh(pkt.voltage) * 0.1;
    decoded.load = int16_t(be16toh(pkt.load));
    decoded.current = int16_t(be16toh(pkt.current)) * 0.01;
    decoded.temperature = be16toh(pkt.temperature);
    decoded.unknown = be16toh(pkt.unknown);

    return true;
}

#endif // HAL_PERIPH_ENABLE_HWESC

