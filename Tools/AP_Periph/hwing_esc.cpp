/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */

#include "AP_Periph.h"
#include "hwing_esc.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <dronecan_msgs.h>

#if AP_PERIPH_HOBBYWING_ESC_ENABLED

#include <SITL/SITL.h>

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
        uart->discard_input();
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
    decoded.rpm = be16toh(pkt.rpm) * 5.0 / 7.0; // scale from eRPM to RPM
    decoded.voltage = be16toh(pkt.voltage) * 0.1;
    decoded.phase_current = int16_t(be16toh(pkt.phase_current)) * 0.01;
    decoded.current = int16_t(be16toh(pkt.current)) * 0.01;
    decoded.mos_temperature = AP_ESC_Telem_XRotor_Parser::temperature_decode(pkt.mos_temperature);
    decoded.cap_temperature = AP_ESC_Telem_XRotor_Parser::temperature_decode(pkt.cap_temperature);
    decoded.status = be16toh(pkt.status);
    if (decoded.status != 0) {
        decoded.error_count++;
    }

    return true;
}

void AP_Periph_FW::hwesc_telem_update()
{
    if (!hwesc_telem.update()) {
        return;
    }
    const HWESC_Telem::HWESC &t = hwesc_telem.get_telem();

    uavcan_equipment_esc_Status pkt {};
    pkt.esc_index = g.esc_number[0]; // only supports a single ESC
    pkt.voltage = t.voltage;
    pkt.current = t.current;
    pkt.temperature = C_TO_KELVIN(MAX(t.mos_temperature, t.cap_temperature));
    pkt.rpm = t.rpm;
    pkt.power_rating_pct = t.phase_current;
    pkt.error_count = t.error_count;

    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer, !canfdout());
    canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}

#endif // AP_PERIPH_HOBBYWING_ESC_ENABLED

