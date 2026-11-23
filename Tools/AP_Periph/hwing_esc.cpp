/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC, using the frame parser
  in AP_ESC_Telem

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */

#include "AP_Periph.h"
#include "hwing_esc.h"
#include <dronecan_msgs.h>

#if AP_PERIPH_HOBBYWING_ESC_ENABLED

#include <SITL/SITL.h>

// constructor
HWESC_Telem::HWESC_Telem(void)
{
}

void HWESC_Telem::init(AP_HAL::UARTDriver *_uart)
{
    uart = _uart;
    uart->begin(AP_ESC_Telem_XRotor_Parser::BAUDRATE);
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

/*
  update ESC telemetry
 */
bool HWESC_Telem::update()
{
    uint32_t n = uart->available();

    // don't read too much in one loop to prevent too high CPU load
    if (n > 500) {
        n = 500;
    }

    bool ret = false;

    while (n--) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        if (parser.parse_byte(b)) {
            decode_frame();
            ret = true;
        }
    }
    return ret;
}

/*
  decode the frame the parser has just received
 */
void HWESC_Telem::decode_frame(void)
{
    const AP_ESC_Telem_XRotor_Parser::Frame &f = parser.get_frame();

    decoded.counter = f.counter;
    decoded.throttle_req = f.throttle_req;
    decoded.throttle = f.throttle;
    decoded.rpm = f.rpm * 5.0 / 7.0; // scale from eRPM to RPM
    decoded.voltage = f.voltage * 0.1;
    decoded.phase_current = f.phase_current * 0.01;
    decoded.current = f.current * 0.01;
    decoded.mos_temperature = AP_ESC_Telem_XRotor_Parser::temperature_decode(f.mos_temperature);
    decoded.cap_temperature = AP_ESC_Telem_XRotor_Parser::temperature_decode(f.cap_temperature);
    decoded.status = f.status;
    if (decoded.status != 0) {
        decoded.error_count++;
    }
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

