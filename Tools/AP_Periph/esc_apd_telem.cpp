/*
  ESC Telemetry for the APD HV Pro ESC

  Protocol is here: https://docs.powerdrives.net/products/hv_pro/uart-telemetry-output
 */
#include "esc_apd_telem.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_Math/definitions.h>
#include <string.h>

#ifdef HAL_PERIPH_ENABLE_ESC_APD

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0x9B
#define TELEM_LEN    0x16

ESC_APD_Telem::ESC_APD_Telem (AP_HAL::UARTDriver *_uart, float num_poles) :
    pole_count(num_poles),
    uart(_uart) {
    uart->begin(115200);
}

bool ESC_APD_Telem::update() {
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    // don't read too much in one loop to prevent too high CPU load
    if (n > 50) {
        n = 50;
    }

    bool ret = false;

    while (n--) {
        uint8_t b = uart->read();
        received.bytes[len++] = b;

        // check the packet size first
        if ((size_t)len >= sizeof(received.packet)) {
            // we have a full packet, check the stop byte
            if (received.packet.stop == 65535) {
                // valid stop byte, check the CRC
                if (crc_fletcher16(received.bytes, 18) == received.packet.checksum) {
                    // valid packet, copy the data we need and reset length
                    decoded.voltage = le16toh(received.packet.voltage) * 1e-2f;
                    decoded.temperature = convert_temperature(le16toh(received.packet.temperature));
                    decoded.current = ((int16_t)le16toh(received.packet.bus_current)) * (1 / 12.5f);
                    decoded.rpm = le32toh(received.packet.erpm) / pole_count;
                    decoded.power_rating_pct = le16toh(received.packet.motor_duty) * 1e-2f;
                    ret = true;
                    len = 0;
                } else {
                    // we have an invalid packet, shift it back a byte
                    shift_buffer();
                }
            } else {
                // invalid stop byte, we've lost sync, shift the packet by 1 byte
                shift_buffer();
            }
            
        }
    }
    return ret;
}

// shift the decode buffer left by 1 byte, and rewind the progress
void ESC_APD_Telem::shift_buffer(void) {
    memmove(received.bytes, received.bytes + 1, sizeof(received.bytes) - 1);
    len--;
}

// convert the raw ESC temperature to a useful value (in Kelvin)
// based on the 1.1 example C code found here https://docs.powerdrives.net/products/hv_pro/uart-telemetry-output
float ESC_APD_Telem::convert_temperature(uint16_t raw) const {
    const float series_resistor     = 10000;
    const float nominal_resistance  = 10000;
    const float nominal_temperature = 25;
    const float b_coefficent        = 3455;


    const float Rntc = series_resistor / ((4096 / float(raw)) - 1);

    float temperature = Rntc / nominal_resistance;          // (R/Ro)
    temperature = logf(temperature);                        // ln(R/Ro)
    temperature /= b_coefficent;                            // 1/B * ln(R/Ro)
    temperature += 1 / C_TO_KELVIN(nominal_temperature); // + (1/To)
    temperature = 1 / temperature;                          // invert

    // the example code rejected anything below 0C, or above 200C, the 200C clamp makes some sense, the below 0C is harder to accept
    return temperature;
}

#endif // HAL_PERIPH_ENABLE_ESC_APD
