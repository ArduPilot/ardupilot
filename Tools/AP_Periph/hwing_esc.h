/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if AP_PERIPH_HOBBYWING_ESC_ENABLED

class HWESC_Telem {
public:
    HWESC_Telem();

    void init(AP_HAL::UARTDriver *uart);
    bool update();

    struct HWESC {
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        float rpm;
        float voltage;
        float phase_current;
        float current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint32_t error_count;
    };

    const HWESC &get_telem(void) {
        return decoded;
    }

private:
    AP_HAL::UARTDriver *uart;

    struct PACKED {
        uint8_t header; // 0x9B
        uint8_t pkt_len; // 0x16
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        int16_t current;
        int16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint16_t crc;
    } pkt;

    uint8_t len;
    uint32_t last_read_ms;
    uint32_t error_count;

    struct HWESC decoded;

    bool parse_packet(void);
    uint8_t temperature_decode(uint8_t temp_raw) const;
};

#endif // AP_PERIPH_HOBBYWING_ESC_ENABLED
