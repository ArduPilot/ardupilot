/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_PERIPH_ENABLE_HWESC

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
        uint16_t temperature;
        uint16_t status;
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
        uint16_t temperature;
        uint16_t status;
        uint16_t crc;
    } pkt;

    uint8_t len;
    uint32_t last_read_ms;

    struct HWESC decoded;

    bool parse_packet(void);
};

#endif // HAL_PERIPH_ENABLE_HWESC
