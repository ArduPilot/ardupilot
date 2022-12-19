/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC.

 - this driver is used in two different ways:
  - by the main firmware as an AP_ESC_Telem_Backend
  - by AP_Periph using a simple getter for the decoded telemetry data

 */

#pragma once

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#ifndef AP_ESC_SERIAL_TELEM_ENABLED
#define AP_ESC_SERIAL_TELEM_ENABLED HAL_WITH_ESC_TELEM
#endif

#ifndef AP_ESC_SERIAL_TELEM_ESC_TELEM_BACKEND_ENABLED
#define AP_ESC_SERIAL_TELEM_ESC_TELEM_BACKEND_ENABLED (AP_ESC_SERIAL_TELEM_ENABLED && HAL_WITH_ESC_TELEM)
#endif

#if AP_ESC_SERIAL_TELEM_ENABLED

#include <AP_Param/AP_Param.h>

class AP_ESCSerialTelem {
public:
    AP_ESCSerialTelem();

    void init(class AP_HAL::UARTDriver *uart);
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


// AP_ESC_Telem interface
#if HAL_WITH_ESC_TELEM
class AP_ESCSerialTelem_ESCTelem_Backend
    : AP_ESC_Telem_Backend
{
public:

    // constructor
    AP_ESCSerialTelem_ESCTelem_Backend();

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    void update_telemetry();

private:
    AP_Int32 channel_mask;

    static const uint8_t MAX_BACKENDS { 8 };
    AP_ESCSerialTelem *backends[MAX_BACKENDS];
    uint8_t servo_channel[MAX_BACKENDS];

    uint8_t num_backends;
};
#endif  // HAL_WITH_ESC_TELEM

#endif  // AP_ESC_SERIAL_TELEM_ENABLED
