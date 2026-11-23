/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC, using the frame parser
  in AP_ESC_Telem
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if AP_PERIPH_HOBBYWING_ESC_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem_XRotor_Parser.h>

#if !AP_ESC_TELEM_XROTOR_PARSER_ENABLED
#error "AP_PERIPH_HOBBYWING_ESC_ENABLED needs AP_ESC_TELEM_XROTOR_PARSER_ENABLED"
#endif

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

    AP_ESC_Telem_XRotor_Parser parser;
    uint32_t error_count;

    struct HWESC decoded;

    void decode_frame(void);
};

#endif // AP_PERIPH_HOBBYWING_ESC_ENABLED
