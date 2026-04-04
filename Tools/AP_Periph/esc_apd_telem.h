/*
  ESC Telemetry for APD ESC.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if AP_PERIPH_ESC_APD_ENABLED

class ESC_APD_Telem {
public:
    ESC_APD_Telem (AP_HAL::UARTDriver *_uart, float num_poles);
    bool update();

    CLASS_NO_COPY(ESC_APD_Telem);

    struct telem {
        uint32_t error_count;
        float voltage;
        float current;
        float temperature; // kelvin
        int32_t rpm;
        uint8_t power_rating_pct;
    };

    const telem &get_telem(void) {
        return decoded;
    }

private:
    AP_HAL::UARTDriver *uart;

    union {
        struct PACKED {
            uint16_t voltage;
            uint16_t temperature;
            int16_t bus_current;
            uint16_t reserved0;
            uint32_t erpm;
            uint16_t input_duty;
            uint16_t motor_duty;
            uint16_t reserved1;
            uint16_t checksum; // 16 bit fletcher checksum
            uint16_t stop; // should always be 65535 on a valid packet
        } packet;
        uint8_t bytes[22];
    } received;
    static_assert(sizeof(received.packet) == sizeof(received.bytes), "The packet must be the same size as the raw buffer");

    uint8_t len;

    struct telem decoded;

    float pole_count;

    float convert_temperature(uint16_t raw) const;
    void shift_buffer(void);
};

#endif // AP_PERIPH_ESC_APD_ENABLED
