
#pragma once

#include "AP_ESC.h"
#include "AP_ESC_Backend.h"


#define SINGLE_LOOP_MAX_BYTES 500
#define BUFFER_LOOP_SIZE 44
#define ESC_PACKET_SIZE 22

class AP_ESC_APDHVPro: public AP_ESC_Backend {
public:
    // constructor
    AP_ESC_APDHVPro(AP_ESC &frontend);

    // update the structure
    bool update() override;

private:
    AP_ESC *_frontend;

    AP_HAL::UARTDriver *port;

    uint32_t last_read_ms;

    uint8_t max_buffer[SINGLE_LOOP_MAX_BYTES];
    uint8_t raw_buffer[ESC_PACKET_SIZE];

    bool read_ESC_telemetry_data(uint32_t);
    bool parse_ESC_telemetry_data();
    int check_flectcher16()
};
