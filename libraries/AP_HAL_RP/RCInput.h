#pragma once

#include "AP_HAL_RP.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

class RP::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

private:
    bool _init;

    // For boards with PIO/PPM (if UART is not used)
#ifdef HAL_RP_PULSE_INPUT_ENABLED
    class PulseReader {
    public:
        void init();
        void process();
    } sig_reader;
#endif

    // UART driver for digital protocols (ELRS/CRSF)
    AP_HAL::UARTDriver* _uart;

    // Method for processing the input stream of bytes
    void _process_uart();
};
