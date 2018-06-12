#pragma once 

#include "RC_parser.h"
#include "RCInput.h"

#include <AP_HAL/HAL.h>


enum DSM_STATE {
    S_NONE,
    S_DSM,
    S_SUMD
};

class F4Light::DSM_parser : public F4Light::_parser {
public:
    DSM_parser() {}

    void init(uint8_t ch);
    
    bool bind(int dsmMode) const override;

private:

    static UARTDriver uartSDriver; 

    void add_dsm_uart_input(); // add some DSM input bytes, for RCInput over a serial port
    void _io_completion();
    uint8_t _ioc;
    
    struct DSM { // state of add_dsm_uart_input
        uint8_t frame[16];
        uint8_t partial_frame_count;
        uint64_t last_input_ms;
    } dsm;

    enum DSM_STATE state;
    
    static void _rc_bind(uint16_t dsmMode);    
};

